"""RAX adapter to Evo's local towel-policy socket and read-only EVJ1 cameras.

No ROS publisher, hardware SDK, gain setter, startup, homing, or fault reset.
Construction commands nothing. Physical policy guards remain in Evo/Rust.
"""
import base64
import json
import os
from pathlib import Path
import socket
import struct
import time
import uuid

from services import arming

DEFAULTS = {'control_socket': '/run/evo/towel-alignment.sock',
            'camera_directory': '/run/evo/camera-shm',
            'camera_max_age_s': .3, 'socket_timeout_s': 2.,
            }
CAMERAS = ('base', 'left_arm', 'right_arm')
TERMINAL = {'ReadyForInference', 'ReadyForFoldingSetup', 'Failed', 'Cancelled'}


def read_frame(path, max_age_s):
    """Read one coherent EVJ1 generation; never return an old/future frame."""
    for _ in range(10):
        with open(path, 'rb') as f:
            data = f.read(16 * 1024 * 1024)
            f.seek(8)
            generation_after = f.read(8)
        if len(data) < 64 or data[:4] != b'EVJ1':
            raise ValueError('invalid EVJ1 header')
        generation = struct.unpack_from('<Q', data, 8)[0]
        if generation % 2 or generation_after != data[8:16]:
            continue
        width, height, length = struct.unpack_from('<III', data, 24)
        capture_us = struct.unpack_from('<Q', data, 52)[0]
        age = time.time() - capture_us/1e6
        if not 0 <= age <= max_age_s:
            raise ValueError(f'stale or future camera frame: age_s={age:.6f}')
        jpeg = data[64:64+length]
        if not width or not height or length < 4 or len(jpeg) != length or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
            raise ValueError('invalid EVJ1 JPEG payload')
        return jpeg, {'capture_us': capture_us, 'width': width, 'height': height,
                      'generation': generation, 'age_s': age,
                      'geometry_validated': False, 'capture_time_joints': None,
                      'camera_to_base': None}
    raise ValueError('camera generation did not stabilize')


class EvoV3Backend:
    mutating = frozenset()

    def __init__(self, config):
        unknown = set(config.backend_options)-set(DEFAULTS)
        if unknown:
            raise ValueError(f'unknown Evo options: {sorted(unknown)}')
        self.options = {**DEFAULTS, **config.backend_options}
        for name, maximum in [('camera_max_age_s', 1.), ('socket_timeout_s', 5.)]:
            v = self.options[name]
            if type(v) not in (int, float) or not 0 < v <= maximum:
                raise ValueError(f'invalid {name}')
        for name in ('control_socket', 'camera_directory'):
            if not isinstance(self.options[name], str) or not Path(self.options[name]).is_absolute():
                raise ValueError(f'{name} must be an absolute operator path')
        self.config = config
        self.active = False
        self.closed = False

    def tools(self):
        descriptions = {
            'evo_status': 'Read Evo towel-policy status. No startup or motion. State is not measured pose.',
            'evo_observe': 'Save fresh head and wrist JPEGs. Return per-camera errors and capture times. No validated metric geometry or capture-time joints yet.',
            'evo_hold': 'Cancel the Evo towel policy. Always allowed. This is not a global robot stop. An acknowledgement does not prove physical stop; inspect status.'}
        return [{'name': n, 'description': d, 'inputSchema': {'type': 'object', 'properties': {},
                  'required': [], 'additionalProperties': False}} for n, d in descriptions.items()]

    def validate(self, name, arguments):
        if name not in {t['name'] for t in self.tools()} or not isinstance(arguments, dict) or arguments:
            raise ValueError('unknown Evo tool or arguments')
        if self.closed:
            raise RuntimeError('Evo adapter closed')

    def _command(self, command):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            s.settimeout(self.options['socket_timeout_s'])
            s.connect(self.options['control_socket'])
            s.sendall((command+'\n').encode())
            data = bytearray()
            while b'\n' not in data:
                chunk = s.recv(4096)
                if not chunk:
                    raise ConnectionError('Evo socket closed before response')
                data.extend(chunk)
                if len(data) > 1024*1024:
                    raise ValueError('Evo response too large')
        value = json.loads(bytes(data).split(b'\n', 1)[0])
        expected = {'start':'start_requested', 'cancel':'cancel_requested', 'status':'status'}[command]
        if not isinstance(value, dict) or value.get('result') != expected or not isinstance(value.get('state'), str):
            raise ValueError('invalid Evo policy response')
        return value

    def status(self):
        return {'robot': 'evo-v3', 'policy': self._command('status'),
                'arming': arming.read(self.config.root), 'active_call': self.active}

    def _observe(self):
        root = self.config.root/'agent-workspace'/'runtime'/'frames'/uuid.uuid4().hex
        root.mkdir(parents=True, exist_ok=False)
        metadata = {'frame_id': root.name, 'cameras': {}, 'errors': {},
                    'metric_geometry_available': False}
        images = []
        for camera in CAMERAS:
            try:
                jpeg, meta = read_frame(Path(self.options['camera_directory'])/(camera+'.jpeg.shm'), self.options['camera_max_age_s'])
                target = root/(camera+'.jpg'); tmp = target.with_suffix('.tmp')
                tmp.write_bytes(jpeg); os.replace(tmp, target)
                meta['path'] = str(target.relative_to(self.config.root/'agent-workspace'))
                metadata['cameras'][camera] = meta
                images.append({'type': 'image', 'mimeType': 'image/jpeg', 'data': base64.b64encode(jpeg).decode()})
            except (OSError, ValueError) as error:
                metadata['errors'][camera] = str(error)
        stamps = [v['capture_us'] for v in metadata['cameras'].values()]
        metadata['capture_skew_ms'] = (max(stamps)-min(stamps))/1000 if stamps else None
        metadata['ok'] = not metadata['errors'] and metadata['capture_skew_ms'] <= 50
        tmp = root/'meta.tmp'; tmp.write_text(json.dumps(metadata, indent=2)); os.replace(tmp, root/'meta.json')
        return metadata, images

    def call(self, name, arguments):
        self.validate(name, arguments)
        if name == 'evo_observe':
            return self._observe()
        if name == 'evo_hold':
            return {'acknowledgement': self._command('cancel'), 'physical_stop_verified': False}, []
        return self.status(), []

    def close(self):
        if self.closed:
            return
        if self.active:
            self._command('cancel')
        self.closed = True
