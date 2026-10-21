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
            'motion_socket': '/run/evo/rax-motion.sock', 'enable_motion': False, 'enable_hold_correction': False, 'hold_profile': 'default', 'enable_alignment': False, 'attempt_timeout_s': 125., 'poll_s': .1}
CAMERAS = ('base', 'left_arm', 'right_arm')
class PolicyBusy(RuntimeError):
    pass


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
    mutating = frozenset({'evo_towel_align', 'evo_step', 'evo_hold_correct'})

    def __init__(self, config):
        unknown = set(config.backend_options)-set(DEFAULTS)
        if unknown:
            raise ValueError(f'unknown Evo options: {sorted(unknown)}')
        self.options = {**DEFAULTS, **config.backend_options}
        for name, maximum in [('camera_max_age_s', 1.), ('socket_timeout_s', 5.),
                              ('attempt_timeout_s', 130.), ('poll_s', .5)]:
            v = self.options[name]
            if type(v) not in (int, float) or not 0 < v <= maximum:
                raise ValueError(f'invalid {name}')
        if type(self.options['enable_alignment']) is not bool:
            raise ValueError('enable_alignment must be a boolean')
        for name in ('control_socket', 'camera_directory'):
            if not isinstance(self.options[name], str) or not Path(self.options[name]).is_absolute():
                raise ValueError(f'{name} must be an absolute operator path')
        from services.evo_v3.hold import trial_limit
        trial_limit(self.options['hold_profile'])
        self.config = config
        if type(self.options['enable_hold_correction']) is not bool:
            raise ValueError('invalid hold-correction configuration')
        if type(self.options['enable_motion']) is not bool or not Path(self.options['motion_socket']).is_absolute():
            raise ValueError('invalid motion configuration')
        self.active = False
        self.closed = False

    def tools(self):
        descriptions = {
            'evo_status': 'Read Evo towel-policy status. No startup or motion. State is not measured pose.',
            'evo_observe': 'Save fresh head and wrist JPEGs. Return per-camera errors and capture times. No validated metric geometry or capture-time joints yet.',
            'evo_towel_align': 'Run one configured Evo alignment attempt. Requires operator enablement, arming and protocol v2. Returns the matching attempt result only after verified stop. No ACT or folding.',
            'evo_hold': 'Cancel the Evo towel policy. Always allowed. This is not a global robot stop. An acknowledgement does not prove physical stop; inspect status.'}
        return [{'name':'evo_hold_correct','description':'One fixed-target J4 baseline/hold/withdraw trial through the opt-in Evo hardware driver. Requires recording, operator enablement and arming; driver must match the operator-configured torque profile. Not a grasp command.','inputSchema':{'type':'object','properties':{'arm':{'type':'string','enum':['left','right']}},'required':['arm'],'additionalProperties':False}}, {'name': 'evo_measured_state', 'description': 'Read fresh measured joints and palm FK from Evo; no motion.', 'inputSchema': {'type': 'object', 'properties': {}, 'additionalProperties': False}}, {'name': 'evo_step', 'description': 'Bounded paired palm translation in carriage_link (positive Z down), fixed measured orientation; optional gripper targets. Existing Evo IK and Ruckig. Maximum 3 cm per hand. Returns measured errors and stop proof. Requires recording and operator arming.', 'inputSchema': {'type':'object','properties':{'delta':{'type':'array','minItems':2,'maxItems':2,'items':{'type':'array','minItems':3,'maxItems':3,'items':{'type':'number'}}},'grippers':{'type':'array','minItems':2,'maxItems':2,'items':{'type':'number','minimum':-.6,'maximum':0}},'duration_s':{'type':'number','minimum':2,'maximum':15}},'required':['delta'],'additionalProperties':False}}] + [{'name': n, 'description': d, 'inputSchema': {'type': 'object', 'properties': {},
                  'required': [], 'additionalProperties': False}} for n, d in descriptions.items()]

    def validate(self, name, arguments):
        if name == 'evo_hold_correct':
            if not isinstance(arguments, dict) or set(arguments) != {'arm'} or arguments['arm'] not in ('left', 'right'):
                raise ValueError('hold correction requires one arm')
            if self.closed or not self.options['enable_hold_correction'] or not arming.read(self.config.root)['armed']:
                raise RuntimeError('hold correction requires operator enablement and arming')
            return
        if name == 'evo_step':
            from services.evo_v3.motion import validate_step
            validate_step(arguments)
            if self.closed or not self.options['enable_motion']:
                raise RuntimeError('motion adapter is disabled')
            if not arming.read(self.config.root)['armed']:
                raise RuntimeError('motion requires operator arming')
            return
        if name not in {t['name'] for t in self.tools()} or not isinstance(arguments, dict) or arguments:
            raise ValueError('unknown Evo tool or arguments')
        if self.closed:
            raise RuntimeError('Evo adapter closed')
        if name == 'evo_towel_align':
            if not self.options['enable_alignment']:
                raise RuntimeError('Evo alignment is not enabled by the operator')
            if not arming.read(self.config.root)['armed']:
                raise RuntimeError('Evo alignment requires operator arming')

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
        if isinstance(value, dict) and command == 'start' and value.get('result') == 'busy':
            raise PolicyBusy('Evo policy is busy')
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

    @staticmethod
    def _attempt(value):
        a = value.get('attempt', {})
        if (value.get('protocol') != 'evo.towel.control.v2' or not isinstance(a, dict)
                or not isinstance(a.get('session'), str) or not a['session']
                or any(type(a.get(k)) is not int or a[k] < 0 for k in ('requested', 'completed'))
                or a['completed'] > a['requested']
                or any(type(a.get(k)) is not bool for k in ('running', 'stop_verified', 'cancel_requested'))):
            raise ValueError('Evo attempt protocol v2 is required')
        return a

    def _align(self):
        before = self._attempt(self._command('status'))
        if before['running']:
            raise RuntimeError('Evo policy is busy')
        self.active = True
        try:
            accepted = self._attempt(self._command('start'))
            identity = (accepted['session'], accepted['requested'])
            if identity != (before['session'], before['requested']+1):
                raise RuntimeError('unexpected accepted Evo attempt identity')
            deadline = time.monotonic()+self.options['attempt_timeout_s']
            while time.monotonic() < deadline:
                if not arming.read(self.config.root)['armed']:
                    raise RuntimeError('operator disarmed during Evo alignment')
                value = self._command('status')
                a = self._attempt(value)
                if (a['session'], a['requested']) != identity:
                    raise RuntimeError('Evo restarted or attempt identity changed')
                if not a['running'] and a['completed'] == identity[1]:
                    if not a['stop_verified']:
                        raise RuntimeError('Evo attempt completed without verified stop')
                    if value['state'] not in TERMINAL:
                        raise RuntimeError('completed Evo attempt has nonterminal state')
                    self.active = False
                    return {'ok': value['state'] in {'ReadyForInference', 'ReadyForFoldingSetup'},
                            'measured': value, 'why': value.get('error'),
                            'verification': 'Evo feedback checks; independent physical acceptance is separate'}, []
                time.sleep(self.options['poll_s'])
            raise TimeoutError('Evo attempt deadline expired')
        except PolicyBusy:
            self.active = False
            raise
        except BaseException as error:
            # Never resend start after uncertain acknowledgement or connection loss.
            try:
                self._command('cancel')
                note = 'cancellation acknowledged; physical stop not yet verified'
            except Exception as cancel_error:
                note = f'cancellation failed: {type(cancel_error).__name__}: {cancel_error}'
            self.active = False
            raise RuntimeError(f'{error}; {note}') from error

    def call(self, name, arguments):
        self.validate(name, arguments)
        if name == 'evo_hold_correct':
            from services.evo_v3.hold import run
            result = run(self.config, arguments['arm'], self.config.root/'agent-workspace/runtime/hold-events.jsonl', self.options['hold_profile'])
            return result, []
        if name in ('evo_step', 'evo_measured_state'):
            from services.evo_v3.motion import request
            return request(self.config, self.options['motion_socket'], arguments if name == 'evo_step' else None)
        if name == 'evo_towel_align':
            return self._align()
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
