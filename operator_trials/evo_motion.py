"""Transport for the native bounded-step adapter. No motor control in Python."""
import json
import math
import socket
import time
import uuid
from services import arming


def validate_step(args):
    if not isinstance(args, dict) or set(args) - {'delta', 'grippers', 'duration_s'} or 'delta' not in args:
        raise ValueError('expected delta, optional grippers and duration_s')
    delta = args['delta']
    if not isinstance(delta, list) or len(delta) != 2 or any(not isinstance(d, list) or len(d) != 3 for d in delta):
        raise ValueError('delta must contain two XYZ vectors in carriage_link')
    number = lambda x: type(x) in (int, float) and math.isfinite(x)
    duration = args.get('duration_s', 3.)
    if not number(duration) or not 2 <= duration <= 15:
        raise ValueError('duration outside bounds')
    for d in delta:
        if not all(number(x) for x in d):
            raise ValueError('nonfinite delta')
        length = math.sqrt(sum(x*x for x in d))
        if length > .030001 or 1.5*length/duration > .015:
            raise ValueError('step exceeds distance or speed bound')
    g = args.get('grippers')
    if g is not None and (not isinstance(g, list) or len(g) != 2 or not all(number(x) and -.6 <= x <= 0 for x in g)):
        raise ValueError('invalid grippers')


def request(config, path, arguments=None):
    moving = arguments is not None
    if moving:
        validate_step(arguments)
        if not arming.read(config.root)['armed']:
            raise RuntimeError('motion requires operator arming')
        message = {'id': uuid.uuid4().hex, 'duration_s': 3., **arguments}
        payload = json.dumps(message)
    else:
        payload = 'status'
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
        s.settimeout(2.)
        s.connect(path)
        s.sendall((payload+'\n').encode())
        s.settimeout(.1)
        deadline = time.monotonic() + (arguments.get('duration_s', 3.) + 11 if moving else 2)
        data = b''
        while b'\n' not in data:
            if moving and not arming.read(config.root)['armed']:
                raise RuntimeError('disarmed; connection closed to cancel; stop not verified')
            if time.monotonic() >= deadline:
                raise TimeoutError('motion response expired; do not retry; stop not verified')
            try:
                part = s.recv(65536)
            except socket.timeout:
                continue
            if not part:
                raise RuntimeError('motion adapter disconnected; stop not verified')
            data += part
            if len(data) > 262144:
                raise ValueError('motion response too large')
        result = json.loads(data.split(b'\n', 1)[0])
        if result.get('protocol') != 'evo.rax.motion.v1':
            raise ValueError('wrong motion protocol')
        if moving and (result.get('step_id') != message['id'] or result.get('stop_verified') is not True):
            raise RuntimeError(f'motion did not verify this attempt and stop: {result}')
        return result, []
