"""Host-owned robot/API MCP server: the agent's only path to the robot and to Gemini.

One process per session owns the robot backend (services.backends). It speaks
newline-delimited JSON-RPC (MCP tools) on a private Unix socket, multiplexing the
agent CLI's stdio bridge (harness/claude_cli/mcp_bridge.py) and the agent's own
Python scripts (auto_research_agent/api/runtime.py). Ordinary calls share one
dispatcher, so two clients never start motion at once. Backends can declare
status tools that use a separate dispatcher. Stop tools bypass both dispatchers
and frontend audit I/O; the backend records their execution. Socket reads and image
writes do not occupy a dispatcher. Without --socket it serves one client on stdin/stdout.
"""
import argparse
from contextlib import nullcontext
import json
import os
from pathlib import Path
import signal
import socket
import sys
import threading
import time
import uuid

from services.backends import make_backend
from services.json_io import jsonable
from .billing import reserve_cost
from .config import Configuration
from . import operator
from .errors import public_failure
from .gemini import GeminiRouter, MAX_REQUEST_BYTES, load_key
from .session import Session
from .timing import Timings

PROTOCOL_VERSION = '2025-06-18'


def tool(name, description, properties=None, required=()):
    return {'name': name, 'description': description, 'inputSchema': {'type': 'object',
            'properties': properties or {}, 'required': list(required), 'additionalProperties': False}}


STATUS = tool('status', 'Session task, wall-clock and Gemini budgets, and the live robot summary.')
OPERATOR = tool('request_operator',
    'Ask the human operator for help or for a scene reset, and wait for the answer. kind "reset": the '
    'operator rearranges the scene (e.g. after a cube fell off the table or to start a new attempt) and '
    'replies when done; "help": a question or a problem you cannot solve through the tools (stuck '
    'object, unclear task, robot fault). Say exactly what you need. The robot does nothing while you wait; '
    'observe again after the answer, because the scene may have changed.',
    {'kind': {'type': 'string', 'enum': ['help', 'reset']},
     'message': {'type': 'string', 'maxLength': 4000},
     'timeout_s': {'type': 'number', 'minimum': 30, 'maximum': 3600,
                   'description': 'How long to wait for the answer (default 900).'}},
    ['kind', 'message'])
SKILL_MERGE = tool('skill_library_merge',
    "Merge your session branch of the skill library (the clone in skill-library/) into the library's main, so "
    'later sessions get your skills. Only committed changes count (git add, git commit first). The harness '
    'checks the format and evidence of every skill you changed (see the continual-learning skill) and merges; '
    'it returns problems to fix, or conflicts to resolve by merging origin/main, or the merged commit.',
    {'message': {'type': 'string', 'maxLength': 500,
                 'description': 'What this merge adds or changes, one line.'}}, ['message'])
GEMINI = {
    'name': 'gemini_generate',
    'description': ('OpenRouter-style chat completion via Gemini 3.8 Flash: text/images, JSON schema output and '
                    'client function calling. Returns the complete assistant message plus usage/cost/API budget; '
                    'Gemini spending is capped for the session. Per-request size/output limits apply. Preserve '
                    'assistant messages including reasoning_details when sending tool results. Tool calls are '
                    'data only: your script validates and executes functions. Images: {type: image, mimeType: '
                    'image/png or image/jpeg, data: base64}, or image_url with an inline data URL. No paths, '
                    'remote URLs or server tools. See the gemini skill references/api.md.'),
    'inputSchema': {'type': 'object', 'additionalProperties': False, 'required': ['messages'], 'properties': {
        'messages': {'type': 'array', 'minItems': 1, 'maxItems': 64, 'items': {'type': 'object'},
                     'description': 'Chat messages: system/user/assistant/tool. Tool results use tool_call_id '
                                    'and string content; preserve complete assistant messages.'},
        'max_tokens': {'type': 'integer', 'minimum': 1, 'maximum': 16384},
        'temperature': {'type': 'number', 'minimum': 0, 'maximum': 2},
        'response_format': {'type': 'object', 'description': 'text, json_object, or json_schema with {name, strict, schema}.'},
        'tools': {'type': 'array', 'maxItems': 64, 'items': {'type': 'object'},
                  'description': 'Client function declarations: {type: function, function: {name, description, parameters, strict?}}.'},
        'tool_choice': {'oneOf': [{'type': 'string', 'enum': ['auto', 'none', 'required']}, {'type': 'object'}],
                        'description': 'Or {type: function, function: {name: declared_name}}.'},
        'reasoning': {'type': 'object', 'description': 'OpenRouter reasoning options: effort or max_tokens, optionally enabled/exclude.'},
    }},
}


class Frontend:
    def __init__(self, session, backend, gemini=None, *, started_at=None):
        self.s, self.backend, self.gemini = session, backend, gemini
        self.started_at = started_at
        self.timings = Timings()
        # Interlocks (arming, workspace limits, holds) belong to the backend itself; errors reach
        # the agent whole (services.controller.errors), never as a label such as "control uncertain".
        self._dispatch = threading.Lock()
        self._priority_dispatch = threading.Lock()
        self._closing = threading.Event()
        self._priority_tools = frozenset(getattr(backend, "priority_tools", ()))
        self._stop_tools = frozenset(getattr(backend, "stop_tools", ()))
        self._backend_tools = {t['name']: t for t in backend.tools()}
        reserved = {'status', 'gemini_generate', 'request_operator', 'skill_library_merge'} & set(self._backend_tools)
        # The persistent skill library clone, when the launcher created one for this workspace.
        self.workspace = Path(session.config.root)/'agent-workspace'
        self.library = (self.workspace/'skill-library'/'.git').is_dir()
        if reserved:
            raise ValueError(f'Backend tools may not reuse harness tool names: {sorted(reserved)}')

    def tools(self):
        return [STATUS, OPERATOR, *self._backend_tools.values(), *([GEMINI] if self.gemini else []),
                *([SKILL_MERGE] if self.library else [])]

    def status(self):
        config = self.s.config
        value = {'task': config.task, 'robot': config.robot, 'observation_profile': config.observation_profile,
                 'api_budget': self.s.budget()}
        if self.started_at is not None:
            value['session_seconds'] = config.session_seconds
            value['seconds_since_launch'] = round(time.time() - self.started_at, 1)
        try:
            value['robot_status'] = self.backend.status()
        except Exception as exc:
            value['robot_status_error'] = public_failure(exc, operation='status')
        return value

    def gemini_call(self, arguments, failure):
        failure['stage'] = 'api_validation'
        payload, reservation = self.gemini.prepare(arguments)
        cost_reservation = reserve_cost(reservation, payload['max_tokens'])
        failure['stage'] = 'api_budget'
        self.s.charge(reservation, cost_reservation)  # Durable reservation before billable I/O.
        failure['stage'] = 'provider_request'
        value, used = self.gemini.execute(payload, reservation)
        failure['stage'] = 'api_accounting'
        self.s.refund(reservation - used, value['usage'], cost_reservation)
        value['budget'] = self.s.budget()
        return value, []

    def call(self, name, arguments):
        """One tools/call; returns an MCP CallToolResult."""
        call_id = uuid.uuid4().hex
        failure = {'stage': 'request'}
        priority = isinstance(name, str) and name in self._priority_tools
        stop_request = isinstance(name, str) and name in self._stop_tools
        dispatch = nullcontext() if stop_request else (self._priority_dispatch if priority else self._dispatch)
        with dispatch, self.timings.measure(name if isinstance(name, str) else 'invalid'):
            try:
                if self._closing.is_set():
                    raise RuntimeError('Frontend is closing')
                if not isinstance(arguments, dict):
                    raise ValueError('Arguments must be an object')
                # Evo records stops itself. Frontend audit I/O must never delay a stop.
                if not stop_request:
                    self.s.audit({'event': 'mcp_request', 'call_id': call_id, 'tool': name, 'arguments': arguments})
                if name == 'status':
                    if arguments:
                        raise ValueError('status takes no arguments')
                    value, images = self.status(), []
                elif name == 'request_operator':
                    failure['stage'] = 'validation'
                    if set(arguments) - {'kind', 'message', 'timeout_s'}:
                        raise ValueError('request_operator takes kind, message, timeout_s')
                    timeout = float(arguments.get('timeout_s', 900))
                    if not 30 <= timeout <= 3600:
                        raise ValueError('timeout_s must be 30..3600')
                    request = operator.open_request(self.s.config.root, arguments.get('kind'), arguments.get('message'))
                    print(f"OPERATOR REQUEST [{request['kind']}] {request['message']}", file=sys.stderr, flush=True)
                    failure['stage'] = 'operator_wait'
                    value = operator.wait(self.s.config.root, request, timeout)
                    try:
                        value['robot_status'] = self.backend.status()
                    except Exception as exc:
                        value['robot_status_error'] = public_failure(exc, operation='status')
                    images = []
                elif name == 'skill_library_merge' and self.library:
                    failure['stage'] = 'validation'
                    if set(arguments) - {'message'} or not str(arguments.get('message', '')).strip():
                        raise ValueError('skill_library_merge takes message (one line)')
                    from services import skill_library
                    value, images = skill_library.merge(self.s.config, self.workspace, arguments['message']), []
                elif name == 'gemini_generate' and self.gemini:
                    value, images = self.gemini_call(arguments, failure)
                elif name in self._backend_tools:
                    failure['stage'] = 'validation'
                    validate = getattr(self.backend, 'validate', None)
                    if validate:
                        validate(name, arguments)
                    failure['stage'] = 'robot_execution'
                    value, images = self.backend.call(name, arguments)
                else:
                    raise PermissionError('Unknown tool')
                value = jsonable(value)
                result = {'content': [{'type': 'text', 'text': json.dumps(value, allow_nan=False)}, *images]}
                if not stop_request:
                    self.s.audit({'event': 'mcp_result', 'call_id': call_id, 'tool': name,
                                  'result': result['content'][0]['text'][:65536], 'images': len(images)})
                return result
            except Exception as exc:
                # Expected refusals and handled stops carry their own complete message; anything else
                # also gets its traceback tail.
                handled = isinstance(exc, (ValueError, PermissionError)) or getattr(exc, 'handled_stop', False)
                error = jsonable(public_failure(exc, operation=name, stage=failure['stage'],
                                                traceback_tail=not handled))
                try:
                    if not stop_request:
                        self.s.audit({'event': 'mcp_error', 'call_id': call_id, **error})
                except Exception:
                    error['audit_recording_failed'] = True
                return {'isError': True, 'content': [{'type': 'text', 'text': json.dumps({'error': error})}]}

    def handle(self, request):
        if not isinstance(request, dict):
            return {'jsonrpc': '2.0', 'id': None, 'error': {'code': -32600, 'message': 'Invalid request'}}
        ident = request.get('id')
        try:
            if request.get('jsonrpc') != '2.0':
                raise ValueError('Invalid JSON-RPC')
            method = request.get('method')
            if method == 'initialize':
                result = {'protocolVersion': PROTOCOL_VERSION, 'capabilities': {'tools': {}},
                          'serverInfo': {'name': 'robot', 'version': '1.0'}}
            elif isinstance(method, str) and method.startswith('notifications/'):
                return None
            elif method == 'ping':
                result = {}
            elif method == 'tools/list':
                result = {'tools': self.tools()}
            elif method == 'tools/call':
                params = request['params']
                result = self.call(params['name'], params.get('arguments', {}))
            else:
                raise PermissionError('Unsupported method')
            return {'jsonrpc': '2.0', 'id': ident, 'result': result}
        except Exception as exc:
            failure = public_failure(exc)
            return {'jsonrpc': '2.0', 'id': ident, 'error': {'code': -32000, 'message': failure['reason'], 'data': failure}}

    def _respond(self, line):
        try:
            request = json.loads(line)
        except (ValueError, UnicodeError):
            return {'jsonrpc': '2.0', 'id': None, 'error': {'code': -32700, 'message': 'Invalid request'}}
        return self.handle(request)

    def serve_socket(self, listener, stop=None):
        """Bounded socket workers; stop tools bypass ordinary work and image writes."""
        listener.settimeout(0.1)
        stopping = threading.Event()
        clients = {}
        clients_lock = threading.Lock()
        # Retain admission through response transfer. Slow image readers cannot use
        # every connection slot. Each connection preserves its own request order.
        # An asynchronous action backend must not queue a new motion behind a
        # blocked observation: a stop could otherwise overtake and then precede it.
        ordinary = threading.BoundedSemaphore(1 if self._priority_tools else 4)
        priority = threading.BoundedSemaphore(4)
        stops = threading.BoundedSemaphore(4)

        def respond(line):
            try:
                request = json.loads(line)
            except (ValueError, UnicodeError):
                request = None
                response = {'jsonrpc': '2.0', 'id': None,
                            'error': {'code': -32700, 'message': 'Invalid request'}}
            else:
                response = None
            params = request.get('params') if isinstance(request, dict) else None
            name = params.get('name') if isinstance(params, dict) else None
            urgent = (isinstance(request, dict) and (
                request.get('method') in ('initialize', 'ping', 'tools/list')
                or (request.get('method') == 'tools/call'
                    and isinstance(name, str) and name in self._priority_tools)))
            stop_request = (isinstance(request, dict) and request.get("method") == "tools/call"
                            and isinstance(name, str) and name in self._stop_tools)
            admission = stops if stop_request else (priority if urgent else ordinary)
            if not admission.acquire(blocking=False):
                return None, {'jsonrpc': '2.0',
                              'id': request.get('id') if isinstance(request, dict) else None,
                              'error': {'code': -32000, 'message': 'Frontend workers busy'}}
            try:
                return admission, response if response is not None else self.handle(request)
            except BaseException:
                admission.release()
                raise

        def connection(client):
            pending = bytearray()
            try:
                while not stopping.is_set() and not self._closing.is_set():
                    client.settimeout(0.2)
                    try:
                        block = client.recv(65536)
                    except socket.timeout:
                        continue
                    if not block:
                        return
                    pending.extend(block)
                    if len(pending) > MAX_REQUEST_BYTES:
                        return
                    while b'\n' in pending:
                        if stopping.is_set() or self._closing.is_set():
                            return
                        line, _, tail = pending.partition(b'\n')
                        pending[:] = tail
                        admission, response = respond(line)
                        try:
                            if response is not None:
                                encoded = json.dumps(response, allow_nan=False).encode() + b'\n'
                                client.settimeout(5)
                                client.sendall(encoded)
                        finally:
                            if admission is not None:
                                admission.release()
            except (ConnectionError, OSError):
                pass
            finally:
                with clients_lock:
                    clients.pop(client, None)
                client.close()

        try:
            while not self._closing.is_set() and not (stop and stop.is_set()):
                try:
                    client, _ = listener.accept()
                except socket.timeout:
                    continue
                with clients_lock:
                    if len(clients) >= 32:
                        client.close()
                        continue
                    worker = threading.Thread(target=connection, args=(client,), daemon=True)
                    clients[client] = worker
                worker.start()
        finally:
            stopping.set()
            self._closing.set()
            with clients_lock:
                active = list(clients)
            for client in active:
                try:
                    client.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

    def serve(self, reader, writer):
        while line := reader.readline(MAX_REQUEST_BYTES + 1):
            if len(line.encode('utf-8')) > MAX_REQUEST_BYTES:
                writer.write(json.dumps({'jsonrpc': '2.0', 'id': None,
                    'error': {'code': -32600, 'message': 'Request exceeds wire limit'}}) + '\n')
                writer.flush()
                return  # Do not interpret the remaining fragment as another request.
            response = self._respond(line)
            if response is not None:
                writer.write(json.dumps(response, allow_nan=False) + '\n')
                writer.flush()

    def close(self):
        self._closing.set()
        try:
            self.backend.close()
        finally:
            self.s.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--socket', type=Path, help='Serve this private Unix socket instead of stdio')
    parser.add_argument('--gemini-provider', choices=('openrouter', 'google'), default='openrouter')
    parser.add_argument('--key-file', type=Path, help='OpenRouter key for gemini_generate (optional)')
    parser.add_argument('--started-at', type=float, help='Launch wall-clock start (epoch seconds)')
    args = parser.parse_args()
    config = Configuration.load(args.config)
    key = load_key(args.key_file) if args.key_file else None
    session = Session(config)
    try:
        backend = make_backend(config)
    except BaseException:
        session.close()
        raise
    from .gemini_google import GoogleGeminiRouter
    router = GoogleGeminiRouter if args.gemini_provider == 'google' else GeminiRouter
    frontend = Frontend(session, backend, router(key) if key else None, started_at=args.started_at)
    def stop(*_):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, stop)
    try:
        if args.socket:
            args.socket.unlink(missing_ok=True)
            with socket.socket(socket.AF_UNIX) as listener:
                listener.bind(str(args.socket))
                os.chmod(args.socket, 0o600)
                listener.listen(16)
                print(f'MCP listening on {args.socket}', flush=True)
                frontend.serve_socket(listener)
        else:
            frontend.serve(sys.stdin, sys.stdout)
    finally:
        frontend.close()
        if args.socket:
            args.socket.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
