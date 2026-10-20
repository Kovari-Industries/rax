"""Google Gemini transport using the existing validated image/request contract.

Google does not return billed USD here. The harness retains its full cost reservation;
that reservation is an accounting estimate, not a provider-enforced price ceiling.
"""
import json
import urllib.request
from .gemini import GeminiRouter

MODEL = 'gemini-3.8-flash'
ENDPOINT = 'https://generativelanguage.googleapis.com/v1beta/openai/chat/completions'


class GoogleGeminiRouter(GeminiRouter):
    def prepare(self, arguments):
        if 'reasoning' in arguments:
            raise ValueError('Google transport does not accept OpenRouter reasoning fields')
        payload, reservation = super().prepare(arguments)
        payload.pop('provider')
        payload['model'] = MODEL
        return payload, reservation

    def _http(self, payload):
        request = urllib.request.Request(ENDPOINT, data=json.dumps(payload).encode(), headers={
            'Authorization': f'Bearer {self._key}', 'Content-Type': 'application/json'})
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self, *args, **kwargs):
                return None
        with urllib.request.build_opener(NoRedirect).open(request, timeout=60) as response:
            raw = response.read(2*1024*1024+1)
        if len(raw) > 2*1024*1024:
            raise ValueError('Google response exceeds 2 MiB')
        return json.loads(raw)

    def execute(self, payload, reservation):
        response = self._send(payload)
        if response.get('model') != MODEL:
            raise ValueError('Google returned an unexpected model')
        usage = response.get('usage', {})
        used = usage.get('total_tokens')
        if type(used) is not int or not 0 <= used <= reservation:
            raise ValueError('Missing or excessive Google usage; reservation retained')
        choices = response.get('choices')
        if not isinstance(choices, list) or len(choices) != 1 or not isinstance(choices[0].get('message'), dict):
            raise ValueError('Invalid Google choices')
        return {'model': MODEL, 'provider': 'google', 'message': choices[0]['message'],
                'finish_reason': choices[0].get('finish_reason'), 'usage': usage,
                'cost_usd': None, 'cost_note': 'Google billed USD unavailable; full reservation retained'}, used
