import pytest
from services.controller.gemini_google import GoogleGeminiRouter, MODEL


def test_google_wire_contract_and_missing_cost():
    response={'model':MODEL,'usage':{'total_tokens':12},'choices':[{'message':{'role':'assistant','content':'visible'},'finish_reason':'stop'}]}
    router=GoogleGeminiRouter('not-a-real-key',send=lambda _:response)
    payload,reserve=router.prepare({'messages':[{'role':'user','content':'Describe visible edges.'}],'max_tokens':128})
    assert payload['model']==MODEL and 'provider' not in payload
    value,used=router.execute(payload,reserve)
    assert used==12 and value['cost_usd'] is None
    assert 'not-a-real-key' not in str(payload)+str(value)
    with pytest.raises(ValueError,match='reasoning'):
        router.prepare({'reasoning':{'effort':'low'}})


def test_google_rejects_other_model():
    router=GoogleGeminiRouter('not-a-real-key',send=lambda _: {'model':'unexpected'})
    with pytest.raises(ValueError,match='unexpected model'):router.execute({},100)


def test_explicit_gemini_key_is_hidden_from_agent(tmp_path):
    import json
    from types import SimpleNamespace
    from harness.claude_cli.launcher import protected_paths
    key=tmp_path/'outside-credentials'/'key'
    (tmp_path/'model-relay.json').write_text(json.dumps({'gemini_key_file':str(key)}))
    assert key.resolve() in protected_paths(SimpleNamespace(root=tmp_path))
