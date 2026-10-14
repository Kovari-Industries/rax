"""Workspace generation for the real-robot agent: template, skills, blocks and contract."""
import json
from pathlib import Path
import re

import pytest

from services.workspace import ROBOT_FACTS, Setup, build, recipe_for, render
from services.workspace.templates import registry

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('profile', ['rgbd', 'rgb-only'])
def test_franka_workspace_renders_without_leftover_markup(tmp_path, profile):
    workspace = tmp_path/'ws'
    manifest = build(recipe_for(), Setup('franka', 'stack_cups', 'claude', profile), workspace)
    for path in workspace.rglob('*.md'):
        text = path.read_text()
        assert '{{' not in text and '{%' not in text, path
        assert not re.search(r'[Rr]obo[Dd]ojo|BEHAVIOR|rehears|formal batch|container', text), path
    skills = sorted(p.name for p in (workspace/'.agents/skills').iterdir())
    assert {'manual-robot-control', 'experience-memory', 'wrist-camera-inspection', 'gemini'} <= set(skills)
    contract = json.loads((workspace/'MCP_CONTRACT.json').read_text())
    assert contract['robot'] == 'franka' and 'status' in contract['tools']
    assert set(ROBOT_FACTS['franka']['tools']) <= set(contract['tools'])
    assert manifest['contract'] == contract
    assert 'real hardware' in (workspace/'MCP_SESSION.md').read_text()
    assert 'Franka FR3' in (workspace/'AGENTS.md').read_text()


def test_subagent_text_only_with_subagents(tmp_path):
    build(recipe_for(), Setup('franka', subagents=0), tmp_path/'none')
    build(recipe_for(), Setup('franka', subagents=2), tmp_path/'two')
    assert 'subagents' not in (tmp_path/'none/AGENTS.md').read_text()
    assert 'up to 2 subagents' in (tmp_path/'two/AGENTS.md').read_text()


def test_blocks_registry_and_unknown_names_fail():
    assert set(registry()) == {'session.contract-pointer', 'images.forwarding'}
    with pytest.raises(ValueError, match='Unknown workspace template name'):
        render('{{ nonexistent }}', Setup('franka'))
    with pytest.raises(ValueError, match='Unknown workspace block'):
        render('{{ block("missing") }}', Setup('franka'))


def test_every_template_variable_is_a_setup_fact():
    """Sources only use names Setup.context() and vocab provide (StrictUndefined renders all)."""
    for path in [*ROOT.glob('auto_research_agent/*.md'), *ROOT.glob('agent_skills/**/*.md'),
                 *ROOT.glob('agent_blocks/**/*.md')]:
        if path.name == 'README.md' and path.parent.name == 'agent_skills':
            continue
        render(path.read_text(), Setup('franka'))
