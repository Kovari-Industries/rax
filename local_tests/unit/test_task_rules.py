"""Task rules summarized from the official task code, injected into TASK.md on request."""
from pathlib import Path
import re

import pytest

from services.robodojo import task_rubrics
from services.robodojo.task_rubrics import RULES_DIRECTORY, task_rubric_context, task_rules_summary

PROJECT = Path(__file__).resolve().parents[2]
TASKS = sorted(p.stem for p in (PROJECT/'RoboDojo/task/RoboDojo/config').glob('*.yml') if not p.stem.startswith('_'))
# Brief rubric summaries: what is checked, not how it is measured or how scenes vary.
SECTIONS = ('**Goal:**', '**Success requires, in order:**', '**Success requires, all at the same step:**',
            '**Success requires:**',  # Trigger-based success (e.g. judged when a bottle returns upright).
            '**Fails the episode if at any step:**',
            '**Counted exactly:**', '**Partial credit (failed episodes):**', '**Rules:**')


def test_every_installed_task_has_a_summary():
    assert TASKS, 'RoboDojo task configs are required'
    missing = [task for task in TASKS if not any(p.stem.lower() == task.lower() for p in RULES_DIRECTORY.glob('*.md'))]
    assert not missing
    assert len(list(RULES_DIRECTORY.glob('*.md'))) == len(TASKS)


@pytest.mark.parametrize('path', sorted(RULES_DIRECTORY.glob('*.md')), ids=lambda p: p.stem)
def test_summaries_are_text_in_the_agreed_format(path):
    text = path.read_text()
    # Summaries, not exported code: no identifiers, paths or source excerpts.
    for forbidden in (r'reward_manager', r'func_parser', r'\bdef ', r'\.py\b', r'\bis_[a-z_]+\(', r'self\.', r'```'):
        assert not re.search(forbidden, text), f'{path.name}: {forbidden}'
    headings = re.findall(r'^\*\*[^*]+:\*\*', text, flags=re.M)
    assert headings and set(headings) <= set(SECTIONS), (path.name, headings)
    if path.stem.lower().endswith('_random') and text.startswith('**Rules:**'):
        assert re.match(r'\*\*Rules:\*\* identical to `?[A-Za-z_]+`?\.', text)
    else:
        assert text.startswith('**Goal:**') and '**Success requires' in text, path.name
    # The idea of each rule, never its exact values: digits only as ordered-stage markers.
    assert not re.findall(r'\d', re.sub(r'^\s*\d+\. ', '', text, flags=re.M)), f'{path.name} gives numbers'
    stages = len(re.findall(r'^\d+\. ', text, flags=re.M))
    assert len(text.splitlines()) <= (30 if stages > 8 else 20), f'{path.name} is not brief'
    # One check stage is a set of simultaneous conditions, never a numbered sequence.
    single = re.search(r'\*\*Success requires, all at the same step:\*\*\n(.*?)(?:\n\n|\Z)', text, re.S)
    if single:
        assert not re.search(r'^\d+\.', single.group(1), re.M), f'{path.name}: numbered conditions in one stage'
    assert not ('**Success requires, in order:**' in text and single), path.name


def test_code_summary_is_added_only_when_selected():
    public = task_rubric_context('make_kong', include_source_url=False)
    assert 'Task rules (summarized from the official task code)' not in public
    extended = task_rubric_context('make_kong', include_source_url=False, task_rules='code-summary')
    assert extended.startswith(public)
    assert 'How the official evaluator decides' in extended and task_rules_summary('make_kong').strip() in extended
    with pytest.raises(ValueError, match='task_rules'):
        task_rubric_context('make_kong', task_rules='source-code')


def test_identical_variant_expands_to_base_rules_and_its_own_scene(tmp_path, monkeypatch):
    (tmp_path/'demo.md').write_text('**Goal:** Do it.\n\n**Limits:** 100 steps (4 s).\n\n**Scene variation:** base scene.\n')
    (tmp_path/'demo_random.md').write_text('**Rules:** identical to demo.\n\n**Scene variation:** clutter.\n')
    monkeypatch.setattr(task_rubrics, 'RULES_DIRECTORY', tmp_path)
    text = task_rules_summary('demo_random')
    assert text.startswith('**Goal:** Do it.') and '**Limits:** 100 steps' in text
    assert 'base scene' not in text and '**Scene variation:** clutter.' in text


def test_operator_settings_validate_task_rules(tmp_path):
    from dataclasses import replace
    from test_controller_backend import configuration
    from services.dashboard.sessions import ResearchSessions
    config = configuration(tmp_path)
    assert replace(config, task_rules='code-summary').task_rules == 'code-summary'
    with pytest.raises(ValueError, match='task_rules'):
        replace(config, task_rules='everything')
    sessions = ResearchSessions(tmp_path, lambda: ['make_kong'])
    assert sessions.settings({'task_rules': 'code-summary'})['task_rules'] == 'code-summary'
    with pytest.raises(ValueError, match='task_rules'):
        sessions.settings({'task_rules': 'everything'})
