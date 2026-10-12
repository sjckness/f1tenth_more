"""missions/ holds hand-written REFERENCE missions; LLM plans never go there.

1d00853 overwrote missions/go_to_person.json and bottle_then_person.json with
LLM-generated plans (mission_id llm_*), which silently changed what three
tests and every run of those missions meant. The planner writes its plans to
missions/llm_generated/ (llm_planner_node._write_mission_file); this test
fails if a translator-shaped mission lands among the reference ones again.
"""

import json
import pathlib

import pytest

MISSIONS = pathlib.Path(__file__).resolve().parents[1] / 'missions'


@pytest.mark.parametrize('path', sorted(MISSIONS.glob('*.json')), ids=lambda p: p.name)
def test_a_reference_mission_is_not_an_llm_plan(path):
    mission = json.loads(path.read_text())
    assert not str(mission.get('mission_id', '')).startswith('llm_'), (
        f'{path.name} carries an LLM mission_id: LLM plans belong in '
        'missions/llm_generated/ or a run folder, never over a reference mission')
