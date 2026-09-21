"""The test-campaign logger and the mission logger cannot interfere.

Both live in f1tenth_logger and both may run at once. The mission logger
starts with the stack; the test-campaign logger only by hand. Pinned here:

* never started automatically: no launch file or components.yaml in the
  workspace names it, except its own launch file;
* different node names;
* different output folders, neither inside the other;
* no topic the test-campaign side publishes is one the mission logger
  subscribes to or records;
* no module of the test-campaign side imports the mission logger or touches
  its archive, index or lock file.

Plain file and set checks: nothing is spun.
"""

import ast
from pathlib import Path

import pytest
import yaml

from f1tenth_logger.test_campaign import logger_node
from f1tenth_logger.test_campaign.robot_logger import DEFAULT_CAMPAIGN, find_root

PACKAGE_DIR = Path(__file__).resolve().parents[2]
SUBPACKAGE_DIR = PACKAGE_DIR / "f1tenth_logger" / "test_campaign"
OWN_LAUNCH = PACKAGE_DIR / "launch" / "test_campaign_logger.launch.py"
TRIGGER_TOPICS = {"/test/plan_result", "/test/mission_event"}   # trigger.py defaults

mission_logger = pytest.importorskip("f1tenth_logger.mission_logger_node")


def _workspace_src():
    return find_root() / "src"


def test_no_bringup_starts_it_not_even_behind_a_flag():
    offenders = []
    for path in _workspace_src().rglob("*"):
        if path.suffix not in (".py", ".yaml", ".xml") or not path.is_file():
            continue
        if "launch" not in path.name and path.name != "components.yaml":
            continue
        if path.resolve() == OWN_LAUNCH.resolve():
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        if "test_campaign_logger" in text:
            offenders.append(str(path))
    assert offenders == []


def test_the_node_names_differ():
    assert logger_node.NODE_NAME == "test_campaign_logger"
    assert logger_node.NODE_NAME != "mission_logger_node"


def test_the_config_file_is_keyed_to_that_node_name():
    with open(PACKAGE_DIR / "config" / "test_campaign_logger.yaml", encoding="utf-8") as fh:
        config = yaml.safe_load(fh)
    assert list(config) == [logger_node.NODE_NAME]
    params = config[logger_node.NODE_NAME]["ros__parameters"]
    assert params["campaign"] == DEFAULT_CAMPAIGN
    assert "root" not in params, "the launch file resolves root, never the yaml"


def test_the_output_folders_are_disjoint():
    campaign = (find_root() / DEFAULT_CAMPAIGN).resolve()
    runs = Path(mission_logger._default_runs_dir()).expanduser().resolve()
    assert campaign != runs
    assert runs not in campaign.parents and campaign not in runs.parents


def test_nothing_the_campaign_side_publishes_is_read_or_recorded_by_the_mission_logger():
    published = {logger_node.STATUS_TOPIC} | TRIGGER_TOPICS
    mission_side = set(mission_logger._DEFAULT_TOPICS) | {"/mission/status"}
    assert published & mission_side == set()


def test_no_campaign_module_imports_the_mission_logger_or_touches_its_files():
    for path in SUBPACKAGE_DIR.glob("*.py"):
        text = path.read_text(encoding="utf-8")
        for node in ast.walk(ast.parse(text)):
            if isinstance(node, ast.ImportFrom) and node.module:
                names = [node.module] + [f"{node.module}.{a.name}" for a in node.names]
            elif isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            else:
                continue
            for name in names:
                if name.startswith("f1tenth_logger"):
                    assert name.startswith("f1tenth_logger.test_campaign"), (path.name, name)
        for needle in ("f1tenth_archive", "runs.db", "mission_logger.lock",
                       "mission_logger_runs_dir"):
            assert needle not in text, f"{path.name} mentions {needle!r}"


def test_the_trigger_publishes_on_exactly_those_topics_by_default():
    text = (SUBPACKAGE_DIR / "trigger.py").read_text(encoding="utf-8")
    for topic in TRIGGER_TOPICS:
        assert f'default="{topic}"' in text, topic
