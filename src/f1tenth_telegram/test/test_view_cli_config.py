"""Rendering, the --cli front end and the config/token loading."""

import io
import os

from f1tenth_telegram import cli, config, view
from f1tenth_telegram.controller import Controller, Panel
import pytest

from test_controller import FakeTransport, status_answer

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_render_connected_panel():
    p = Panel('Orin', reach={'online': True}, connected=True,
              status=status_answer(stack='running', logger='crashed', ready=True,
                                   latched=True))
    p.status['processes']['logger']['exit_status'] = 1
    p.status['processes']['planner'] = {'state': 'running', 'managed': False}
    text, rows = view.render(p)
    assert '🟢 online' in text and 'READY ✅' in text
    assert 'stack 🟢 running' in text and 'logger 🔴 crashed (exit 1)' in text
    assert 'planner 🟠 running (not started by the bot)' in text
    assert 'e-stop LATCHED' in text and 'scan 40 Hz' in text
    labels = [b.label for b in rows[0]]
    assert labels == ['⏹ Stack', '⏹ Planner', '▶️ Logger']


def test_render_not_checked_yet():
    text, rows = view.render(Panel('Orin'))
    assert 'not checked yet' in text and [[b.action for b in r] for r in rows] == [['refresh']]


def test_cli_scripted_sequence():
    cfg = config.from_dict({})
    fake = FakeTransport()
    out = io.StringIO()
    rc = cli.run(Controller(cfg, fake), ['refresh', 'connect', 'start:stack', 'confirm',
                                         'stop:stack', 'nonsense', 'killall', 'cancel'],
                 out=out)
    text = out.getvalue()
    assert rc == 0
    assert ['start', 'stack'] in fake.calls and ['killall'] not in fake.calls
    assert 'really start the stack?' in text and 'stack stopped' in text
    assert 'unknown action "nonsense"' in text and '✖ cancelled' in text


def test_cli_interactive_numbers_and_quit():
    cfg = config.from_dict({})
    fake = FakeTransport()
    answers = iter(['1', '9', 'q'])   # 1 = Connect; 9 = no such button
    out = io.StringIO()
    rc = cli.run(Controller(cfg, fake), None, out=out, inp=lambda _p: next(answers))
    assert rc == 0 and ['connect'] in fake.calls and '? type a button number' in out.getvalue()


def test_config_defaults_and_errors():
    cfg = config.from_dict({})
    assert cfg.target.host == 'jetson-orin' and cfg.allowed_user_ids == []
    assert cfg.max_command_age_s == 10.0 and cfg.timeout('estop') == 15.0
    assert cfg.timeout('whatever') == cfg.timeouts['default']
    with pytest.raises(config.ConfigError, match='unknown keys'):
        config.from_dict({'target': {'hots': 'x'}})
    with pytest.raises(config.ConfigError, match='numeric'):
        config.from_dict({'telegram': {'allowed_user_ids': ['@me']}})


@pytest.mark.parametrize('name', ['telegram.yaml', 'telegram_sim.yaml'])
def test_shipped_configs_load(name):
    cfg = config.load(os.path.join(PKG, 'config', name))
    assert cfg.target.ctl_command.endswith('f1tenth_ctl/bin/f1tenth_ctl')
    assert cfg.max_command_age_s == 10.0


def test_token_from_environment_wins(tmp_path):
    cfg = config.from_dict({'telegram': {'token_env_file': str(tmp_path / 'none')}})
    assert config.read_token(cfg, {'TELEGRAM_BOT_TOKEN': ' abc '}) == 'abc'


def test_token_file_must_be_private(tmp_path):
    f = tmp_path / 'env'
    f.write_text('# bot\nexport TELEGRAM_BOT_TOKEN="123:xyz"\n')
    cfg = config.from_dict({'telegram': {'token_env_file': str(f)}})
    f.chmod(0o644)
    with pytest.raises(config.ConfigError, match='chmod 600'):
        config.read_token(cfg, {})
    f.chmod(0o600)
    assert config.read_token(cfg, {}) == '123:xyz'


def test_token_missing(tmp_path):
    cfg = config.from_dict({'telegram': {'token_env_file': str(tmp_path / 'none')}})
    with pytest.raises(config.ConfigError, match='no TELEGRAM_BOT_TOKEN'):
        config.read_token(cfg, {})
