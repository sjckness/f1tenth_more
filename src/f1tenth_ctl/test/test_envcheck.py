"""The env file against the target's shell: normalisation, diff, the reference shell."""

from f1tenth_ctl import config, envcheck

ORIN_SHELL = {'ROS_DISTRO': 'humble', 'ROS_DISCOVERY_SERVER': '127.0.0.1:11811',
              'ROS_SUPER_CLIENT': 'TRUE', 'ROS_LOCALHOST_ONLY': '0',
              'AMENT_PREFIX_PATH': '/ws/install/a:/opt/ros/humble',
              'LD_LIBRARY_PATH': '/cuda:/opt/ros/humble/lib'}


def test_explicit_defaults_equal_unset_ones():
    env_file = dict(ORIN_SHELL, ROS_DOMAIN_ID='0', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    assert envcheck.diff(env_file, ORIN_SHELL) == []


def test_super_client_spellings():
    assert envcheck.diff(dict(ORIN_SHELL, ROS_SUPER_CLIENT='true'), ORIN_SHELL) == []
    d = envcheck.diff({k: v for k, v in ORIN_SHELL.items() if k != 'ROS_SUPER_CLIENT'},
                      ORIN_SHELL)
    assert d == [{'key': 'ROS_SUPER_CLIENT', 'env_file': None, 'shell': 'TRUE'}]


def test_graph_changing_differences_are_reported():
    env_file = dict(ORIN_SHELL, ROS_DOMAIN_ID='42', ROS_DISCOVERY_SERVER='10.42.0.2:11811')
    keys = [d['key'] for d in envcheck.diff(env_file, ORIN_SHELL)]
    assert keys == ['ROS_DOMAIN_ID', 'ROS_DISCOVERY_SERVER']


def test_path_differences_name_the_entries():
    env_file = dict(ORIN_SHELL, LD_LIBRARY_PATH='/opt/ros/humble/lib')
    d = envcheck.diff(env_file, ORIN_SHELL)
    assert d == [{'key': 'LD_LIBRARY_PATH', 'only_env_file': [], 'only_shell': ['/cuda']}]
    swapped = dict(ORIN_SHELL, AMENT_PREFIX_PATH='/opt/ros/humble:/ws/install/a')
    assert envcheck.diff(swapped, ORIN_SHELL)[0]['order_differs'] is True


def _settings(tmp_path, body):
    path = tmp_path / 't.env'
    path.write_text('\n'.join(f'F1T_{n.upper()}_CMD="sleep 1"' for n in config.MANAGED)
                    + '\n' + body)
    return config.load(str(path))


def test_snippet_reference_shell(tmp_path):
    s = _settings(tmp_path, 'ROS_DOMAIN_ID=42\nROS_DISCOVERY_SERVER=127.0.0.1:11811\n'
                            'F1T_REF_DOMAIN=42\n'
                            "F1T_SHELL_REF='ROS_DOMAIN_ID=$F1T_REF_DOMAIN; "
                            "ROS_DISCOVERY_SERVER=127.0.0.1:11811'\n")
    item = envcheck.check(s)
    assert item['ok'] and 'same ROS environment' in item['value']
    s = _settings(tmp_path, 'ROS_DOMAIN_ID=42\n'
                            "F1T_SHELL_REF='ROS_DOMAIN_ID=7'\n")
    item = envcheck.check(s)
    assert not item['ok'] and not item['required']
    assert item['differences'] == [{'key': 'ROS_DOMAIN_ID', 'env_file': '42', 'shell': '7'}]


def test_interactive_reference_reads_bashrc(tmp_path, monkeypatch):
    home = tmp_path / 'home'
    home.mkdir()
    (home / '.bashrc').write_text('export ROS_DISCOVERY_SERVER=127.0.0.1:11811\n'
                                  'export ROS_SUPER_CLIENT=TRUE\necho noise\n')
    monkeypatch.setenv('HOME', str(home))
    s = _settings(tmp_path, 'ROS_DISCOVERY_SERVER=127.0.0.1:11811\n')
    item = envcheck.check(s)
    assert not item['ok'] and [d['key'] for d in item['differences']] == ['ROS_SUPER_CLIENT']
    s = _settings(tmp_path, 'ROS_DISCOVERY_SERVER=127.0.0.1:11811\nROS_SUPER_CLIENT=TRUE\n')
    assert envcheck.check(s)['ok']


def test_check_can_be_switched_off(tmp_path):
    assert envcheck.check(_settings(tmp_path, 'F1T_SHELL_REF=off\n')) is None
