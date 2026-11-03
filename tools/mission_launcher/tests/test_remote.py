"""Command builders and parsers in remote.py; no SSH, tmux, or Qt needed."""

import shlex
from datetime import datetime

from mission_launcher import remote

CONN = remote.Connection(host='gerbil', workspace='~/gerbil-software', domain_id=42)
NOW = datetime(2026, 11, 2, 14, 30, 5)


def test_remote_path_keeps_tilde_expandable():
    assert remote.remote_path('~/gerbil-software') == '"$HOME"/gerbil-software'
    assert remote.remote_path('~/my ws') == '"$HOME"/' + "'my ws'"
    assert remote.remote_path('/opt/ws') == '/opt/ws'


def test_ssh_argv_wraps_script_in_one_bash_argument():
    argv = remote.ssh_argv(CONN, "echo 'hi' && ls")
    assert argv[:5] == ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5']
    assert argv[5] == 'gerbil'
    assert shlex.split(argv[6]) == ['bash', '-c', "echo 'hi' && ls"]


def test_session_names():
    assert remote.mission_session('equipment_servicing') == 'mission_equipment_servicing'
    assert remote.dev_session('gerbil_nav2.launch.py') == 'dev_gerbil_nav2'
    assert remote.dev_session('display.launch.xml') == 'dev_display'
    assert remote.session_name('dev_', 'a.b:c') == 'dev_a_b_c'


def test_parse_sessions_keeps_only_launcher_sessions():
    out = 'rover\nmission_delivery\nbuild\ndev_gerbil_slam\nmy_build\n'
    assert remote.parse_sessions(out) == ['mission_delivery', 'build', 'dev_gerbil_slam']
    assert remote.is_launch_session('mission_delivery')
    assert not remote.is_launch_session('build')


def test_render_launch_args():
    rendered = remote.render_launch_args(
        {'use_mock_hardware': True, 'launch_imu': False, 'foxglove_port': 8765})
    assert rendered == 'use_mock_hardware:=true launch_imu:=false foxglove_port:=8765'
    assert shlex.split(remote.render_launch_args({'name': 'a b'})) == ['name:=a b']


def test_ros_env_script():
    env = remote.ros_env_script(CONN)
    assert env == ('source /opt/ros/humble/setup.bash && '
                   'source "$HOME"/gerbil-software/install/setup.bash && '
                   'export ROS_DOMAIN_ID=42')
    no_domain = remote.Connection(host='gerbil')
    assert 'ROS_DOMAIN_ID' not in remote.ros_env_script(no_domain)


def test_start_launch_script():
    script = remote.start_launch_script(
        CONN, 'mission_delivery', 'gerbil_bringup', 'mission_delivery.launch.py',
        {'use_mock_hardware': True}, now=NOW)
    # Refuses while another launch holds the RoboClaw
    assert "grep -Ex 'mission_.*|dev_.*'" in script
    assert ('ln -sfn "$HOME"/.ros/mission_logs/mission_delivery-20261102-143005.log'
            in script)
    # The tmux command is one quoted argument to bash -c
    tmux = script[script.index('tmux new-session'):].split(' && echo')[0]
    words = shlex.split(tmux)
    assert words[:7] == ['tmux', 'new-session', '-d', '-s', 'mission_delivery',
                         'bash', '-c']
    inner = words[7]
    assert ('ros2 launch gerbil_bringup mission_delivery.launch.py '
            'use_mock_hardware:=true; } 2>&1 | tee -i -a') in inner
    assert inner.startswith('{ source /opt/ros/humble/setup.bash')
    assert '${PIPESTATUS[0]}' in inner


def test_build_is_not_exclusive():
    script = remote.start_session_script(
        CONN, remote.BUILD_SESSION, remote.build_command(CONN, ['gerbil_bringup']),
        exclusive=False, now=NOW)
    assert 'grep -Ex' not in script
    assert 'colcon build --symlink-install --packages-select gerbil_bringup' in script
    assert remote.build_command(CONN).endswith('colcon build --symlink-install')


def test_stop_sessions_script():
    one = remote.stop_sessions_script(['mission_delivery'], timeout=10)
    assert 'targets=mission_delivery' in one
    assert 'C-c' in one and 'seq 1 20' in one and 'kill-session' in one
    every = remote.stop_sessions_script(None)
    assert "grep -Ex 'mission_.*|dev_.*|build'" in every


def test_checks_round_trip():
    script = remote.checks_script(CONN, ['/scan', '/fix'])
    assert 'ros2 topic echo --once /scan' in script
    assert script.rstrip().endswith('wait')
    out = 'OK /scan\nsome warning\nFAIL /fix\n'
    assert remote.parse_checks(out) == {'/scan': True, '/fix': False}


SHOW_ARGS = """Arguments (pass arguments as '<name>:=<value>'):

    'use_mock_hardware':
        Use mock hardware for bench testing
        (default: 'false')

    'use_duty_cycle':
        no description given
        (default: 'true')

    'map':
        Path to the map
        that spans two lines

"""


def test_parse_show_args():
    args = remote.parse_show_args(SHOW_ARGS)
    assert [a.name for a in args] == ['use_mock_hardware', 'use_duty_cycle', 'map']
    assert args[0].default == 'false'
    assert args[0].description == 'Use mock hardware for bench testing'
    assert args[1].description == ''
    assert args[2].default is None
    assert args[2].description == 'Path to the map that spans two lines'


def test_parse_launch_files():
    out = 'gerbil.launch.xml\nREADME.md\nmission_delivery.launch.py\n'
    assert remote.parse_launch_files(out) == ['gerbil.launch.xml',
                                              'mission_delivery.launch.py']


def test_parse_resolved_host():
    out = 'user dave\nhostname 192.168.1.40\nport 22\n'
    assert remote.parse_resolved_host(out, 'gerbil') == '192.168.1.40'
    assert remote.parse_resolved_host('', 'dave@10.0.0.5') == '10.0.0.5'


def test_parse_ping_ms():
    windows = ('Pinging 10.0.0.5 with 32 bytes of data:\n'
               'Reply from 10.0.0.5: bytes=32 time=12ms TTL=64\n\n'
               'Ping statistics for 10.0.0.5:\n'
               '    Minimum = 12ms, Maximum = 12ms, Average = 12ms\n')
    assert remote.parse_ping_ms(windows) == 12.0
    assert remote.parse_ping_ms('Reply from 10.0.0.5: bytes=32 time<1ms TTL=64') == 1.0
    linux = ('64 bytes from 10.0.0.5: icmp_seq=1 ttl=64 time=3.42 ms\n'
             'rtt min/avg/max/mdev = 3.420/3.420/3.420/0.000 ms\n')
    assert remote.parse_ping_ms(linux) == 3.42
    lost_win = ('Request timed out.\n\nPing statistics for 10.0.0.5:\n'
                '    Packets: Sent = 1, Received = 0, Lost = 1 (100% loss),\n')
    assert remote.parse_ping_ms(lost_win) is None
    assert remote.parse_ping_ms('1 packets transmitted, 0 received, 100% packet loss') is None


def test_foxglove_url():
    url = remote.foxglove_url('10.0.0.5', 8765)
    assert url.endswith('ds.url=ws%3A%2F%2F10.0.0.5%3A8765')
