"""SSH + tmux command builders for driving the rover from the base station.

Everything here is a pure function that returns either a remote bash script
or the local argv that runs it, so it can be unit-tested without Qt, SSH, or
a rover. The GUI runs the argv with QProcess.

Every launch runs in its own tmux session on the rover, so a WiFi drop or a
closed GUI never kills it:
  mission_<id>   a mission from missions.yaml
  dev_<stem>     any launch file, started from the Dev tab
  build          colcon build
mission_* and dev_* hold the RoboClaw serial port, so only one may run.
"""

from __future__ import annotations

import re
import shlex
import sys
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Iterable, List, Optional, Tuple

MISSION_PREFIX = 'mission_'
DEV_PREFIX = 'dev_'
BUILD_SESSION = 'build'

# ros2_control and the ZED can take a while to shut down on Ctrl-C
DEFAULT_STOP_TIMEOUT = 15.0

# Serializes the "is anything running?" check with the tmux create, so two
# laptops pressing Start together cannot both get through
LOCK_FILE = '~/.ros/mission_launcher.lock'

# Reuse one SSH connection for the 3 s status poll and every command, instead
# of a full handshake each time over a weak field link. Windows' OpenSSH has
# no connection multiplexing, so there each command still connects afresh.
MULTIPLEX = not sys.platform.startswith('win')
MULTIPLEX_OPTIONS = ['-o', 'ControlMaster=auto', '-o', 'ControlPath=~/.ssh/cm-%C',
                     '-o', 'ControlPersist=120']

# A link that dies mid-command would otherwise leave ssh hanging until TCP
# gives up (minutes), freezing the status poll on its last state. Three
# missed 2 s keepalives (~6 s) end it instead. Also covers a mux master whose
# connection went half-open.
KEEPALIVE_OPTIONS = ['-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=3']


@dataclass
class Connection:
    host: str                          # ssh destination: alias, host, or user@host
    workspace: str = '~/domerrover/gerbil-software'   # where add_dev_user.sh clones
    ros_distro: str = 'humble'
    domain_id: Optional[int] = None    # None leaves ROS_DOMAIN_ID unset
    log_dir: str = '~/.ros/mission_logs'


# ---------------------------------------------------------------- quoting

def remote_path(path: str) -> str:
    """Quote a path for bash, keeping a leading ~ expandable."""
    if path == '~':
        return '"$HOME"'
    if path.startswith('~/'):
        return '"$HOME"/' + shlex.quote(path[2:])
    return shlex.quote(path)


def ssh_argv(conn: Connection, script: str, tty: bool = False,
             multiplex: bool = MULTIPLEX) -> List[str]:
    """Local argv that runs a bash script on the rover.

    BatchMode makes ssh fail instead of prompting for a password the GUI
    cannot answer; key-based login is required.

    tty forces a remote pseudo-terminal (-tt, since the GUI has no local
    one). Use it for long-running commands: when the GUI kills its ssh
    client, sshd hangs up the pty and the remote process gets SIGHUP.
    Without one, an idle `tail -F` never notices and stays on the rover.
    """
    argv = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5'] + KEEPALIVE_OPTIONS
    if multiplex:
        argv += MULTIPLEX_OPTIONS
    if tty:
        argv.append('-tt')
    argv += [conn.host, 'bash -c ' + shlex.quote(script)]
    return argv


# ---------------------------------------------------------------- sessions

def session_name(prefix: str, name: str) -> str:
    """tmux session name; tmux treats '.' and ':' as target separators."""
    return prefix + re.sub(r'[^A-Za-z0-9_-]', '_', name)


def mission_session(mission_id: str) -> str:
    return session_name(MISSION_PREFIX, mission_id)


def dev_session(launch_file: str) -> str:
    # Strip only the extension: nav2.sim.launch.py -> nav2.sim, not nav2
    stem = re.sub(r'(\.launch)?\.(py|xml|yaml)$', '', launch_file)
    return session_name(DEV_PREFIX, stem)


def is_launch_session(name: str) -> bool:
    return name.startswith(MISSION_PREFIX) or name.startswith(DEV_PREFIX)


def is_launcher_session(name: str) -> bool:
    return is_launch_session(name) or name == BUILD_SESSION


def parse_sessions(output: str) -> Dict[str, str]:
    """Launcher-owned sessions out of list_sessions_script(): name -> run id.

    The run id is the session's creation time, so stopping and restarting
    the same mission between two polls still reads as a new run.
    """
    sessions = {}
    for line in output.splitlines():
        name, _, created = line.strip().partition(' ')
        if is_launcher_session(name):
            sessions[name] = f'{name}@{created}'
    return sessions


FOXGLOVE_LISTENING = 'FOXGLOVE_LISTENING'
WORKSPACE_MISSING = 'WORKSPACE_MISSING'


def list_sessions_script(foxglove_port: Optional[int] = None,
                         workspace: Optional[str] = None) -> str:
    """tmux sessions, plus whether foxglove_bridge's port is listening and
    whether the workspace is built.

    The port is checked on the rover with `ss` rather than by connecting
    from the laptop: foxglove_bridge logs an ERROR for every client that
    connects without completing its handshake, which every few seconds
    buried the launch log. So it says the bridge is listening, not that the
    laptop can reach it.

    The workspace check catches a wrong Settings path up front: personal
    accounts (add_dev_user.sh) clone under ~/domerrover/, the shared account
    under ~/, and a wrong guess otherwise only shows as a failed Start.
    """
    # No tmux server running means no sessions, not an error
    script = "tmux ls -F '#S #{session_created}' 2>/dev/null || true"
    if foxglove_port:
        script += (f'; ss -ltnH "sport = :{int(foxglove_port)}" 2>/dev/null'
                   f' | grep -q . && echo {FOXGLOVE_LISTENING}; true')
    if workspace:
        script += (f'; test -f {remote_path(workspace)}/install/setup.bash'
                   f' || echo {WORKSPACE_MISSING}; true')
    return script


def parse_foxglove_listening(output: str) -> bool:
    return any(line.strip() == FOXGLOVE_LISTENING for line in output.splitlines())


def parse_workspace_missing(output: str) -> bool:
    return any(line.strip() == WORKSPACE_MISSING for line in output.splitlines())


# ---------------------------------------------------------------- ros env

def ros_env_script(conn: Connection) -> str:
    parts = [
        f'source /opt/ros/{shlex.quote(conn.ros_distro)}/setup.bash',
        f'source {remote_path(conn.workspace)}/install/setup.bash',
    ]
    if conn.domain_id is not None:
        parts.append(f'export ROS_DOMAIN_ID={int(conn.domain_id)}')
    return ' && '.join(parts)


def render_launch_args(args: Dict[str, object]) -> str:
    """{'use_mock_hardware': True} -> "use_mock_hardware:=true"."""
    out = []
    for key, value in args.items():
        if isinstance(value, bool):
            value = 'true' if value else 'false'
        out.append(shlex.quote(f'{key}:={value}'))
    return ' '.join(out)


# ---------------------------------------------------------------- start / stop

def log_paths(conn: Connection, session: str,
              now: Optional[datetime] = None) -> Tuple[str, str]:
    """(this run's log, the <session>.log symlink that always points at it)."""
    stamp = (now or datetime.now()).strftime('%Y%m%d-%H%M%S')
    log_dir = conn.log_dir.rstrip('/')
    return f'{log_dir}/{session}-{stamp}.log', f'{log_dir}/{session}.log'


LAUNCH_SESSION_REGEX = 'mission_.*|dev_.*'


def start_session_script(conn: Connection, session: str, command: str,
                         exclusive: bool = True,
                         now: Optional[datetime] = None) -> str:
    """Run `command` in a detached tmux session, teeing output to a log.

    exclusive refuses to start while any mission_*/dev_* session runs (they
    all hold the RoboClaw) or a build does (it would load half-built files).
    The check and the create run under one flock on the rover, so two GUIs
    pressing Start together cannot both get through. The lock fd is closed
    for tmux, or a tmux server started here would hold the lock forever.

    tee -i ignores the Ctrl-C that stop sends to the whole pane, so it keeps
    logging ros2 launch's shutdown instead of dying first and SIGPIPE-ing it.
    """
    log, latest = log_paths(conn, session, now)
    log_q, latest_q = remote_path(log), remote_path(latest)
    # Grouped so a failing `source` or `cd` is logged too, not just the last
    # command of an && chain
    inner = (
        f'{{ {command}; }} 2>&1 | tee -i -a {log_q}; '
        f'echo "[mission_launcher] exited with code ${{PIPESTATUS[0]}}" >> {log_q}'
    )
    lock = remote_path(LOCK_FILE)
    guard = ''
    if exclusive:
        guard = (
            "busy=$(tmux ls -F '#S' 2>/dev/null | "
            f"grep -Ex '{LAUNCH_SESSION_REGEX}|{BUILD_SESSION}' | paste -sd ' '); "
            'if [ -n "$busy" ]; then '
            'echo "refusing to start: $busy already running" >&2; exit 1; fi; '
        )
    return (
        f'mkdir -p "$(dirname {lock})" && exec 9>{lock} || exit 1; '
        'if ! flock -w 10 9; then '
        'echo "another start is in progress on the rover" >&2; exit 1; fi; '
        + guard +
        f'if tmux has-session -t ={session} 2>/dev/null; then '
        f'echo "session {session} is already running" >&2; exit 1; fi; '
        f'mkdir -p {remote_path(conn.log_dir)} && '
        f'ln -sfn {log_q} {latest_q} && '
        f'tmux new-session -d -s {session} bash -c {shlex.quote(inner)} 9>&- && '
        f'echo "started {session}, logging to {log}"'
    )


def start_launch_script(conn: Connection, session: str, package: str,
                        launch_file: str, args: Dict[str, object],
                        now: Optional[datetime] = None) -> str:
    command = (
        f'{ros_env_script(conn)} && '
        f'ros2 launch {shlex.quote(package)} {shlex.quote(launch_file)} '
        f'{render_launch_args(args)}'
    ).rstrip()
    return start_session_script(conn, session, command, exclusive=True, now=now)


def stop_sessions_script(sessions: Optional[Iterable[str]] = None,
                         timeout: float = DEFAULT_STOP_TIMEOUT) -> str:
    """Ctrl-C each session, wait for a clean exit, kill what is left.

    Ctrl-C first matters: it lets ros2_control stop the motors. A kill skips
    that, and with the RoboClaw serial timeout at 0 the last command latches.
    sessions=None means every launcher session, looked up on the rover.
    """
    if sessions is None:
        targets = ("targets=$(tmux ls -F '#S' 2>/dev/null"
                   f" | grep -Ex '{LAUNCH_SESSION_REGEX}|{BUILD_SESSION}')")
    else:
        targets = 'targets=' + shlex.quote(' '.join(sessions))
    polls = max(1, int(timeout / 0.5))
    return (
        f'{targets}; '
        'if [ -z "$targets" ]; then echo "nothing running"; exit 0; fi; '
        # send-keys takes a pane target: "=name" alone fails, "=name:" works
        'for s in $targets; do tmux send-keys -t "=$s:" C-c; done; '
        f'for i in $(seq 1 {polls}); do '
        'alive=""; '
        'for s in $targets; do tmux has-session -t "=$s" 2>/dev/null && alive="$alive $s"; done; '
        'if [ -z "$alive" ]; then echo "stopped:$targets"; exit 0; fi; '
        'sleep 0.5; done; '
        'for s in $alive; do tmux kill-session -t "=$s"; echo "killed $s (no clean exit)"; done'
    )


def tail_log_script(conn: Connection, session: str, lines: int = 200) -> str:
    _, latest = log_paths(conn, session)
    return f'tail -n {int(lines)} -F {remote_path(latest)} 2>/dev/null'


def attach_argv(conn: Connection, session: str) -> List[str]:
    """Interactive ssh that attaches to a session (for a local terminal)."""
    return ['ssh', '-t', conn.host, f'tmux attach -t ={session}']


# ---------------------------------------------------------------- health checks

CHECK_OK = 'OK'
CHECK_FAIL = 'FAIL'


# One rclpy node that subscribes to every checked topic, instead of one
# `ros2 topic echo` process per topic: a single DDS participant on the Jetson
# while the mission is still starting up. Subscriptions are created as
# discovery finds each topic (Humble cannot subscribe to an unknown type) and
# are raw, so a compressed image is never deserialized. Best effort receives
# from both reliable and best-effort publishers.
CHECK_PROGRAM = '''\
import sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

timeout, topics = float(sys.argv[1]), sys.argv[2:]
rclpy.init()
node = rclpy.create_node('mission_launcher_check')
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
got, subs = set(), {}
deadline = time.monotonic() + timeout
while time.monotonic() < deadline and len(got) < len(topics):
    types = dict(node.get_topic_names_and_types())
    for t in topics:
        if t not in subs and types.get(t):
            subs[t] = node.create_subscription(
                get_message(types[t][0]), t, lambda _m, t=t: got.add(t), qos, raw=True)
    rclpy.spin_once(node, timeout_sec=0.2)
for t in topics:
    print(('OK ' if t in got else 'FAIL ') + t)
'''


def checks_script(conn: Connection, topics: Iterable[str],
                  timeout: float = 15.0) -> str:
    """Wait up to `timeout` for one message on each topic.

    Prints one 'OK <topic>' or 'FAIL <topic>' line per topic.
    """
    args = ' '.join(shlex.quote(t) for t in topics)
    return (
        f'{ros_env_script(conn)} || exit 1\n'
        f"python3 - {timeout:g} {args} <<'MISSION_LAUNCHER_CHECK'\n"
        f'{CHECK_PROGRAM}'
        'MISSION_LAUNCHER_CHECK\n'
    )


def parse_checks(output: str) -> Dict[str, bool]:
    results = {}
    for line in output.splitlines():
        status, _, topic = line.strip().partition(' ')
        if status in (CHECK_OK, CHECK_FAIL) and topic:
            results[topic] = status == CHECK_OK
    return results


# ---------------------------------------------------------------- dev tab

def list_launch_files_script(conn: Connection, package: str) -> str:
    pkg = shlex.quote(package)
    return (
        f'{ros_env_script(conn)} && '
        f'ls "$(ros2 pkg prefix {pkg})/share/"{pkg}/launch'
    )


def parse_launch_files(output: str) -> List[str]:
    return sorted(
        name.strip() for name in output.splitlines()
        if re.search(r'\.launch\.(py|xml|yaml)$', name.strip())
    )


def show_args_script(conn: Connection, package: str, launch_file: str) -> str:
    return (
        f'{ros_env_script(conn)} && '
        f'ros2 launch {shlex.quote(package)} {shlex.quote(launch_file)} --show-args'
    )


@dataclass
class LaunchArg:
    name: str
    default: Optional[str]      # None = required, no default
    description: str = ''


def parse_show_args(output: str) -> List[LaunchArg]:
    """Parse `ros2 launch ... --show-args`, whose entries look like:

        'use_mock_hardware':
            Use mock hardware for simulation
            (default: 'false')
    """
    args: List[LaunchArg] = []
    current: Optional[LaunchArg] = None
    for raw in output.splitlines():
        name = re.match(r"^\s+'([^']+)':\s*$", raw)
        if name:
            current = LaunchArg(name.group(1), None)
            args.append(current)
            continue
        if current is None or not raw.strip():
            continue
        default = re.match(r"^\s+\(default: '(.*)'\)\s*$", raw)
        if default:
            current.default = default.group(1)
        elif raw.strip() != 'no description given':
            sep = ' ' if current.description else ''
            current.description += sep + raw.strip()
    return args


def git_script(conn: Connection, action: str) -> str:
    commands = {
        'status': 'git status -sb && git log --oneline -5',
        'pull': 'git pull --ff-only',
    }
    return f'cd {remote_path(conn.workspace)} && {commands[action]}'


def build_command(conn: Connection, packages: Iterable[str] = ()) -> str:
    """colcon build in the workspace, for start_session_script()."""
    selected = [shlex.quote(p) for p in packages if p]
    select = ' --packages-select ' + ' '.join(selected) if selected else ''
    return (
        f'cd {remote_path(conn.workspace)} && '
        f'source /opt/ros/{shlex.quote(conn.ros_distro)}/setup.bash && '
        f'colcon build --symlink-install{select}'
    )


# ---------------------------------------------------------------- network

def resolve_host_argv(conn: Connection) -> List[str]:
    """`ssh -G` prints the effective config, so ~/.ssh/config aliases resolve."""
    return ['ssh', '-G', conn.host]


def parse_resolved_host(output: str, fallback: str) -> str:
    for line in output.splitlines():
        key, _, value = line.partition(' ')
        if key == 'hostname' and value.strip():
            return value.strip()
    return fallback.rsplit('@', 1)[-1]


def ping_argv(hostname: str, windows: bool) -> List[str]:
    if windows:
        return ['ping', '-n', '1', '-w', '1000', hostname]
    return ['ping', '-c', '1', '-W', '1', hostname]


def parse_ping_ms(output: str) -> Optional[float]:
    """Round-trip time from Windows or Linux ping output, None if lost.

    Matches 'time=3.2 ms', 'time<1ms', and localized forms like 'Zeit=3ms':
    a lost or unreachable reply has no '=<n>ms' before the summary.
    """
    match = re.search(r'[=<]\s*([\d.]+)\s*ms\b', output)
    return float(match.group(1)) if match else None


def foxglove_url(hostname: str, port: int) -> str:
    from urllib.parse import quote
    ws = quote(f'ws://{hostname}:{port}', safe='')
    return f'https://app.foxglove.dev/~/view?ds=foxglove-websocket&ds.url={ws}'
