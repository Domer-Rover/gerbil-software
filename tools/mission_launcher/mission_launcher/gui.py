"""PyQt5 mission launcher: start, watch, and stop rover launches over SSH.

The GUI holds no rover state of its own. It polls tmux on the rover, so
closing and reopening it (or a second laptop) picks up whatever is running.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
from typing import Callable, Dict, List, Optional

from PyQt5.QtCore import QProcess, QSettings, Qt, QTimer, QUrl
from PyQt5.QtGui import QDesktopServices, QFont
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDialog, QDialogButtonBox, QFormLayout,
    QGridLayout, QGroupBox, QHBoxLayout, QInputDialog, QLabel, QLineEdit,
    QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QScrollArea, QSpinBox, QSplitter,
    QTabWidget, QVBoxLayout, QWidget,
)

from . import remote
from .config import Mission, Registry

WINDOWS = sys.platform.startswith('win')

STATUS_INTERVAL_MS = 3000
PING_INTERVAL_MS = 2000
CHECK_DELAY_MS = 20000          # let the ZED and EKF come up before checking
UNREACHABLE_AFTER = 2           # consecutive failed polls before "unreachable"
COMMAND_TIMEOUT_MS = 45000      # stop waits up to 15 s on the rover, plus margin
SLOW_COMMAND_TIMEOUT_MS = 120000    # git pull over a weak link
RECONNECT_LOG_LINES = 500       # history reloaded when a dropped log stream returns
LOG_MAX_LINES = 5000

# Health lights for a dev launch that is not one of the missions
DEFAULT_CHECKS = {'/joint_states': 'Encoders', '/odometry/filtered': 'EKF'}

GREEN, AMBER, RED, GRAY = '#2e9d4f', '#d99a00', '#c8352b', '#8a8a8a'

ANSI = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')


# ---------------------------------------------------------------- settings

class Settings:
    """Connection settings, persisted per user with QSettings."""

    DEFAULTS = {
        'host': 'gerbil',
        'workspace': '~/domerrover/gerbil-software',   # where add_dev_user.sh clones
        'ros_distro': 'humble',
        'domain_id': '',
        'log_dir': '~/.ros/mission_logs',
        'foxglove_port': 8765,
        'rtt_warn_ms': 150,
    }

    def __init__(self, robot: str):
        self.qs = QSettings('DomerRover', f'mission_launcher_{robot}')

    def get(self, key):
        default = self.DEFAULTS[key]
        value = self.qs.value(key, default)
        return int(value) if isinstance(default, int) else str(value)

    def set(self, key, value):
        self.qs.setValue(key, value)

    def connection(self) -> remote.Connection:
        domain = self.get('domain_id').strip()
        return remote.Connection(
            host=self.get('host'),
            workspace=self.get('workspace'),
            ros_distro=self.get('ros_distro'),
            domain_id=int(domain) if domain else None,
            log_dir=self.get('log_dir'),
        )

    def presets(self) -> Dict[str, dict]:
        try:
            return json.loads(self.qs.value('dev_presets', '{}'))
        except ValueError:
            return {}

    def set_presets(self, presets: Dict[str, dict]):
        self.qs.setValue('dev_presets', json.dumps(presets))


class SettingsDialog(QDialog):
    LABELS = {
        'host': 'SSH host (alias or user@host)',
        'workspace': 'Workspace on the rover',
        'ros_distro': 'ROS distro',
        'domain_id': 'ROS_DOMAIN_ID (blank = unset)',
        'log_dir': 'Log directory on the rover',
        'foxglove_port': 'Foxglove port',
        'rtt_warn_ms': 'Warn above RTT (ms)',
    }

    def __init__(self, settings: Settings, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Rover connection')
        self.settings = settings
        self.fields = {}
        form = QFormLayout()
        for key, label in self.LABELS.items():
            if isinstance(Settings.DEFAULTS[key], int):
                field = QSpinBox()
                field.setRange(1, 65535)
                field.setValue(settings.get(key))
            else:
                field = QLineEdit(settings.get(key))
            self.fields[key] = field
            form.addRow(label, field)
        note = QLabel('Login must work without a password: '
                      'set up an SSH key (ssh-copy-id) first.')
        note.setWordWrap(True)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(note)
        layout.addWidget(buttons)

    def accept(self):
        domain = self.fields['domain_id'].text().strip()
        if domain and not domain.isdigit():
            QMessageBox.warning(self, 'Settings', 'ROS_DOMAIN_ID must be a number.')
            return
        for key, field in self.fields.items():
            value = field.value() if isinstance(field, QSpinBox) else field.text().strip()
            self.settings.set(key, value)
        super().accept()


# ---------------------------------------------------------------- widgets

class Light(QLabel):
    """A colored dot plus text."""

    def __init__(self, text: str = '', parent=None):
        super().__init__(parent)
        self.set(GRAY, text)

    def set(self, color: str, text: str):
        self.setText(f'<span style="color:{color}; font-size:16px">&#9679;</span> {text}')


class LogView(QPlainTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(LOG_MAX_LINES)
        self.setFont(QFont('Consolas' if WINDOWS else 'Monospace', 9))
        self.setLineWrapMode(QPlainTextEdit.NoWrap)

    def append_text(self, text: str):
        text = ANSI.sub('', text).replace('\r\n', '\n')
        bar = self.verticalScrollBar()
        at_bottom = bar.value() >= bar.maximum() - 4
        cursor = self.textCursor()
        cursor.movePosition(cursor.End)
        cursor.insertText(text)
        if at_bottom:
            bar.setValue(bar.maximum())


class MissionCard(QGroupBox):
    def __init__(self, mission: Mission, on_start: Callable, on_stop: Callable):
        super().__init__(mission.title)
        self.mission = mission
        self.session = remote.mission_session(mission.id)
        layout = QVBoxLayout(self)

        desc = QLabel(mission.description)
        desc.setWordWrap(True)
        layout.addWidget(desc)
        network = ('Needs the link (teleop + video)' if mission.network == 'required'
                   else 'Keeps running if the link drops')
        layout.addWidget(QLabel(f'<i>{network}</i>'))

        self.arg_widgets = {}
        for name, default in mission.args.items():
            if isinstance(default, bool):
                widget = QCheckBox(name)
                widget.setChecked(default)
                layout.addWidget(widget)
            else:
                widget = QLineEdit(str(default))
                row = QHBoxLayout()
                row.addWidget(QLabel(name))
                row.addWidget(widget)
                layout.addLayout(row)
            self.arg_widgets[name] = widget

        self.state = QLabel('')
        layout.addWidget(self.state)
        buttons = QHBoxLayout()
        self.start_btn = QPushButton('Start')
        self.stop_btn = QPushButton('Stop')
        self.start_btn.clicked.connect(lambda: on_start(self))
        self.stop_btn.clicked.connect(lambda: on_stop(self))
        buttons.addWidget(self.start_btn)
        buttons.addWidget(self.stop_btn)
        layout.addLayout(buttons)
        layout.addStretch()

    def args(self) -> Dict[str, object]:
        return {name: (w.isChecked() if isinstance(w, QCheckBox) else w.text())
                for name, w in self.arg_widgets.items()}

    def update_state(self, running: bool, can_start: bool):
        self.start_btn.setEnabled(can_start)
        self.stop_btn.setEnabled(running)
        for w in self.arg_widgets.values():
            w.setEnabled(not running)
        self.state.setText('<b style="color:%s">RUNNING</b>' % GREEN if running else '')


# ---------------------------------------------------------------- main window

class MainWindow(QMainWindow):
    def __init__(self, registry: Registry):
        super().__init__()
        self.registry = registry
        self.settings = Settings(registry.robot)
        self.setWindowTitle(f'Mission Launcher: {registry.robot}')
        # Fit the screen: at 125-150% Windows scaling a fixed 1200x850 is
        # taller than a laptop display and pushes the logs off the bottom
        screen = QApplication.primaryScreen().availableGeometry()
        self.resize(min(1200, int(screen.width() * 0.9)),
                    min(850, int(screen.height() * 0.9)))

        self.procs = set()               # keeps running QProcesses alive
        self.sessions: Dict[str, str] = {}    # session -> run id
        self.reachable: Optional[bool] = None
        self.poll_failures = 0
        self.hostname = self.settings.get('host')
        self.status_busy = self.ping_busy = False
        self.launch_tail: Optional[QProcess] = None
        self.build_tail: Optional[QProcess] = None
        self.tailing: Dict[str, str] = {}     # 'launch'/'build' -> run id
        self.dev_launches: Dict[str, str] = {}   # dev session -> launch file
        self.check_timer = QTimer(self, singleShot=True, timeout=self.run_checks)
        self.check_gen = 0              # bumped per launch; stale results are dropped
        self.check_running = False
        self.foxglove_listening = False     # from the status poll, checked on the rover

        self._build_ui()
        self.resolve_host()

        self.status_timer = QTimer(self, timeout=self.poll_status)
        self.status_timer.start(STATUS_INTERVAL_MS)
        self.ping_timer = QTimer(self, timeout=self.poll_ping)
        self.ping_timer.start(PING_INTERVAL_MS)
        QTimer.singleShot(0, self.poll_status)

    # ------------------------------------------------------------ layout

    def _build_ui(self):
        top = QHBoxLayout()
        self.host_label = QLabel()
        self.link_light = Light('link: ?')
        self.fox_light = Light('foxglove: ?')
        self.session_label = QLabel('Running: ?')
        settings_btn = QPushButton('Settings')
        settings_btn.clicked.connect(self.edit_settings)
        fox_btn = QPushButton('Open Foxglove')
        fox_btn.clicked.connect(self.open_foxglove)
        self.stop_all_btn = QPushButton('STOP ALL')
        self.stop_all_btn.setStyleSheet(
            f'QPushButton {{ background:{RED}; color:white; font-weight:bold;'
            ' padding:8px 18px; }')
        self.stop_all_btn.clicked.connect(self.stop_all)
        for w in (self.host_label, self.link_light, self.fox_light, self.session_label):
            top.addWidget(w)
            top.addSpacing(12)
        top.addStretch()
        for w in (settings_btn, fox_btn, self.stop_all_btn):
            top.addWidget(w)

        self.tabs = QTabWidget()
        # Scrollable, so on a short screen the tabs shrink instead of the logs
        self.tabs.addTab(scrollable(self._missions_tab()), 'Missions')
        self.tabs.addTab(scrollable(self._dev_tab()), 'Dev')

        # Health lights + logs, shared by both tabs
        health_box = QGroupBox('Health')
        self.health_layout = QVBoxLayout(health_box)
        self.health_lights: Dict[str, Light] = {}
        self.check_btn = QPushButton('Run checks')
        self.check_btn.clicked.connect(self.run_checks)
        self.health_layout.addWidget(self.check_btn)
        self.health_layout.addStretch()

        self.logs = QTabWidget()
        self.launch_log, self.build_log, self.output_log = LogView(), LogView(), LogView()
        self.logs.addTab(self.launch_log, 'Launch log')
        self.logs.addTab(self.build_log, 'Build log')
        self.logs.addTab(self.output_log, 'Commands')

        bottom = QSplitter(Qt.Horizontal)
        bottom.addWidget(health_box)
        bottom.addWidget(self.logs)
        bottom.setSizes([220, 980])

        split = QSplitter(Qt.Vertical)
        split.addWidget(self.tabs)
        split.addWidget(bottom)
        split.setSizes([430, 420])
        # Neither half can be dragged (or squeezed) out of sight
        split.setCollapsible(0, False)
        split.setCollapsible(1, False)
        bottom.setCollapsible(0, False)
        bottom.setCollapsible(1, False)
        bottom.setMinimumHeight(180)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.addLayout(top)
        layout.addWidget(split)
        self.setCentralWidget(central)
        self.update_host_label()

    def _missions_tab(self) -> QWidget:
        tab = QWidget()
        grid = QGridLayout(tab)
        self.cards: List[MissionCard] = []
        for i, mission in enumerate(self.registry.missions):
            card = MissionCard(mission, self.start_mission, self.stop_card)
            self.cards.append(card)
            grid.addWidget(card, 0, i)
        return tab

    def _dev_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)

        launch_box = QGroupBox('Launch any file')
        lv = QVBoxLayout(launch_box)
        row = QHBoxLayout()
        self.launch_combo = QComboBox()
        self.launch_combo.currentTextChanged.connect(self.load_launch_args)
        refresh = QPushButton('Refresh')
        refresh.clicked.connect(self.refresh_launch_files)
        row.addWidget(QLabel(f'{self.registry.package}:'))
        row.addWidget(self.launch_combo, 1)
        row.addWidget(refresh)
        lv.addLayout(row)

        preset_row = QHBoxLayout()
        self.preset_combo = QComboBox()
        self.preset_combo.activated.connect(self.apply_preset)
        save_preset = QPushButton('Save preset')
        save_preset.clicked.connect(self.save_preset)
        del_preset = QPushButton('Delete')
        del_preset.clicked.connect(self.delete_preset)
        preset_row.addWidget(QLabel('Preset:'))
        preset_row.addWidget(self.preset_combo, 1)
        preset_row.addWidget(save_preset)
        preset_row.addWidget(del_preset)
        lv.addLayout(preset_row)
        self.reload_presets()

        self.args_form_host = QWidget()
        self.args_form = QFormLayout(self.args_form_host)
        self.dev_arg_widgets: Dict[str, QWidget] = {}
        self.dev_arg_defaults: Dict[str, Optional[str]] = {}
        # The launch file whose arguments the form holds. Start stays off
        # until they load, so an empty form never falls back to the file's
        # own use_mock_hardware default (false = real motors).
        self.dev_args_for: Optional[str] = None
        # (launch file, values) from a preset, applied only to that file
        self.pending_preset: Optional[tuple] = None
        lv.addWidget(self.args_form_host)
        lv.addStretch()

        buttons = QHBoxLayout()
        self.dev_start_btn = QPushButton('Start')
        self.dev_start_btn.clicked.connect(self.start_dev)
        self.dev_stop_btn = QPushButton('Stop')
        self.dev_stop_btn.clicked.connect(self.stop_launch)
        self.dev_attach_btn = QPushButton('Attach terminal')
        self.dev_attach_btn.clicked.connect(lambda: self.attach(self.active_launch()))
        for b in (self.dev_start_btn, self.dev_stop_btn, self.dev_attach_btn):
            buttons.addWidget(b)
        lv.addLayout(buttons)

        ws_box = QGroupBox('Workspace on the rover')
        wv = QVBoxLayout(ws_box)
        git_row = QHBoxLayout()
        for label, action in (('git status', 'status'), ('git pull', 'pull')):
            b = QPushButton(label)
            b.clicked.connect(lambda _, a=action: self.run_git(a))
            git_row.addWidget(b)
        wv.addLayout(git_row)
        self.packages_edit = QLineEdit()
        self.packages_edit.setPlaceholderText('packages to build (blank = all)')
        wv.addWidget(self.packages_edit)
        build_row = QHBoxLayout()
        self.build_btn = QPushButton('colcon build')
        self.build_btn.clicked.connect(self.start_build)
        self.build_stop_btn = QPushButton('Stop build')
        self.build_stop_btn.clicked.connect(
            lambda: self.stop_sessions([remote.BUILD_SESSION]))
        build_attach = QPushButton('Attach')
        build_attach.clicked.connect(lambda: self.attach(remote.BUILD_SESSION))
        for b in (self.build_btn, self.build_stop_btn, build_attach):
            build_row.addWidget(b)
        wv.addLayout(build_row)
        wv.addStretch()

        layout.addWidget(launch_box, 3)
        layout.addWidget(ws_box, 2)
        return tab

    # ------------------------------------------------------------ process plumbing

    def run(self, argv: List[str], on_done: Optional[Callable[[int, str], None]] = None,
            echo: bool = False, timeout_ms: int = COMMAND_TIMEOUT_MS) -> QProcess:
        """Run argv; on_done(exit_code, output). exit_code -1 = failed to start.

        A command still running after timeout_ms is killed and reported as a
        failure. SSH keepalives already end a dead link in ~6 s; this is the
        backstop for anything else that hangs.
        """
        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        self.procs.add(proc)
        done = {'called': False, 'timed_out': False}
        watchdog = QTimer(proc, singleShot=True)

        def expire():
            done['timed_out'] = True
            proc.kill()

        watchdog.timeout.connect(expire)

        def finish(code: int, output: str):
            if done['called']:
                return
            done['called'] = True
            watchdog.stop()
            if done['timed_out']:
                code = code or 1
                output += f'\n[timed out after {timeout_ms // 1000} s]\n'
            self.procs.discard(proc)
            if echo and output.strip():
                self.output_log.append_text(output if output.endswith('\n') else output + '\n')
            if on_done:
                on_done(code, output)
            proc.deleteLater()

        proc.finished.connect(lambda code, _status: finish(
            code, bytes(proc.readAll()).decode('utf-8', 'replace')))
        proc.errorOccurred.connect(lambda err: err == QProcess.FailedToStart and finish(
            -1, f'could not run {argv[0]}: {proc.errorString()}\n'))
        proc.start(argv[0], argv[1:])
        watchdog.start(timeout_ms)
        return proc

    def ssh(self, script: str, on_done=None, echo: bool = True, label: str = '',
            timeout_ms: int = COMMAND_TIMEOUT_MS):
        if echo and label:
            self.output_log.append_text(f'$ {label}\n')
        return self.run(remote.ssh_argv(self.conn(), script), on_done, echo, timeout_ms)

    def conn(self) -> remote.Connection:
        return self.settings.connection()

    def stream(self, script: str, view: LogView) -> QProcess:
        # tty: killing this ssh then hangs up the remote side too, instead of
        # leaving a `tail -F` behind on the rover
        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.readyRead.connect(lambda: view.append_text(
            bytes(proc.readAll()).decode('utf-8', 'replace')))
        proc.start('ssh', remote.ssh_argv(self.conn(), script, tty=True)[1:])
        return proc

    # ------------------------------------------------------------ status

    def active_launch(self) -> Optional[str]:
        return next((s for s in self.sessions if remote.is_launch_session(s)), None)

    def poll_status(self):
        if self.status_busy:
            return
        self.status_busy = True

        def done(code, output):
            self.status_busy = False
            was = self.reachable
            if code == 0:
                self.poll_failures = 0
                self.reachable = True
                self.sessions = remote.parse_sessions(output)
                self.foxglove_listening = remote.parse_foxglove_listening(output)
            else:
                # One dropped poll on a weak field link is not "unreachable"
                self.poll_failures += 1
                if self.poll_failures < UNREACHABLE_AFTER and was is not None:
                    return
                self.reachable = False
                if was is not False:
                    self.output_log.append_text(
                        f'[rover unreachable over ssh] {output.strip()}\n')
            if self.reachable and not was and self.launch_combo.count() == 0:
                self.refresh_launch_files()
            self.update_ui()

        self.ssh(remote.list_sessions_script(self.settings.get('foxglove_port')), done,
                 echo=False)

    def update_ui(self):
        active = self.active_launch()
        building = remote.BUILD_SESSION in self.sessions
        # No launch during a build: it would load half-built files
        can_start = bool(self.reachable) and active is None and not building

        if self.reachable is None:
            self.session_label.setText('Running: ?')
        elif not self.reachable:
            self.session_label.setText('<b style="color:%s">rover unreachable</b>' % RED)
        else:
            parts = [f'Running: <b>{active or "nothing"}</b>']
            if building:
                parts.append('<b>building</b>')
            self.session_label.setText(' | '.join(parts))

        for card in self.cards:
            card.update_state(card.session in self.sessions, can_start)
        launch = self.launch_combo.currentText()
        self.dev_start_btn.setEnabled(
            can_start and bool(launch) and self.dev_args_for == launch)
        self.dev_stop_btn.setEnabled(bool(active))
        self.dev_attach_btn.setEnabled(bool(active))
        self.build_btn.setEnabled(bool(self.reachable) and not building)
        self.build_stop_btn.setEnabled(building)
        self.stop_all_btn.setEnabled(self.reachable is not False)

        port = self.settings.get('foxglove_port')
        if not self.reachable:
            self.fox_light.set(GRAY, 'foxglove: ?')
        elif self.foxglove_listening:
            self.fox_light.set(GREEN, f'foxglove: up :{port}')
        elif active:
            self.fox_light.set(RED, f'foxglove: down :{port}')
        else:
            self.fox_light.set(GRAY, 'foxglove: idle')

        # Compared by run id, so a restart of the same session is a new run
        if active and self.tailing.get('launch') != self.sessions[active]:
            self.on_launch_detected(active)
        elif active and self.reachable and not tail_alive(self.launch_tail):
            # Same run, but the stream died with the link: reload recent history
            self.launch_tail = self.reconnect_tail(self.launch_tail, active, self.launch_log)
        build_run = self.sessions.get(remote.BUILD_SESSION)
        if build_run and self.tailing.get('build') != build_run:
            self.tailing['build'] = build_run
            self.build_log.clear()
            self.logs.setCurrentWidget(self.build_log)
            self.build_tail = self.restart_tail(self.build_tail, remote.BUILD_SESSION,
                                                self.build_log)
        elif build_run and self.reachable and not tail_alive(self.build_tail):
            self.build_tail = self.reconnect_tail(self.build_tail, remote.BUILD_SESSION,
                                                  self.build_log)

    def on_launch_detected(self, session: str):
        """A launch started, by us or anyone: follow its log and checks."""
        self.tailing['launch'] = self.sessions[session]
        self.launch_log.clear()
        self.launch_tail = self.restart_tail(self.launch_tail, session, self.launch_log)
        self.logs.setCurrentWidget(self.launch_log)
        # A check still in flight belongs to the previous launch: drop it
        self.check_gen += 1
        self.check_running = False
        self.check_btn.setEnabled(True)
        self.set_health_topics(self.checks_for(session))
        self.check_timer.start(CHECK_DELAY_MS)

    def restart_tail(self, old: Optional[QProcess], session: str, view: LogView,
                     lines: int = 200):
        # The previous tail keeps running until a new session starts, so the
        # shutdown lines of a stopped launch still reach the log pane.
        if old is not None:
            old.kill()
            old.deleteLater()
        return self.stream(remote.tail_log_script(self.conn(), session, lines), view)

    def reconnect_tail(self, old: Optional[QProcess], session: str, view: LogView):
        view.clear()
        view.append_text(f'[mission_launcher] log stream reconnected; '
                         f'last {RECONNECT_LOG_LINES} lines:\n')
        return self.restart_tail(old, session, view, RECONNECT_LOG_LINES)

    # ------------------------------------------------------------ network

    def resolve_host(self):
        conn = self.conn()

        def done(code, output):
            self.hostname = remote.parse_resolved_host(output if code == 0 else '', conn.host)
            self.update_host_label()

        self.run(remote.resolve_host_argv(conn), done)

    def update_host_label(self):
        host = self.settings.get('host')
        extra = f' ({self.hostname})' if self.hostname and self.hostname != host else ''
        self.host_label.setText(f'Rover: <b>{host}</b>{extra}')

    def poll_ping(self):
        if self.ping_busy:
            return
        self.ping_busy = True
        active = self.active_launch()
        mission = self.mission_for_session(active) if active else None
        required = mission is None or mission.network == 'required'
        warn = self.settings.get('rtt_warn_ms')

        def done(_code, output):
            self.ping_busy = False
            rtt = remote.parse_ping_ms(output)
            if rtt is None:
                note = '' if required else ' (mission keeps running)'
                self.link_light.set(RED, f'link: lost{note}')
            elif required and rtt > warn:
                self.link_light.set(AMBER, f'link: {rtt:.0f} ms (slow)')
            else:
                self.link_light.set(GREEN, f'link: {rtt:.0f} ms')

        self.run(remote.ping_argv(self.hostname, WINDOWS), done)

    def open_foxglove(self):
        url = remote.foxglove_url(self.hostname, self.settings.get('foxglove_port'))
        QDesktopServices.openUrl(QUrl(url))

    # ------------------------------------------------------------ health checks

    def mission_for_session(self, session: str) -> Optional[Mission]:
        for m in self.registry.missions:
            if remote.mission_session(m.id) == session:
                return m
        if session.startswith(remote.DEV_PREFIX):
            # The file that is running, not whatever the dropdown shows now
            launch = self.dev_launches.get(session) or next(
                (f for f in self.launch_files() if remote.dev_session(f) == session), None)
            if launch:
                return self.registry.mission_for_launch(launch)
        return None

    def launch_files(self) -> List[str]:
        return [self.launch_combo.itemText(i) for i in range(self.launch_combo.count())]

    def checks_for(self, session: str) -> Dict[str, str]:
        mission = self.mission_for_session(session)
        return mission.checks if mission and mission.checks else DEFAULT_CHECKS

    def set_health_topics(self, checks: Dict[str, str]):
        for light in self.health_lights.values():
            self.health_layout.removeWidget(light)
            light.deleteLater()
        self.health_lights = {}
        for i, (topic, label) in enumerate(checks.items()):
            light = Light(label)
            light.setToolTip(topic)
            self.health_layout.insertWidget(i, light)
            self.health_lights[topic] = light

    def run_checks(self):
        active = self.active_launch()
        if not active:
            self.output_log.append_text('nothing running to check\n')
            return
        if self.check_running:
            return          # the timer and the button can both ask; one at a time
        checks = self.checks_for(active)
        if list(self.health_lights) != list(checks):
            self.set_health_topics(checks)
        for topic, light in self.health_lights.items():
            light.set(AMBER, f'{checks[topic]} (checking)')
        self.check_running = True
        self.check_btn.setEnabled(False)
        gen = self.check_gen

        def done(_code, output):
            if gen != self.check_gen:
                return      # a new launch replaced these lights meanwhile
            self.check_running = False
            self.check_btn.setEnabled(True)
            results = remote.parse_checks(output)
            for topic, light in self.health_lights.items():
                ok = results.get(topic)
                light.set(GREEN if ok else RED,
                          checks.get(topic, topic) + ('' if ok else ' (no data)'))

        self.ssh(remote.checks_script(self.conn(), list(checks)), done, echo=False)

    # ------------------------------------------------------------ missions

    def start_mission(self, card: MissionCard):
        m = card.mission
        script = remote.start_launch_script(
            self.conn(), card.session, self.registry.package, m.launch, card.args())
        self.start_launch(script, f'start {m.title}')

    def stop_card(self, card: MissionCard):
        self.stop_sessions([card.session])

    def start_launch(self, script: str, label: str):
        def done(code, output):
            if code != 0:
                QMessageBox.warning(self, 'Start failed', output.strip() or f'exit {code}')
            self.poll_status()

        self.ssh(script, done, label=label)

    def stop_launch(self):
        active = self.active_launch()
        if active:
            self.stop_sessions([active])

    def stop_sessions(self, sessions: Optional[List[str]]):
        label = 'stop all' if sessions is None else 'stop ' + ' '.join(sessions)
        self.ssh(remote.stop_sessions_script(sessions), lambda *_: self.poll_status(),
                 label=label)

    def stop_all(self):
        self.stop_sessions(None)

    # ------------------------------------------------------------ dev tab

    def refresh_launch_files(self):
        def done(code, output):
            if code != 0:
                return
            current = self.launch_combo.currentText()
            files = remote.parse_launch_files(output)
            self.launch_combo.blockSignals(True)
            self.launch_combo.clear()
            self.launch_combo.addItems(files)
            if current in files:
                self.launch_combo.setCurrentText(current)
            self.launch_combo.blockSignals(False)
            self.load_launch_args(self.launch_combo.currentText())

        self.ssh(remote.list_launch_files_script(self.conn(), self.registry.package),
                 done, echo=False)

    def load_launch_args(self, launch_file: str):
        while self.args_form.rowCount():
            self.args_form.removeRow(0)
        self.dev_arg_widgets, self.dev_arg_defaults = {}, {}
        self.dev_args_for = None
        if self.pending_preset and self.pending_preset[0] != launch_file:
            self.pending_preset = None
        self.update_ui()
        if not launch_file:
            return
        self.args_form.addRow(QLabel('loading arguments...'))

        def done(code, output):
            if self.launch_combo.currentText() != launch_file:
                return              # selection changed while loading
            while self.args_form.rowCount():
                self.args_form.removeRow(0)
            pending = {}
            if self.pending_preset and self.pending_preset[0] == launch_file:
                pending = self.pending_preset[1]
            self.pending_preset = None
            if code != 0:
                # Start stays disabled: no form means no use_mock_hardware
                self.args_form.addRow(QLabel('could not read arguments (Refresh to retry)'))
                self.output_log.append_text(output)
                return
            for arg in remote.parse_show_args(output):
                default = arg.default
                # Dev launches default to mock so nothing drives by accident
                if arg.name == 'use_mock_hardware':
                    default_shown = 'true'
                else:
                    default_shown = default
                value = pending.get(arg.name, default_shown)
                if default in ('true', 'false'):
                    widget = QCheckBox()
                    widget.setChecked(value == 'true')
                else:
                    widget = QLineEdit(value or '')
                    if default is None:
                        widget.setPlaceholderText('required')
                widget.setToolTip(arg.description)
                label = QLabel(arg.name)
                label.setToolTip(arg.description)
                self.args_form.addRow(label, widget)
                self.dev_arg_widgets[arg.name] = widget
                self.dev_arg_defaults[arg.name] = default
            if not self.dev_arg_widgets:
                self.args_form.addRow(QLabel('no arguments'))
            self.dev_args_for = launch_file
            self.update_ui()

        self.ssh(remote.show_args_script(self.conn(), self.registry.package, launch_file),
                 done, echo=False)

    def dev_values(self) -> Dict[str, str]:
        values = {}
        for name, w in self.dev_arg_widgets.items():
            values[name] = (('true' if w.isChecked() else 'false')
                            if isinstance(w, QCheckBox) else w.text().strip())
        return values

    def start_dev(self):
        launch = self.launch_combo.currentText()
        if self.dev_args_for != launch:
            return          # arguments not loaded; the button is disabled too
        values = self.dev_values()
        missing = [n for n, v in values.items()
                   if not v and self.dev_arg_defaults.get(n) is None]
        if missing:
            QMessageBox.warning(self, 'Missing arguments', 'Required: ' + ', '.join(missing))
            return
        # Only pass what differs from the launch file's own defaults
        args = {n: v for n, v in values.items()
                if v and v != self.dev_arg_defaults.get(n)}
        session = remote.dev_session(launch)
        self.dev_launches[session] = launch
        script = remote.start_launch_script(
            self.conn(), session, self.registry.package, launch, args)
        self.start_launch(script, f'start {launch} ' + remote.render_launch_args(args))

    def reload_presets(self):
        self.preset_combo.clear()
        self.preset_combo.addItem('')
        self.preset_combo.addItems(sorted(self.settings.presets()))

    def save_preset(self):
        launch = self.launch_combo.currentText()
        if not launch:
            return
        name, ok = QInputDialog.getText(self, 'Save preset', 'Preset name:')
        if not ok or not name.strip():
            return
        presets = self.settings.presets()
        presets[name.strip()] = {'launch': launch, 'args': self.dev_values()}
        self.settings.set_presets(presets)
        self.reload_presets()
        self.preset_combo.setCurrentText(name.strip())

    def delete_preset(self):
        name = self.preset_combo.currentText()
        presets = self.settings.presets()
        if name in presets:
            del presets[name]
            self.settings.set_presets(presets)
            self.reload_presets()

    def apply_preset(self, _index: int):
        preset = self.settings.presets().get(self.preset_combo.currentText())
        if not preset:
            return
        if preset['launch'] not in self.launch_files():
            QMessageBox.warning(
                self, 'Preset', f'{preset["launch"]} is not in {self.registry.package} '
                'on the rover any more. Delete the preset or Refresh the file list.')
            return
        self.pending_preset = (preset['launch'], preset['args'])
        if self.launch_combo.currentText() == preset['launch']:
            self.load_launch_args(preset['launch'])
        else:
            self.launch_combo.setCurrentText(preset['launch'])

    def run_git(self, action: str):
        self.logs.setCurrentWidget(self.output_log)
        self.ssh(remote.git_script(self.conn(), action), label=f'git {action}',
                 timeout_ms=SLOW_COMMAND_TIMEOUT_MS)

    def start_build(self):
        if self.active_launch():
            answer = QMessageBox.question(
                self, 'Build while running?',
                f'{self.active_launch()} is running. Rebuilding C++ packages under '
                'a running launch can crash it. Build anyway?')
            if answer != QMessageBox.Yes:
                return
        packages = self.packages_edit.text().split()
        command = remote.build_command(self.conn(), packages)
        script = remote.start_session_script(
            self.conn(), remote.BUILD_SESSION, command, exclusive=False)
        # The build log is cleared and followed once the new run shows up
        self.ssh(script, lambda *_: self.poll_status(), label='colcon build')

    # ------------------------------------------------------------ misc

    def attach(self, session: Optional[str]):
        if not session:
            return
        argv = remote.attach_argv(self.conn(), session)
        self.output_log.append_text(
            f'$ {" ".join(argv)}\n(detach with Ctrl-b d; Ctrl-c in there stops it)\n')
        if not open_terminal(argv):
            QMessageBox.information(
                self, 'Attach', 'No terminal emulator found. Run:\n\n' + ' '.join(argv))

    def edit_settings(self):
        if SettingsDialog(self.settings, self).exec_():
            # Nothing from the previous rover or workspace may carry over: its
            # file list and argument form would otherwise still count as loaded
            self.reachable = None
            self.poll_failures = 0
            self.sessions = {}
            self.tailing.clear()
            self.dev_launches.clear()
            for proc in (self.launch_tail, self.build_tail):
                if proc is not None:
                    proc.kill()
            self.launch_combo.blockSignals(True)
            self.launch_combo.clear()
            self.launch_combo.blockSignals(False)
            self.load_launch_args('')       # clears the form and dev_args_for
            # Ping the new host right away; ssh -G refines it when it returns
            self.hostname = remote.parse_resolved_host('', self.settings.get('host'))
            self.resolve_host()
            self.update_host_label()
            self.poll_status()              # refetches the launch files once reachable

    def closeEvent(self, event):
        # Closing the GUI never stops anything on the rover
        for proc in [self.launch_tail, self.build_tail, *self.procs]:
            if proc is not None:
                proc.kill()
        super().closeEvent(event)


def scrollable(widget: QWidget) -> QScrollArea:
    area = QScrollArea()
    area.setWidgetResizable(True)
    area.setFrameShape(QScrollArea.NoFrame)
    area.setWidget(widget)
    return area


def tail_alive(proc: Optional[QProcess]) -> bool:
    return proc is not None and proc.state() != QProcess.NotRunning


def open_terminal(argv: List[str]) -> bool:
    """Run argv in a new local terminal window."""
    if WINDOWS:
        subprocess.Popen(argv, creationflags=subprocess.CREATE_NEW_CONSOLE)
        return True
    if sys.platform == 'darwin':
        cmd = ' '.join(f"'{a}'" for a in argv)
        subprocess.Popen(['osascript', '-e', f'tell app "Terminal" to do script "{cmd}"'])
        return True
    for term, flag in (('x-terminal-emulator', '-e'), ('gnome-terminal', '--'),
                       ('konsole', '-e'), ('xterm', '-e')):
        if shutil.which(term):
            subprocess.Popen([term, flag, *argv])
            return True
    return False


def run_app(registry: Registry) -> int:
    os.environ.setdefault('QT_AUTO_SCREEN_SCALE_FACTOR', '1')
    app = QApplication(sys.argv)
    window = MainWindow(registry)
    window.show()
    return app.exec_()
