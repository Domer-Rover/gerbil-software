# Mission launcher

A desktop GUI for the base-station laptop that starts a URC mission on the
rover with one click. It does this over SSH, in a named tmux session, so a
WiFi drop or a closed GUI never kills the launch. It also streams the launch
log, shows health lights for each sensor, and stops the mission cleanly.

The laptop doesn't need ROS. It needs Python 3.8+, PyQt5, PyYAML, and an `ssh`
client. Windows 10/11 already ships OpenSSH.

## Setup

1. **Key-based SSH to the Jetson.** The GUI never prompts for a password, so
   `ssh <user>@<jetson>` must log in without one. Follow
   [First login in docs/jetson-setup.md](../../docs/jetson-setup.md#first-login-developer-from-your-laptop),
   which has the Windows command (Windows has no `ssh-copy-id`) and the
   optional `Host gerbil` alias.

2. **tmux on the Jetson**: `sudo apt install tmux`. The workspace must be
   built there, as usual.

3. **Install the GUI on the laptop**, from this directory:

   ```bash
   python -m venv .venv           # a private Python for the app (ignored by git)
   .venv/bin/pip install -e .     # Windows: .venv\Scripts\pip install -e .
   .venv/bin/python -m mission_launcher    # Windows: .venv\Scripts\python ...
   ```

   On Ubuntu, `apt install python3-pyqt5 python3-yaml` works instead of the venv.

   On first start, open **Settings** and set the SSH host (`gerbil`, or
   `user@ip`). The workspace defaults to `~/domerrover/gerbil-software`, where
   `scripts/add_dev_user.sh` clones it; change it if yours is elsewhere.

## How it fits together

```
LAPTOP                                    ROVER (Jetson)
Mission Launcher ──ssh: start/stop────▶  tmux session mission_<id>
                 ──ssh: tmux ls (3 s)──▶   └ ros2 launch: ros2_control, ZED, EKF,
                 ◀─ssh: tail -F log────         twist_mux, foxglove_bridge :8765
Foxglove         ◀─websocket :8765───────────────┘
Terminal         ──ssh──▶ teleop_keyboard.sh / teleop_joystick.sh ──▶ twist_mux
```

- The GUI talks to the rover only over SSH, one short command at a time. The
  launch lives in tmux on the rover, not in any SSH connection, so closing the
  GUI or losing WiFi never stops it.
- The ROS nodes talk to each other on the rover. The laptop is not on the ROS
  network, which is why it needs no ROS install.
- **Teleop is not part of any mission.** Run it in a terminal SSHed into the
  rover; it reaches the controller through twist_mux (joystick > keyboard >
  Nav2). Keyboard teleop works from the laptop over SSH. The joystick script
  needs the gamepad plugged into the **rover**.

## Running a session

1. **Check the robot is free.** Teammates may be running something, possibly
   from another account, which the GUI cannot see:

   ```bash
   ssh gerbil 'ps -eo user,etime,cmd | grep -E "[r]os2 launch|[c]omponent_container|[r]os2_control_node"'
   ```

   No output means free. A leftover `component_container` from a killed launch
   holds the ZED; its owner can end it with `pkill -INT -f zed_container`.
   Only one launch may drive the RoboClaw or open the ZED at a time.
2. **Start the mission** in the GUI. Tick `use_mock_hardware` for a dry run.
   Health lights go green 20 to 35 s after Start.
3. **Drive** (optional), in a second terminal:

   ```bash
   ssh gerbil
   ros2 run gerbil_bringup teleop_keyboard.sh     # or teleop_joystick.sh
   SPEED=0.15 ros2 run gerbil_bringup teleop_keyboard.sh   # slower, indoors
   ```

   Keep the keyboard window focused. Letting go stops the wheels within about
   a second (twist_mux and the controller each time out after 0.5 s). In
   Autonomy, letting go hands control back to Nav2 instead.
4. **Before the real motors** (mock unticked): the RoboClaw serial timeout
   must be set, since it is what stops the wheels after Stop (see below):

   ```bash
   ssh gerbil 'cd ~/domerrover/gerbil-software && python3 scripts/serial_timeout.py'   # expect 0.2s, not DISABLED
   ```

   The script needs `pip3 install --user basicmicro` once per account. Test
   with the wheels off the ground first.
5. **Stop** in the GUI, then Ctrl-C the teleop terminal.
6. **Results** stay on the rover in `~/.ros/mission_logs/` (see [Logs](#logs)).

## Missions tab

The **Missions** tab has one card per mission from
[`src/gerbil_bringup/config/missions.yaml`](../../src/gerbil_bringup/config/missions.yaml).
Each card's toggles are the only arguments a mission exposes. Everything else
is fixed in its `mission_<id>.launch.py`.

- **Start** runs the mission in tmux session `mission_<id>`. Only one launch
  can run at a time, because they all hold the RoboClaw serial port. Nothing
  can launch while a `colcon build` runs either. The rover enforces both
  rules under a lock, so two laptops pressing Start together cannot both win.
- **use_mock_hardware** is unticked (real motors) by default and resets every
  time the GUI restarts, so starting with it off asks for confirmation first.
- **Stop** sends Ctrl-C, waits up to 15 s for a clean shutdown, then kills the
  session. Once the driver goes quiet, the RoboClaw's serial timeout is what
  stops the wheels: `gerbil_hw` sends no zero command on shutdown. Keep that
  timeout set (`python3 scripts/serial_timeout.py`, 0.2 s); at 0 the motors
  would keep their last command. Tested on Gerbil: the wheels stop within ~1 s.
- **STOP ALL** does the same for every launcher session.
- **Health** checks each mission's topics about 20 s after start, and again
  whenever you press **Run checks**.
- The top bar shows the link round-trip time and whether the Foxglove bridge's
  port is listening on the rover (checked there, so it adds nothing to the
  bridge's log). A slow or lost link turns amber or red for teleop missions.
  Autonomy only reports the link, since it has to keep running without it.

Closing the GUI leaves the mission running. Reopening it picks the mission
back up, because all state lives in tmux on the rover.

## Dev tab

The Dev tab is for working on the robot.

- **Launch any file**: pick any launch file in `gerbil_bringup`. Its arguments
  are read with `ros2 launch --show-args` and shown as a form.
  `use_mock_hardware` defaults to **on** here. Start stays disabled until the
  form has loaded, so a slow or failed read can never launch with the file's
  own default (mock off). Only values that differ from
  the file's defaults are passed. Frequent setups can be saved as presets.
  The launch runs in session `dev_<name>`, under the same one-at-a-time lock.
- **Workspace**: `git status`, `git pull --ff-only`, and `colcon build`
  (optionally `--packages-select`). The build runs in session `build`, with
  its own log tab.
- **Attach** opens a local terminal attached to the tmux session. Detach with
  `Ctrl-b d`; pressing Ctrl-C in there stops the launch.

## SSH traffic

On Linux and macOS, every command reuses one SSH connection
(`ControlMaster`, kept open for 2 minutes), so the 3-second status poll costs
almost nothing on a weak link. Windows' OpenSSH can't share connections, so
there each command connects afresh. On both, a single failed poll is ignored;
the rover shows as unreachable only after two in a row.

## Logs

Each run logs to `~/.ros/mission_logs/<session>-<timestamp>.log` on the
rover. `<session>.log` always points at the latest run.

## Adding a mission or porting to Capybara

The GUI is robot-agnostic. To add a mission:

1. Write `src/<robot>_bringup/launch/mission_<id>.launch.py`.
2. Add an entry to `src/<robot>_bringup/config/missions.yaml`.

`tests/test_config.py` checks that every registry entry points at a real
launch file, and that each toggle is a real argument of that file.

## Tests

```bash
pip install -e .[test]
pytest
```

The tests cover the SSH/tmux command builders and output parsers in
`remote.py`, plus the registry. They need neither Qt, SSH, nor a rover.
