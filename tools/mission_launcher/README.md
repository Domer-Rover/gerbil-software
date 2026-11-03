# Mission launcher

A desktop GUI for the base-station laptop that starts a URC mission on the
rover with one click. It does this over SSH, in a named tmux session, so a
WiFi drop or a closed GUI never kills the launch. It also streams the launch
log, shows health lights for each sensor, and stops the mission cleanly.

The laptop doesn't need ROS. It needs Python 3.8+, PyQt5, PyYAML, and an `ssh`
client. Windows 10/11 already ships OpenSSH.

## Setup

1. **Key-based SSH to the Jetson.** The GUI never prompts for a password.

   ```bash
   ssh-keygen -t ed25519          # if you have no key yet
   ssh-copy-id <user>@<jetson>    # Windows: append ~/.ssh/id_ed25519.pub to the
                                  # Jetson's ~/.ssh/authorized_keys by hand
   ```

   Optionally, add an alias to `~/.ssh/config`:

   ```
   Host gerbil
       HostName 192.168.1.40
       User <user>
   ```

2. **tmux on the Jetson**: `sudo apt install tmux`. The workspace must be
   built there, as usual.

3. **Install the GUI on the laptop**, from this directory:

   ```bash
   pip install -e .               # or: apt install python3-pyqt5 python3-yaml
   python -m mission_launcher
   ```

   On first start, open **Settings** and set the SSH host (`gerbil`, or
   `user@ip`) and the workspace path on the rover.

## Missions tab

The **Missions** tab has one card per mission from
[`src/gerbil_bringup/config/missions.yaml`](../../src/gerbil_bringup/config/missions.yaml).
Each card's toggles are the only arguments a mission exposes. Everything else
is fixed in its `mission_<id>.launch.py`.

- **Start** runs the mission in tmux session `mission_<id>`. Only one launch
  can run at a time, because they all hold the RoboClaw serial port. Nothing
  can launch while a `colcon build` runs either. The rover enforces both
  rules under a lock, so two laptops pressing Start together cannot both win.
- **Stop** sends Ctrl-C, waits up to 15 s for a clean shutdown, then kills the
  session. Ctrl-C comes first so that ros2_control stops the motors.
- **STOP ALL** does the same for every launcher session.
- **Health** checks each mission's topics about 20 s after start, and again
  whenever you press **Run checks**.
- The top bar shows the link round-trip time and whether the Foxglove bridge
  is up. A slow or lost link turns amber or red for teleop missions.
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
