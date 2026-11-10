# Jetson Setup

Gerbil runs on its own Jetson, separate from Capybara's. Only one person can use the robot hardware at a time.

## Add a developer (admin)

```bash
cd ~/gerbil-software
sudo scripts/add_dev_user.sh <username> "<Full Name>" <git-email>   # username must be lowercase
sudo scripts/add_dev_user.sh --regen-key <username>                 # replace their GitHub key
sudo pkill -u <username>; sudo userdel -r <username>                # remove a developer
```

The script creates the user, adds hardware groups (`dialout video render plugdev i2c gpio zed jtop adm`), sets git author, clones the three repos into `~/domerrover/`, and prints a GitHub SSH key. Add that key at https://github.com/settings/keys.

Developers have no sudo. System packages: ask the admin (`sudo apt install`). Personal Python packages: `python3 -m venv .venv`.

## First login (developer, from your laptop)

Key login means the Jetson recognizes your laptop's key instead of asking for a
password. It is required for the [mission launcher GUI](../tools/mission_launcher/README.md),
which cannot type passwords. The password you enter below is your **Jetson**
account's, once.

Linux / macOS:

```bash
ssh-keygen -t ed25519                              # skip if you already have a key
ssh-copy-id <username>@<gerbil-jetson>       # once; last time you type the password
ssh <username>@<gerbil-jetson>
ssh -T git@github.com                              # confirms GitHub key works
```

Windows (PowerShell; Windows has no `ssh-copy-id`):

```powershell
ssh-keygen -t ed25519                              # skip if ~\.ssh\id_ed25519 exists; Enter for no passphrase
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh <username>@<gerbil-jetson> "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
ssh <username>@<gerbil-jetson>                     # should log in with no password prompt
```

Leave the key's passphrase empty: the GUI runs ssh in the background, where a
passphrase prompt cannot appear.

Optional short name, in `~/.ssh/config` (Windows: `C:\Users\<you>\.ssh\config`):

```
Host gerbil
    HostName <gerbil-jetson>
    User <username>
```

Then `ssh gerbil` works, and `gerbil` is the host to enter in the GUI. **Only
the first `Host gerbil` block counts**: a second one (say, for the shared
`domerrover` account) is silently ignored. Give each account its own name,
e.g. `Host gerbil-team`.

Still asked for a password? You are logging in as a different user than the one
whose `authorized_keys` you edited, or `~/.ssh` on the Jetson is writable by
others (rerun the copy command; it fixes the permissions).

## Serial devices

Gerbil's Jetson has different USB adapters than Capybara's, so fill in the real
IDs before installing the rules:

```bash
ls -l /dev/serial/by-id/
udevadm info -q property -n /dev/ttyUSB0 | grep ID_SERIAL=
```

Put them into `udev/99-gerbil.rules`, then:

```bash
sudo cp udev/99-gerbil.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/gerbil_*
```

| Name | Device |
|---|---|
| `/dev/gerbil_roboclaw` | RoboClaw (address 128), both wheels + encoders |
| `/dev/gerbil_lidar` | LD19 LIDAR |
| `/dev/gerbil_gps` | u-blox GPS |

