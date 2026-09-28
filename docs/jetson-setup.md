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

```bash
ssh-keygen -t ed25519                              # skip if you already have a key
ssh-copy-id <username>@<gerbil-jetson>       # once; last time you type the password
ssh <username>@<gerbil-jetson>
ssh -T git@github.com                              # confirms GitHub key works
```

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

