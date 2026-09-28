#!/usr/bin/env bash
# add_dev_user.sh — onboard a developer onto the Jetson.
#
# Creates a Linux user with their own home, adds the hardware groups, clones the
# team repos, sets their git author, sources ROS in their shell, and generates
# an SSH key for GitHub (Jetson -> GitHub pushes).
#
# The GitHub key is NOT a login key. To log in without a password, each
# developer runs `ssh-copy-id <username>@<jetson>` once from their own laptop.
#
# Usage (run on the Jetson as a sudoer):
#   sudo scripts/add_dev_user.sh <username> "<Full Name>" <git-email>
#   sudo scripts/add_dev_user.sh --regen-key <username>
#
# Safe to re-run on a user that already exists; finished steps are skipped.
set -euo pipefail

GITHUB_ORG="Domer-Rover"
REPOS=(capybara-software gerbil-software phoenix-software)
WORK_DIR="domerrover"   # repos land in ~/domerrover/<repo>
ROS_SETUP="/opt/ros/humble/setup.bash"
# Hardware + log access only. No sudo/docker: both are root-equivalent.
GROUPS_TO_ADD=(dialout video render plugdev i2c gpio zed jtop adm)

usage() {
    echo "Usage: sudo $0 <username> \"<Full Name>\" <git-email>" >&2
    echo "       sudo $0 --regen-key <username>" >&2
    exit 1
}

if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo." >&2
    exit 1
fi

REGEN_KEY=false
if [[ ${1:-} == "--regen-key" ]]; then
    [[ $# -eq 2 ]] || usage
    REGEN_KEY=true
    USERNAME="$2"
else
    [[ $# -eq 3 ]] || usage
    USERNAME="$1"
    FULLNAME="$2"
    EMAIL="$3"
fi

if [[ ! $USERNAME =~ ^[a-z][a-z0-9_-]*$ ]]; then
    echo "Bad username '$USERNAME': Linux usernames must be all lowercase (letters, digits, - or _)," >&2
    echo "starting with a letter, e.g. 'henry'. This is separate from your GitHub handle." >&2
    exit 1
fi

HOME_DIR="/home/$USERNAME"
SSH_DIR="$HOME_DIR/.ssh"
KEY="$SSH_DIR/id_ed25519"

# Run a command as the user from inside their home. Without the cd, the command
# inherits the admin's working directory, which the user can't read ("could not stat").
as_user() {
    (cd "$HOME_DIR" && sudo -u "$USERNAME" -H "$@")
}

make_github_key() {
    local comment
    comment="$(as_user git config --global user.email 2>/dev/null || echo "$USERNAME@$(hostname)")"
    as_user mkdir -p "$SSH_DIR"
    chmod 700 "$SSH_DIR"
    as_user ssh-keygen -t ed25519 -C "$comment" -N "" -f "$KEY" -q
    if ! as_user ssh-keygen -F github.com &>/dev/null; then
        as_user bash -c "ssh-keyscan -t ed25519 github.com 2>/dev/null >> ~/.ssh/known_hosts"
    fi
}

print_github_key() {
    echo
    echo "================================================================"
    echo "GitHub SSH key for $USERNAME. Add it at https://github.com/settings/keys"
    echo "(New SSH key -> paste the whole line below):"
    echo
    cat "$KEY.pub"
    echo "================================================================"
    echo
}

# --regen-key: replace a lost or never-added GitHub key. Login keys
# (authorized_keys) are not touched.
if $REGEN_KEY; then
    id "$USERNAME" &>/dev/null || { echo "User $USERNAME does not exist." >&2; exit 1; }
    rm -f "$KEY" "$KEY.pub"
    make_github_key
    print_github_key
    echo "If the old key was added to GitHub, delete it there."
    exit 0
fi

# 1. Linux account. useradd, not adduser: the Jetson's /etc/adduser.conf adds
#    desktop EXTRA_GROUPS (e.g. lightdm) that don't exist on this image, which
#    makes adduser abort halfway through.
if id "$USERNAME" &>/dev/null; then
    echo "User $USERNAME already exists, skipping account creation."
else
    useradd --create-home --shell /bin/bash --comment "$FULLNAME" "$USERNAME"
    echo "Set a login password for $USERNAME:"
    until passwd "$USERNAME"; do
        echo "Passwords didn't match, try again."
    done
fi

# 2. Groups (skip any that don't exist on this board instead of failing)
for g in "${GROUPS_TO_ADD[@]}"; do
    if getent group "$g" &>/dev/null; then
        usermod -aG "$g" "$USERNAME"
    else
        echo "Warning: group '$g' does not exist on this system, skipping."
    fi
done

# 3. Git author and ROS environment
as_user git config --global user.name "$FULLNAME"
as_user git config --global user.email "$EMAIL"
if ! grep -qF "$ROS_SETUP" "$HOME_DIR/.bashrc" 2>/dev/null; then
    echo "source $ROS_SETUP" | as_user tee -a "$HOME_DIR/.bashrc" >/dev/null
fi

# 4. GitHub key, printed before cloning so a clone failure can't hide it
[[ -f "$KEY" ]] || make_github_key
print_github_key

# 5. Repo clones over HTTPS (works before the key is on GitHub, for public
#    repos); push URL set to SSH so pushes work once the key is added.
as_user mkdir -p "$HOME_DIR/$WORK_DIR"
for repo in "${REPOS[@]}"; do
    dest="$HOME_DIR/$WORK_DIR/$repo"
    if [[ -d "$dest/.git" ]]; then
        echo "$repo already cloned."
        continue
    fi
    if as_user git clone "https://github.com/$GITHUB_ORG/$repo.git" "$dest"; then
        as_user git -C "$dest" remote set-url --push origin "git@github.com:$GITHUB_ORG/$repo.git"
    else
        echo "Warning: could not clone $repo (private repo?). After adding the GitHub key, run:" >&2
        echo "  git clone git@github.com:$GITHUB_ORG/$repo.git ~/$WORK_DIR/$repo" >&2
    fi
done

echo
echo "User $USERNAME is set up."
echo "Groups: $(id -nG "$USERNAME")"
echo "Repos:  ~/$WORK_DIR/"
echo "Login:  from your laptop, once:  ssh-copy-id $USERNAME@$(hostname -f)"
echo "        then:                    ssh $USERNAME@$(hostname -f)"
echo "Group changes apply on the next login."
