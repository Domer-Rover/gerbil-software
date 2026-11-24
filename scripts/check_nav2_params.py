#!/usr/bin/env python3
"""Find parameters in nav2_params.yaml that the running nodes never declared.

WHY THIS EXISTS: a Nav2 node declares the parameters it understands and
IGNORES everything else in the yaml, silently. A typo, or a parameter copied
from a newer Nav2 release, does not warn -- it just has no effect. We shipped
"movement_timeout: 15.0" for weeks; the real name is
"movement_time_allowance", so the 10.0 default was in force the whole time.

Run it with the stack UP, since it asks the live nodes what they declared:

    ros2 launch gerbil_bringup gerbil_nav2.launch.py
    python3 scripts/check_nav2_params.py

Anything it reports as DEAD is doing nothing. Delete it, or fix the name.
"""
import subprocess
import sys

import yaml

CONFIG = 'src/gerbil_bringup/config/nav2_params.yaml'


def leaves(node, prefix=''):
    """Flatten a nested dict into dotted parameter names, as ROS 2 sees them."""
    out = []
    for key, value in node.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            out.extend(leaves(value, f'{name}.'))
        else:
            out.append(name)
    return out


def declared(node_name):
    """Ask the live node which parameters it actually declared."""
    try:
        r = subprocess.run(['ros2', 'param', 'list', f'/{node_name}'],
                           capture_output=True, text=True, timeout=20)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    if r.returncode != 0:
        return None
    return {line.strip() for line in r.stdout.splitlines()
            if line.strip() and not line.strip().endswith(':')}


def main():
    with open(CONFIG) as f:
        cfg = yaml.safe_load(f)

    total_dead = 0
    skipped = []
    for node_name, body in cfg.items():
        if not isinstance(body, dict) or 'ros__parameters' not in body:
            continue
        live = declared(node_name)
        if live is None:
            skipped.append(node_name)
            continue
        dead = [p for p in leaves(body['ros__parameters']) if p not in live]
        status = f'{len(dead)} DEAD' if dead else 'all declared'
        print(f'{node_name}: {status}')
        for p in sorted(dead):
            print(f'    DEAD  {p}')
        total_dead += len(dead)

    if skipped:
        print(f'\nnot running, so not checked: {", ".join(skipped)}')
        print('bring the whole stack up for a complete answer')
    print(f'\n{total_dead} dead parameter(s)')
    # Dead params are informational, not a build failure: a node that is not
    # running cannot be distinguished from one with a bad name here.
    return 0


if __name__ == '__main__':
    sys.exit(main())
