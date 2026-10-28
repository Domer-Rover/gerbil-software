#!/usr/bin/env python3
"""Check that the wheel geometry in the URDF matches diff_drive_controller's config.

mobile_base.xacro and <robot>_controllers.yaml each hold their own copy of
wheel_radius and wheel_separation. If they drift apart, RViz shows one robot
while odometry is computed for another, so fail CI instead.

This file is identical in capybara-software and gerbil-software: the robot name
is discovered from the workspace layout rather than hardcoded.
"""
import glob
import sys
import xml.etree.ElementTree as ET

import yaml

PARAMS = ('wheel_radius', 'wheel_separation')
XACRO_NS = '{http://www.ros.org/wiki/xacro}'


def find_one(pattern):
    matches = sorted(glob.glob(pattern))
    if len(matches) != 1:
        sys.exit(f'expected exactly one match for {pattern}, found {matches}')
    return matches[0]


def main():
    xacro_path = find_one('src/*_description/urdf/mobile_base.xacro')
    controllers_path = find_one('src/*_bringup/config/*_controllers.yaml')

    urdf = {
        prop.get('name'): prop.get('value')
        for prop in ET.parse(xacro_path).getroot().iter(XACRO_NS + 'property')
    }
    with open(controllers_path) as f:
        controller = yaml.safe_load(f)['diff_drive_controller']['ros__parameters']

    ok = True
    for name in PARAMS:
        # float() also rejects an expression like "${...}", which this check can't evaluate
        urdf_value = float(urdf[name])
        yaml_value = float(controller[name])
        status = 'ok' if abs(urdf_value - yaml_value) < 1e-9 else 'MISMATCH'
        print(f'{name}: {xacro_path}={urdf_value} {controllers_path}={yaml_value} [{status}]')
        if status != 'ok':
            print(f'::error file={controllers_path}::{name} differs from {xacro_path}')
            ok = False
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
