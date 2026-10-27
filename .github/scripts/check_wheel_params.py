#!/usr/bin/env python3
"""Check that the wheel geometry in the URDF matches diff_drive_controller's config.

The URDF (mobile_base.xacro) and gerbil_controllers.yaml each hold their own copy of
wheel_radius and wheel_separation. If they drift apart, RViz shows one robot while
odometry is computed for another, so fail CI instead.
"""
import sys
import xml.etree.ElementTree as ET

import yaml

XACRO = 'src/gerbil_description/urdf/mobile_base.xacro'
CONTROLLERS = 'src/gerbil_bringup/config/gerbil_controllers.yaml'
PARAMS = ('wheel_radius', 'wheel_separation')
XACRO_NS = '{http://www.ros.org/wiki/xacro}'


def main():
    urdf = {
        prop.get('name'): prop.get('value')
        for prop in ET.parse(XACRO).getroot().iter(XACRO_NS + 'property')
    }
    with open(CONTROLLERS) as f:
        controller = yaml.safe_load(f)['diff_drive_controller']['ros__parameters']

    ok = True
    for name in PARAMS:
        # float() also rejects an expression like "${...}", which this check can't evaluate
        urdf_value = float(urdf[name])
        yaml_value = float(controller[name])
        status = 'ok' if abs(urdf_value - yaml_value) < 1e-9 else 'MISMATCH'
        print(f'{name}: {XACRO}={urdf_value} {CONTROLLERS}={yaml_value} [{status}]')
        if status != 'ok':
            print(f'::error file={CONTROLLERS}::{name} differs from {XACRO}')
            ok = False
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
