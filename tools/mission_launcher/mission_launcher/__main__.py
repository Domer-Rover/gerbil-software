"""python -m mission_launcher [--missions PATH]"""

import argparse
import sys

from .config import load_registry


def main() -> int:
    parser = argparse.ArgumentParser(prog='mission_launcher', description=__doc__)
    parser.add_argument(
        '--missions', metavar='PATH',
        help='missions.yaml to load (default: src/*_bringup/config/missions.yaml '
             'in this checkout)')
    args = parser.parse_args()
    registry = load_registry(args.missions)

    # Imported late so --help and registry errors work without PyQt5
    from .gui import run_app
    return run_app(registry)


if __name__ == '__main__':
    sys.exit(main())
