"""Load the mission registry (src/<robot>_bringup/config/missions.yaml)."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml

NETWORK_MODES = ('required', 'tolerant')

# tools/mission_launcher/mission_launcher/config.py -> repo root
REPO_ROOT = Path(__file__).resolve().parents[3]


@dataclass
class Mission:
    id: str
    title: str
    launch: str
    description: str = ''
    args: Dict[str, object] = field(default_factory=dict)
    network: str = 'required'
    checks: Dict[str, str] = field(default_factory=dict)   # topic -> label


@dataclass
class Registry:
    robot: str
    package: str
    missions: List[Mission]
    path: Optional[Path] = None

    def mission(self, mission_id: str) -> Mission:
        for m in self.missions:
            if m.id == mission_id:
                return m
        raise KeyError(mission_id)

    def mission_for_launch(self, launch_file: str) -> Optional[Mission]:
        return next((m for m in self.missions if m.launch == launch_file), None)


def find_registry(root: Path = REPO_ROOT) -> Path:
    """The one src/*_bringup/config/missions.yaml in this checkout."""
    found = sorted(root.glob('src/*_bringup/config/missions.yaml'))
    if not found:
        raise FileNotFoundError(
            f'no src/*_bringup/config/missions.yaml under {root}; '
            'pass --missions PATH')
    return found[0]


def parse_registry(data: dict, path: Optional[Path] = None) -> Registry:
    where = str(path) if path else 'missions.yaml'
    if not isinstance(data, dict):
        raise ValueError(f'{where}: expected a mapping at the top level')
    for key in ('robot', 'package', 'missions'):
        if key not in data:
            raise ValueError(f'{where}: missing "{key}"')

    missions = []
    seen = set()
    for i, entry in enumerate(data['missions'] or []):
        label = f'{where}: missions[{i}]'
        for key in ('id', 'title', 'launch'):
            if key not in entry:
                raise ValueError(f'{label}: missing "{key}"')
        if entry['id'] in seen:
            raise ValueError(f'{label}: duplicate id "{entry["id"]}"')
        seen.add(entry['id'])
        network = entry.get('network', 'required')
        if network not in NETWORK_MODES:
            raise ValueError(
                f'{label}: network must be one of {NETWORK_MODES}, got "{network}"')
        missions.append(Mission(
            id=str(entry['id']),
            title=str(entry['title']),
            launch=str(entry['launch']),
            description=str(entry.get('description', '')),
            args=dict(entry.get('args') or {}),
            network=network,
            checks={str(k): str(v) for k, v in (entry.get('checks') or {}).items()},
        ))

    return Registry(str(data['robot']), str(data['package']), missions, path)


def load_registry(path: Optional[Path] = None) -> Registry:
    path = Path(path) if path else find_registry()
    with open(path, encoding='utf-8') as f:
        return parse_registry(yaml.safe_load(f), path)
