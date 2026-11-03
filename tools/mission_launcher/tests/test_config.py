"""missions.yaml parsing, plus a check of the repo's real registry."""

import pytest

from mission_launcher.config import REPO_ROOT, find_registry, load_registry, parse_registry


def test_repo_registry_launch_files_exist():
    registry = load_registry(find_registry())
    launch_dir = REPO_ROOT / 'src' / registry.package / 'launch'
    assert registry.missions, 'no missions defined'
    for mission in registry.missions:
        launch = launch_dir / mission.launch
        assert launch.is_file(), f'{mission.id}: {launch} does not exist'
        # Every GUI toggle must be a real argument of the launch file
        source = launch.read_text(encoding='utf-8')
        for arg in mission.args:
            assert f"'{arg}'" in source, f'{mission.id}: launch has no arg {arg}'
        assert mission.checks, f'{mission.id}: no health checks'


def minimal(**mission):
    entry = {'id': 'a', 'title': 'A', 'launch': 'a.launch.py'}
    entry.update(mission)
    return {'robot': 'r', 'package': 'r_bringup', 'missions': [entry]}


def test_defaults():
    m = parse_registry(minimal()).missions[0]
    assert m.network == 'required' and m.args == {} and m.checks == {}


def test_rejects_bad_network():
    with pytest.raises(ValueError, match='network'):
        parse_registry(minimal(network='sometimes'))


def test_rejects_missing_key():
    data = minimal()
    del data['missions'][0]['launch']
    with pytest.raises(ValueError, match='launch'):
        parse_registry(data)


def test_rejects_duplicate_ids():
    data = minimal()
    data['missions'].append(dict(data['missions'][0]))
    with pytest.raises(ValueError, match='duplicate'):
        parse_registry(data)
