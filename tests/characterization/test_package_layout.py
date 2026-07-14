from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'ktcc'


def test_toolchanger_code_is_organized_by_feature() -> None:
    expected = (
        PACKAGE / 'tools.py',
        PACKAGE / 'profiles' / 'models.py',
        PACKAGE / 'profiles' / 'service.py',
        PACKAGE / 'toolchange' / 'state.py',
        PACKAGE / 'toolchange' / 'recovery.py',
        PACKAGE / 'toolchange' / 'service.py',
        PACKAGE / 'thermal.py',
        PACKAGE / 'persistence' / 'service.py',
        PACKAGE / 'persistence' / 'codec.py',
        PACKAGE / 'profiles' / 'storage.py',
        PACKAGE / 'klipper' / 'composition.py',
        PACKAGE / 'config' / 'normalization.py',
    )
    assert all(path.is_file() for path in expected)


def test_removed_layer_packages_do_not_return() -> None:
    for layer in ('domain', 'application', 'adapters', 'compatibility'):
        assert not list((PACKAGE / layer).rglob('*.py'))

    source = '\n'.join(
        path.read_text(encoding='utf-8')
        for path in PACKAGE.rglob('*.py')
    )
    assert 'ktcc.domain' not in source
    assert 'ktcc.application' not in source
    assert 'ktcc.adapters' not in source
    assert 'ktcc.compatibility' not in source
    assert 'ktcc_state_v2' not in source
