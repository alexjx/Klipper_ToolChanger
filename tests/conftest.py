from __future__ import annotations

import configparser
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
FOUR_PHYSICAL = ROOT / "tests" / "fixtures" / "four_physical"


def read_fixture_config(name: str) -> configparser.ConfigParser:
    """Read a fixture file using Klipper's INI-like continuation rules."""
    parser = configparser.ConfigParser(
        interpolation=None,
        delimiters=("=", ":"),
        strict=True,
    )
    with (FOUR_PHYSICAL / name).open(encoding="utf-8") as config_file:
        parser.read_file(config_file)
    return parser


@pytest.fixture
def fixture_root() -> Path:
    return FOUR_PHYSICAL


@pytest.fixture
def tools_config() -> configparser.ConfigParser:
    return read_fixture_config("tools.cfg")


@pytest.fixture
def variables_config() -> configparser.ConfigParser:
    return read_fixture_config("variables.cfg")
