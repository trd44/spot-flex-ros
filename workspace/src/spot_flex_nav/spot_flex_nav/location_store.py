from pathlib import Path
from typing import Any, Dict

import yaml


def default_locations_file() -> Path:
    return Path.home() / '.ros' / 'spot_flex_locations.yaml'


def load_locations(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {'locations': {}}
    with path.open('r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    if 'locations' not in data or data['locations'] is None:
        data['locations'] = {}
    return data


def save_locations(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, sort_keys=True)
