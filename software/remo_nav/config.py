"""YAML 설정을 점(.) 접근이 되는 객체로 읽는다."""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config.yaml"


def _to_ns(obj):
    if isinstance(obj, dict):
        return SimpleNamespace(**{k: _to_ns(v) for k, v in obj.items()})
    if isinstance(obj, list):
        return [_to_ns(v) for v in obj]
    return obj


def load_config(path: str | Path | None = None) -> SimpleNamespace:
    with open(path or DEFAULT_CONFIG, encoding="utf-8") as f:
        return _to_ns(yaml.safe_load(f))
