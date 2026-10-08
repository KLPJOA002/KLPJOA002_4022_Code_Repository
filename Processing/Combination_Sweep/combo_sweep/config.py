"""sweep_config.yaml -> Config, with paths resolved against the file's folder."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
PACKAGE_ROOT = HERE.parent
PROCESSING = PACKAGE_ROOT.parent
CROP_DIR = PROCESSING / "Crop"
EVALUATE_DIR = PROCESSING / "Evaluate"
PARAMETER_SWEEP_DIR = PROCESSING / "Parameter_Sweep"
DEFAULT_CONFIG = PACKAGE_ROOT / "sweep_config.yaml"


@dataclass
class Config:
    path: Path
    raw: dict

    def __getitem__(self, key):
        return self.raw[key]

    def get(self, key, default=None):
        return self.raw.get(key, default)

    def rel(self, p) -> Path:
        p = Path(p).expanduser()
        return p if p.is_absolute() else (self.path.parent / p).resolve()

    @property
    def data_root(self) -> Path:
        return self.rel(self.raw["data_root"])

    @property
    def experiments_root(self) -> Path:
        return self.rel(self.raw["experiments_root"])

    @property
    def crop_cache(self) -> Path:
        return self.rel(self.raw["crop_cache"])

    @property
    def fuse_binary(self) -> Path:
        return self.rel(self.raw["fuse"]["binary"])

    def reference(self, wave: str) -> Path:
        return self.rel(self.raw["references"][wave])

    def fuse_params(self, overrides: dict | None = None) -> dict:
        params = dict(self.raw["fuse"].get("params") or {})
        params.update(overrides or {})
        return params


def load_config(path: Path | None = None) -> Config:
    path = Path(path or DEFAULT_CONFIG).resolve()
    with open(path) as f:
        raw = yaml.safe_load(f)
    return Config(path, raw)


def fuse_args(params: dict) -> list[str]:
    """fuse_scans flags: key: value -> --key value, true -> bare flag,
    false/None -> omitted, a list repeats the flag (as Bulk_Process_Pipeline)."""
    out = []
    for key, value in params.items():
        flag = "--" + str(key).lstrip("-")
        if value is True:
            out.append(flag)
        elif value is False or value is None:
            continue
        elif isinstance(value, list):
            for v in value:
                out += [flag, str(v)]
        else:
            out += [flag, str(value)]
    return out
