"""Sweep settings (sweep_config.yaml) and the canonical form of one filter
configuration, which is what gets hashed into a config_id for resuming."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent.parent
PROCESSING = HERE.parent
CROP_DIR = PROCESSING / "Crop"
EVALUATE_DIR = PROCESSING / "Evaluate"
PARAMETER_SWEEP_DIR = PROCESSING / "Parameter_Sweep"
DECODER_BIN = HERE / "decoder" / "build" / "rs_decode"
DEFAULT_CONFIG = HERE / "sweep_config.yaml"

# Bump when the per-job pipeline changes in a way that alters results; rows
# written by an older version are then redone.
PIPELINE_VERSION = 2

FILTERS = ("decimation", "hdr_merge", "spatial", "temporal", "hole_filling")
PARAMS = {
    "decimation": ("magnitude",),
    "hdr_merge": ("enabled",),
    "spatial": ("magnitude", "alpha", "delta", "holes_fill"),
    "temporal": ("alpha", "delta", "persistence"),
    "hole_filling": ("mode",),
}
INT_PARAMS = {"magnitude", "holes_fill", "persistence", "mode"}


@dataclass
class Settings:
    path: Path
    raw: dict
    data_root: Path
    out_dir: Path
    cache_dir: Path
    shapes: list
    heights: object
    scans: object
    references: dict
    filter_defaults: dict
    stages: dict
    smoke: dict = field(default_factory=dict)

    def __getattr__(self, name):          # plain values straight from the YAML
        raw = self.__dict__.get("raw", {})
        if name in raw:
            return raw[name]
        raise AttributeError(name)


def load_settings(path=None) -> Settings:
    path = Path(path or DEFAULT_CONFIG).resolve()
    raw = yaml.safe_load(path.read_text())
    base = path.parent
    rel = lambda p: (base / p).resolve() if not Path(p).is_absolute() else Path(p)
    return Settings(
        path=path, raw=raw, data_root=Path(raw["data_root"]), out_dir=rel(raw.get("out_dir", "results")),
        cache_dir=rel(raw.get("cache_dir", "cache")), shapes=list(raw["shapes"]),
        heights=raw.get("heights", "all"), scans=raw.get("scans", "all"),
        references={k: rel(v) for k, v in raw["references"].items()},
        filter_defaults=raw["filter_defaults"], stages=raw["stages"], smoke=raw.get("smoke", {}),
    )


# --- one filter configuration ------------------------------------------------
#
# Canonical form: every filter key present; an "off" filter is None, an "on"
# one a dict with every parameter filled in. decimation 1 and hdr_merge
# disabled are normalised to None so equal chains hash equal.

def normalise(cfg: dict, defaults: dict) -> dict:
    out = {}
    for name in FILTERS:
        value = cfg.get(name)
        if value is None or value is False or value == "off":
            out[name] = None
            continue
        if value is True:
            value = {}
        params = {**defaults.get(name, {}), **value}
        params = {k: (int(params[k]) if k in INT_PARAMS else
                      bool(params[k]) if k == "enabled" else float(params[k]))
                  for k in PARAMS[name]}
        if name == "decimation" and params["magnitude"] <= 1:
            params = None
        if name == "hdr_merge" and not params["enabled"]:
            params = None
        out[name] = params
    return out


def config_id(cfg: dict) -> str:
    return hashlib.sha1(json.dumps(cfg, sort_keys=True).encode()).hexdigest()[:12]


def config_label(cfg: dict) -> str:
    """Short human-readable chain, e.g. 'dec2 | sp m2 a0.5 d20 h0 | tp a0.4 d20 p3'."""
    parts = []
    if cfg["decimation"]:
        parts.append(f"dec{cfg['decimation']['magnitude']}")
    if cfg["hdr_merge"]:
        parts.append("hdr")
    if cfg["spatial"]:
        s = cfg["spatial"]
        parts.append(f"sp m{s['magnitude']} a{s['alpha']:g} d{s['delta']:g} h{s['holes_fill']}")
    if cfg["temporal"]:
        t = cfg["temporal"]
        parts.append(f"tp a{t['alpha']:g} d{t['delta']:g} p{t['persistence']}")
    if cfg["hole_filling"]:
        parts.append(f"hf{cfg['hole_filling']['mode']}")
    return " | ".join(parts) or "baseline (no filters)"


def flat_columns(cfg: dict) -> dict:
    """Filter settings as flat table columns (NaN-free: off filters get on=False)."""
    row = {}
    for name in FILTERS:
        p = cfg[name]
        row[f"{name}_on"] = p is not None
        for k in PARAMS[name]:
            if k == "enabled":
                continue
            row[f"{name}_{k}"] = None if p is None else p[k]
    row["decimation_magnitude"] = 1 if cfg["decimation"] is None else cfg["decimation"]["magnitude"]
    return row


def chain_spec(cfg: dict) -> str:
    """The same chain as rs_decode's --filters argument (for cross-checking)."""
    items = []
    if cfg["decimation"]:
        items.append(f"dec={cfg['decimation']['magnitude']}")
    if cfg["spatial"]:
        s = cfg["spatial"]
        items.append(f"spatial={s['magnitude']}:{s['alpha']:g}:{s['delta']:g}:{s['holes_fill']}")
    if cfg["temporal"]:
        t = cfg["temporal"]
        items.append(f"temporal={t['alpha']:g}:{t['delta']:g}:{t['persistence']}")
    if cfg["hole_filling"]:
        items.append(f"hole={cfg['hole_filling']['mode']}")
    return ",".join(items)
