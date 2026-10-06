"""Turn the stage definitions in sweep_config.yaml into filter configurations.

  one_factor : each listed parameter swept on its own, that filter on with
               its other parameters at filter_defaults, every other filter off
  grid       : full product of the spatial and temporal parameter lists (plus
               spatial-only / temporal-only rows), decimation and hole filling
               held at a value, "off" or "stage1_best"
  refine     : the top_k stage-2 combinations x the `vary` lists

The baseline (no filters) is added to every stage. Stage 3 and the
"stage1_best" holds read the results table, so stage 1 (and 2) must have run.
"""

from __future__ import annotations

import itertools

import pandas as pd

from .config import config_id, normalise
from .results import rank_configs

BASELINE = {}


def _is_off(value) -> bool:
    # YAML reads a bare `off` as False; `is` keeps mode 0 (== False) a real value
    return value is None or value is False or value == "off"


def _set(cfg: dict, dotted: str, value):
    name, param = dotted.split(".")
    if _is_off(value) and param != "enabled":
        cfg[name] = None
    else:
        cfg.setdefault(name, {})
        if cfg[name] is not None:
            cfg[name][param] = value
    return cfg


def _stage1_best(dotted: str, settings, results: pd.DataFrame, stage1_ids: set):
    """Best stage-1 value of a parameter (its one-factor rows + baseline)."""
    if results is None or results.empty:
        raise SystemExit(f"'stage1_best' for {dotted} needs stage 1 results - run stage 1 first")
    name, param = dotted.split(".")
    values = settings.stages["1"]["factors"].get(dotted, [])
    cands = {config_id(normalise({}, settings.filter_defaults)): "off"}
    for v in values:
        cands[config_id(normalise({name: {param: v}}, settings.filter_defaults))] = v
    ranked = rank_configs(results, settings.rank_metric, [c for c in cands if c in stage1_ids])
    if ranked.empty:
        raise SystemExit(f"no stage-1 results for {dotted}")
    return cands[ranked.iloc[0]["config_id"]]


def _resolve(value, dotted, settings, results, stage1_ids):
    return _stage1_best(dotted, settings, results, stage1_ids) if value == "stage1_best" else value


def build_stage(stage: str, settings, results: pd.DataFrame = None, stage_ids: dict = None) -> list:
    """List of normalised configs for a stage, baseline first, de-duplicated."""
    spec = settings.stages[stage]
    d = settings.filter_defaults
    stage_ids = stage_ids or {}
    raw = [BASELINE]

    if spec["kind"] == "one_factor":
        for dotted, values in spec["factors"].items():
            for v in values:
                raw.append(_set({}, dotted, v))

    elif spec["kind"] == "grid":
        hold = {}
        for dotted, v in (spec.get("hold") or {}).items():
            hold = _set(hold, dotted, _resolve(v, dotted, settings, results, stage_ids.get("1", set())))
        groups = {"spatial": {}, "temporal": {}}
        for dotted, values in spec["grid"].items():
            groups[dotted.split(".")[0]][dotted.split(".")[1]] = values
        options = {}
        for name, params in groups.items():
            keys = list(params)
            opts = [dict(zip(keys, combo)) for combo in itertools.product(*(params[k] for k in keys))]
            if spec.get("include_off", True):
                opts.append(None)
            options[name] = opts
        for sp, tp in itertools.product(options["spatial"], options["temporal"]):
            raw.append({**hold, "spatial": sp, "temporal": tp})

    elif spec["kind"] == "refine":
        if results is None or results.empty or not stage_ids.get("2"):
            raise SystemExit("stage 3 needs stage 2 results - run stage 2 first")
        ranked = rank_configs(results, settings.rank_metric, stage_ids["2"])
        by_id = {}
        for cfg in build_stage("2", settings, results, stage_ids):
            by_id[config_id(cfg)] = cfg
        top = [by_id[c] for c in ranked["config_id"] if c in by_id][: spec.get("top_k", 5)]
        vary = {k: [_resolve(v, k, settings, results, stage_ids.get("1", set())) for v in vals]
                for k, vals in spec["vary"].items()}
        keys = list(vary)
        for base in top:
            for combo in itertools.product(*(vary[k] for k in keys)):
                cfg = {"spatial": base["spatial"], "temporal": base["temporal"]}
                for k, v in zip(keys, combo):
                    if k == "decimation.magnitude" and (v == 1 or _is_off(v)):
                        v = "off"
                    cfg = _set(cfg, k, v)
                raw.append(cfg)
    else:
        raise SystemExit(f"stage {stage}: unknown kind {spec['kind']!r}")

    seen, out = set(), []
    for cfg in raw:
        n = normalise(cfg, d)
        cid = config_id(n)
        if cid not in seen:
            seen.add(cid)
            out.append(n)
    return out


def build_smoke(settings) -> list:
    seen, out = set(), []
    for cfg in settings.smoke.get("configs", [{}]):
        n = normalise(cfg, settings.filter_defaults)
        if config_id(n) not in seen:
            seen.add(config_id(n))
            out.append(n)
    return out
