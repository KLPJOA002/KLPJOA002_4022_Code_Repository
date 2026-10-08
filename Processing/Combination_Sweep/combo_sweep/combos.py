"""Combination families from the config: rotation family x tilt family grid,
plus single-scan baselines. Ids are human readable and stable:

  rot-<rotation family>_tilt-<tilt family>     fused combination
  single_r<rot>_a<tilt>                        one scan, unfused
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Combo:
    combo_id: str
    kind: str                 # fused | single
    family: str               # all | rotation | tilt | joint | single
    rot_family: str
    tilt_family: str
    rotations: tuple[int, ...]
    tilts: tuple[int, ...]

    @property
    def n_scans(self) -> int:
        return len(self.rotations) * len(self.tilts)

    def scans(self, session):
        """The session's scans in this combination. Raises if one is missing."""
        out = []
        for r in self.rotations:
            for t in self.tilts:
                s = session.scan(r, t)
                if s is None:
                    raise ValueError(f"{session.wave}/{session.height}: no scan r{r}_a{t}")
                out.append(s)
        return out


def _resolve(values, available) -> tuple[int, ...]:
    if values == "all":
        return tuple(sorted(available))
    vals = tuple(sorted(int(v) for v in values))
    missing = set(vals) - set(available)
    if missing:
        raise SystemExit(f"Error: family uses angles not in the data: {sorted(missing)} "
                         f"(available {sorted(available)})")
    return vals


def family_of(rot_name: str, tilt_name: str) -> str:
    if rot_name == "all" and tilt_name == "all":
        return "all"
    if tilt_name == "all":
        return "rotation"
    if rot_name == "all":
        return "tilt"
    return "joint"


def fused_combos(cfg, rotations_available, tilts_available) -> list[Combo]:
    rf, tf = cfg["rotation_families"], cfg["tilt_families"]
    grid = cfg["grid"]
    out = []
    for rname in grid["rotations"]:
        for tname in grid["tilts"]:
            rname, tname = str(rname), str(tname)
            if rname not in rf or tname not in tf:
                raise SystemExit(f"Error: grid names unknown family '{rname}' or '{tname}'")
            rots = _resolve(rf[rname], rotations_available)
            tilts = _resolve(tf[tname], tilts_available)
            if len(rots) * len(tilts) < 2:
                continue  # a single scan: covered by the single-scan baselines
            out.append(Combo(f"rot-{rname}_tilt-{tname}", "fused", family_of(rname, tname),
                             rname, tname, rots, tilts))
    ids = [c.combo_id for c in out]
    if len(ids) != len(set(ids)):
        raise SystemExit("Error: duplicate combination ids in the grid")
    return out


def single_combos(rotations_available, tilts_available) -> list[Combo]:
    return [Combo(f"single_r{r}_a{t}", "single", "single", str(r), str(t), (r,), (t,))
            for r in sorted(rotations_available) for t in sorted(tilts_available)]


def all_combos(cfg, session) -> list[Combo]:
    combos = fused_combos(cfg, session.rotations, session.tilts)
    if cfg.get("single_scans", True):
        combos += single_combos(session.rotations, session.tilts)
    return combos
