"""Find the full-sweep session of every (wave, height) and its scans.

Layout: <wave>/<height>/<session>/<session>_r<rot>/<session>_r<rot>_a<tilt>/<...>.ply
The full-sweep session is picked by scan count (the other session has ~4).
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

ROT_RE = re.compile(r"_r(-?\d+)$")
TILT_RE = re.compile(r"_r(-?\d+)_a(-?\d+)$")


@dataclass(frozen=True)
class Scan:
    rotation: int
    tilt: int
    tilt_dir: Path          # source tilt folder (PLY, IMU, detections, scan_info)
    ply: Path

    @property
    def key(self) -> str:
        return f"r{self.rotation}_a{self.tilt}"


@dataclass
class Session:
    wave: str
    height: str
    path: Path
    scans: list[Scan]
    other_sessions: dict = field(default_factory=dict)   # name -> scan count

    @property
    def name(self) -> str:
        return self.path.name

    @property
    def height_mm(self) -> int:
        return int(re.sub(r"\D", "", self.height))

    @property
    def rotations(self) -> list[int]:
        return sorted({s.rotation for s in self.scans})

    @property
    def tilts(self) -> list[int]:
        return sorted({s.tilt for s in self.scans})

    def scan(self, rotation: int, tilt: int) -> Scan | None:
        for s in self.scans:
            if s.rotation == rotation and s.tilt == tilt:
                return s
        return None


def session_scans(session: Path) -> list[Scan]:
    scans = []
    for rot_dir in sorted(p for p in session.iterdir() if p.is_dir() and ROT_RE.search(p.name)):
        for tilt_dir in sorted(p for p in rot_dir.iterdir() if p.is_dir()):
            m = TILT_RE.search(tilt_dir.name)
            if not m:
                continue
            exact = tilt_dir / f"{tilt_dir.name}.ply"
            plys = sorted(tilt_dir.glob("*.ply"))
            ply = exact if exact in plys else (plys[0] if len(plys) == 1 else None)
            if ply is None:
                continue
            scans.append(Scan(int(m.group(1)), int(m.group(2)), tilt_dir, ply))
    return scans


def heights_of(cfg, wave: str) -> list[str]:
    wave_dir = cfg.data_root / wave
    found = sorted((p.name for p in wave_dir.iterdir() if p.is_dir() and re.fullmatch(r"\d+mm", p.name)),
                   key=lambda h: int(h[:-2]))
    want = cfg.get("heights", "all")
    if want == "all":
        return found
    missing = set(want) - set(found)
    if missing:
        raise SystemExit(f"Error: heights not found under {wave_dir}: {sorted(missing)}")
    return [h for h in found if h in want]


def full_sweep_session(cfg, wave: str, height: str) -> Session:
    height_dir = cfg.data_root / wave / height
    candidates = {}
    for s in sorted(p for p in height_dir.iterdir() if p.is_dir()):
        candidates[s] = session_scans(s)
    if not candidates:
        raise SystemExit(f"Error: no sessions in {height_dir}")
    best = max(candidates, key=lambda s: len(candidates[s]))
    n = len(candidates[best])
    if n < cfg.get("min_session_scans", 40):
        raise SystemExit(f"Error: {height_dir}: largest session {best.name} has only {n} scans")
    others = {s.name: len(v) for s, v in candidates.items() if s != best}
    return Session(wave, height, best, candidates[best], others)


def discover(cfg, waves=None, heights=None) -> list[Session]:
    sessions = []
    for wave in waves or cfg["waves"]:
        for height in heights_of(cfg, wave):
            if heights and height not in heights:
                continue
            sessions.append(full_sweep_session(cfg, wave, height))
    return sessions
