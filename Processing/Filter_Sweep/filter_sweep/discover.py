"""Find the 5 scans of every (shape, height).

Layout on disk (rs_marker -D <shape>/<height>):

    <shape>/<height>/<session>/<session>_r<rot>/<session>_r<rot>_a<tilt>/
        <session>_r<rot>_a<tilt>.db3            ROS1 bag (librealsense recording)
        <session>_r<rot>_a<tilt>_detections.csv AprilTag detections, rs_marker frame index
        <session>_r<rot>_a<tilt>.ply            rs_marker's best frame, unfiltered
        imu_a<tilt>.csv                         its frame index
        scan_info.txt

Each height has two sessions, in either time order:
  A - "still" session: four _r<rot>_a0 scans (r0, r45, r90, r135). rs_marker
      auto-labels rotations; nothing was rotated, so these are four repeats
      of one pose. The labels are kept for reference only.
  B - tilt sweep: _a<tilt> folders with non-zero tilts. Only _r0_a0 is used.
A session is B if it has any non-zero tilt folder.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

SCAN_DIR = re.compile(r"^(?P<session>\d{8}_\d{6})_r(?P<rot>\d+)_a(?P<tilt>-?\d+)$")


@dataclass(frozen=True)
class Scan:
    shape: str
    height: str          # folder name, e.g. "863mm"
    height_mm: int
    scan_id: str         # A1..A4 or B
    session: str
    session_kind: str    # "A" (still repeats) or "B" (tilt sweep, R0 A0 only)
    rotation_label: int  # as labelled on disk - unreliable for session A
    tilt_deg: int
    scan_dir: Path

    @property
    def key(self) -> str:
        return f"{self.shape}__{self.height}__{self.scan_id}"

    @property
    def bag(self) -> Path:
        return self.scan_dir / f"{self.scan_dir.name}.db3"

    @property
    def detections(self) -> Path:
        return self.scan_dir / f"{self.scan_dir.name}_detections.csv"

    @property
    def scan_info(self) -> Path:
        return self.scan_dir / "scan_info.txt"

    @property
    def height_dir(self) -> Path:
        return self.scan_dir.parent.parent.parent

    def meta(self) -> dict:
        return {"shape": self.shape, "height": self.height, "height_mm": self.height_mm,
                "scan_id": self.scan_id, "scan_key": self.key, "session": self.session,
                "session_kind": self.session_kind, "rotation_label": self.rotation_label,
                "rotation_label_reliable": self.session_kind != "A", "tilt_deg": self.tilt_deg,
                "scan_dir": str(self.scan_dir)}


def _scan_dirs(session_dir: Path):
    for rot_dir in sorted(p for p in session_dir.iterdir() if p.is_dir()):
        for scan_dir in sorted(p for p in rot_dir.iterdir() if p.is_dir()):
            m = SCAN_DIR.match(scan_dir.name)
            if m and m["session"] == session_dir.name:
                yield scan_dir, int(m["rot"]), int(m["tilt"])


def discover(data_root: Path, shapes, heights="all", scans="all"):
    """Returns (list of Scan, list of warnings)."""
    found, warnings = [], []
    for shape in shapes:
        shape_dir = Path(data_root) / shape
        if not shape_dir.is_dir():
            warnings.append(f"{shape}: no folder {shape_dir}")
            continue
        height_dirs = sorted((p for p in shape_dir.iterdir() if p.is_dir() and re.fullmatch(r"\d+mm", p.name)),
                             key=lambda p: int(p.name[:-2]))
        for hdir in height_dirs:
            if heights != "all" and hdir.name not in heights:
                continue
            sessions = {}
            for sdir in sorted(p for p in hdir.iterdir() if p.is_dir()):
                dirs = list(_scan_dirs(sdir))
                if dirs:
                    sessions[sdir] = dirs
            kinds = {s: ("B" if any(t != 0 for _, _, t in d) else "A") for s, d in sessions.items()}
            if sorted(kinds.values()) != ["A", "B"]:
                warnings.append(f"{shape}/{hdir.name}: expected one still and one tilt session, "
                                f"found {sorted(kinds.values())}")
            for sdir, dirs in sessions.items():
                if kinds[sdir] == "A":
                    still = sorted(((r, d) for d, r, t in dirs if t == 0), key=lambda x: x[0])
                    if len(still) != 4:
                        warnings.append(f"{shape}/{hdir.name}/{sdir.name}: {len(still)} still scans (expected 4)")
                    picks = [(f"A{i + 1}", r, d) for i, (r, d) in enumerate(still)]
                else:
                    r0a0 = [(r, d) for d, r, t in dirs if r == 0 and t == 0]
                    if not r0a0:
                        warnings.append(f"{shape}/{hdir.name}/{sdir.name}: no _r0_a0 scan")
                    picks = [("B", r, d) for r, d in r0a0[:1]]
                for scan_id, rot, d in picks:
                    if scans != "all" and scan_id not in scans:
                        continue
                    scan = Scan(shape, hdir.name, int(hdir.name[:-2]), scan_id, sdir.name, kinds[sdir],
                                rot, 0, d)
                    missing = [p.name for p in (scan.bag, scan.detections, scan.scan_info) if not p.is_file()]
                    if missing:
                        warnings.append(f"{scan.key}: missing {missing}, skipped")
                        continue
                    found.append(scan)
    return found, warnings


def read_scan_info(path: Path) -> dict:
    info = {}
    for line in Path(path).read_text(errors="replace").splitlines():
        if ":" in line and not line.startswith("#"):
            k, _, v = line.partition(":")
            info[k.strip()] = v.strip()
    return info
