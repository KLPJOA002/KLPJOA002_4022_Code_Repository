"""Discover scans and parse rs_marker's scan_info.txt into one flat record.

Layout under the data directory:

    <Object>/<session>/<session>_r<rot>/<session>_r<rot>_a<tilt>/
        <..>.ply  <..>.db3  <..>_detections.csv  imu_a<tilt>.csv  scan_info.txt

scan_info.txt is `key: value` lines with `#` section headers. Every key is
kept (prefixed `info.`), and the sweep parameters are also derived into clean
typed columns (resolution, depth_units_mm, preset, ...).
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

# rs_marker's -p name -> the Visual Preset name the camera reports for it.
# remove_ir_pattern is a custom JSON, which the camera reports as "Custom".
EXPECTED_VISUAL_PRESET = {
    "default": "Default",
    "high_accuracy": "High Accuracy",
    "high_density": "High Density",
    "medium_density": "Medium Density",
    "remove_ir_pattern": "Custom",
}


@dataclass
class Scan:
    object: str
    session: str
    scan_dir: Path
    info_path: Path
    ply: Path | None
    n_ply: int


def find_scans(data_dir: Path, objects) -> list[Scan]:
    scans = []
    for obj in objects:
        root = data_dir / obj
        if not root.is_dir():
            continue
        for info in sorted(root.rglob("scan_info.txt")):
            plys = sorted(info.parent.glob("*.ply"))
            session = info.relative_to(root).parts[0]
            scans.append(Scan(obj, session, info.parent, info, plys[0] if plys else None, len(plys)))
    return scans


def read_scan_info(path: Path) -> dict:
    info = {}
    for line in Path(path).read_text(errors="replace").splitlines():
        if not line.strip() or line.lstrip().startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        info[key.strip()] = value.strip()
    return info


def _num(text, cast=float):
    try:
        return cast(str(text).split()[0])
    except (ValueError, IndexError, TypeError):
        return None


def _enum_name(text):
    """'4  (High Density)' -> 'High Density'; '150' -> '150'."""
    if text is None:
        return None
    m = re.search(r"\((.*)\)\s*$", text)
    return m.group(1) if m else text


def derive_parameters(info: dict) -> dict:
    """Typed sweep parameters from the raw scan_info keys."""
    out = {}
    res = info.get("resolution", "")
    m = re.match(r"(\d+)x(\d+)(?:@(\d+))?", res)
    if m:
        w, h = int(m.group(1)), int(m.group(2))
        out.update(resolution=f"{w}x{h}", res_width=w, res_height=h, res_megapixels=round(w * h / 1e6, 4),
                   fps=_num(m.group(3), int) if m.group(3) else None)
    units = _num(info.get("depth_units_m") or info.get("Stereo Module.Depth Units"))
    out["depth_units_mm"] = None if units is None else round(units * 1e3, 4)

    preset = info.get("preset")
    actual = _enum_name(info.get("Stereo Module.Visual Preset"))
    out["preset"] = preset
    out["preset_actual"] = actual
    expected = EXPECTED_VISUAL_PRESET.get(preset)
    out["preset_mismatch"] = bool(expected and actual and actual != expected)

    out["laser_power"] = _num(info.get("Stereo Module.Laser Power"))
    out["emitter"] = _enum_name(info.get("Stereo Module.Emitter Enabled"))
    out["auto_exposure"] = _num(info.get("Stereo Module.Enable Auto Exposure"), int)
    out["exposure_us"] = _num(info.get("Stereo Module.Exposure"))
    out["gain"] = _num(info.get("Stereo Module.Gain"))
    out["filter_chain"] = info.get("filter_chain")
    out["frames_captured"] = _num(info.get("frames_captured"), int)
    out["best_frame"] = _num(info.get("best_frame"), int)
    vp = re.match(r"(\d+)\s*/\s*(\d+)", info.get("best_valid_pixels", ""))
    out["valid_pixel_fraction"] = round(int(vp.group(1)) / int(vp.group(2)), 5) if vp else None
    out["camera"] = info.get("device.Name")
    out["camera_serial"] = info.get("device.Serial Number")
    out["firmware"] = info.get("device.Firmware Version")
    return out


def previous_presets(scans) -> dict:
    """{(object, session): preset requested by the capture just before it}.
    remove_ir_pattern appears to inherit settings from the previous preset,
    so this lets that be checked in the data. Uses every scan of the object,
    not just the ones being evaluated."""
    out = {}
    by_obj = {}
    for s in scans:
        by_obj.setdefault(s.object, []).append(s)
    for group in by_obj.values():
        prev = None
        for s in sorted(group, key=lambda s: s.session):
            out[(s.object, s.session)] = prev
            prev = read_scan_info(s.info_path).get("preset")
    return out
