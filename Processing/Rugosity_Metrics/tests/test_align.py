"""Moving the scan onto the reference: --flip / --rotate-deg / --shift-mm,
--refine (ICP), --transform round trip, the wrong-pose checks, the --adjust
window's key handling, and the mesh repair that must not cut holes in a
height-field reference."""

import csv

import numpy as np
import pytest
from conftest import PART_MM, ROOT, S, write_part_scan, write_part_stl

import roughness_scans
from roughness.align import pose_matrix
from scan_eval.registration import apply

CENTRE = np.array([PART_MM / 2, PART_MM / 2, 5.0]) * 1e-3      # the part's bounding-box centre, m


def rows(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def main(*argv):
    return roughness_scans.main([str(a) for a in argv])


def run_one(scan, ref, out, *flags):
    assert main(scan, "-o", out, "--reference", ref, "--grid-mm", "1", "--no-plots",
                "--bootstrap-samples", "50", "--label", "t", *flags) == 0
    return rows(out / "roughness_results.csv")[0]


def moved_scan(aligned_scan, tmp_path, T):
    from scan_eval.pipeline import load_scan
    pts = load_scan(aligned_scan, "m", 1.0)
    path = tmp_path / "moved.ply"
    S.write_ply(path, apply(T, pts))
    return path


def test_manual_moves_undo_a_known_displacement(part_stl, aligned_scan, tmp_path):
    # Displace by: shift (4, -3, 1.5) mm, then rotate 7 deg about the centre.
    # Undo in the pipeline's order: rotate -7 deg, then shift back.
    T = pose_matrix(np.eye(4), False, 7.0, None, CENTRE) @ pose_matrix(np.eye(4), False, 0, [4, -3, 1.5], CENTRE)
    scan = moved_scan(aligned_scan, tmp_path, T)
    bad = run_one(scan, part_stl, tmp_path / "bad")
    assert "ALIGN" in bad["flags"].split()
    good = run_one(scan, part_stl, tmp_path / "good", "--rotate-deg", "-7", "--shift-mm", "-4,3,-1.5")
    assert not {"ALIGN", "UPSIDEDOWN", "ALIGNCOVER"} & set(good["flags"].split())
    assert abs(float(good["err_pct_Sq_mm"])) < 1.0
    assert float(good["align_median_dz_mm"]) < 0.05


def test_refine_recovers_a_small_offset(part_stl, aligned_scan, tmp_path):
    T = pose_matrix(np.eye(4), False, 2.0, [2, -1.5, 1], CENTRE)
    scan = moved_scan(aligned_scan, tmp_path, T)
    r = run_one(scan, part_stl, tmp_path / "o", "--refine")
    assert r["align_refined"] == "True"
    assert not {"ALIGN", "UPSIDEDOWN"} & set(r["flags"].split())
    assert abs(float(r["err_pct_Sq_mm"])) < 1.0
    assert float(r["align_moved_mm"]) == pytest.approx(np.hypot(np.hypot(2, 1.5), 1), rel=0.3)


def bumps_mm(x, y):
    """Bumps that all point up, like pyramids: turned over they become pits.
    (The egg carton looks the same either way up, so it can't test this.)"""
    return 3.0 + 4.0 * np.abs(np.sin(np.pi * x / 25.0) * np.sin(np.pi * y / 25.0))


def test_upside_down_is_detected_and_flip_fixes_it(tmp_path):
    part_stl = write_part_stl(tmp_path / "bumps.stl", bumps_mm)
    aligned = write_part_scan(tmp_path / "bumps.ply", bumps_mm)
    centre = np.array([PART_MM / 2, PART_MM / 2, 5.0]) * 1e-3       # bbox centre: z 3..7 mm
    scan = moved_scan(aligned, tmp_path, pose_matrix(np.eye(4), True, 0, None, centre))
    bad = run_one(scan, part_stl, tmp_path / "bad")
    assert "UPSIDEDOWN" in bad["flags"].split()
    good = run_one(scan, part_stl, tmp_path / "good", "--flip")
    assert not {"ALIGN", "UPSIDEDOWN"} & set(good["flags"].split())
    assert abs(float(good["err_pct_Sq_mm"])) < 1.0


def test_saved_transform_reproduces_the_pose(part_stl, aligned_scan, tmp_path):
    T = pose_matrix(np.eye(4), False, 3.0, [1, 1, 0.5], CENTRE)
    scan = moved_scan(aligned_scan, tmp_path, T)
    a = run_one(scan, part_stl, tmp_path / "a", "--rotate-deg", "-3", "--shift-mm", "-1,-1,-0.5", "--refine")
    saved = tmp_path / "a" / "t__moved" / "transform.txt"
    assert "reuse with" in saved.read_text()
    b = run_one(scan, part_stl, tmp_path / "b", "--transform", saved)
    for k in ("Sq_mm", "Sdr_pct", "ref_Sq_mm", "err_Sq_mm", "align_median_dz_mm", "n_points"):
        assert a[k] == b[k], k


def test_align_flags_need_a_reference(aligned_scan, tmp_path, capsys):
    assert main(aligned_scan, "-o", tmp_path / "o", "--refine") == 1
    assert "need a --reference" in capsys.readouterr().err


class Ev:
    """Stand-in for a matplotlib key / mouse event."""
    def __init__(self, key=None, inaxes=None, xdata=None, ydata=None):
        self.key, self.inaxes, self.xdata, self.ydata = key, inaxes, xdata, ydata


def make_aligner(part_stl, aligned_scan, start):
    from roughness.align import Aligner
    from roughness.params import RoughnessParams
    from roughness.validate import load_height_reference
    from scan_eval.pipeline import load_scan
    ref = load_height_reference(RoughnessParams(reference_def=str(part_stl)))
    pts = load_scan(aligned_scan, "m", 1.0)
    return Aligner(ref, pts, start, RoughnessParams(), np.random.default_rng(0))


def test_aligner_keys_headless(part_stl, aligned_scan):
    import matplotlib.pyplot as plt
    start = pose_matrix(np.eye(4), False, 0, [3, 0, 0], CENTRE)        # 3 mm too far in +x
    al = make_aligner(part_stl, aligned_scan, start)
    rms0 = al.stats["rms_dz_mm"]
    assert al.step == 1.0
    for _ in range(3):
        al.on_key(Ev("left"))
    # Not ~0: cells on the part's edge also hold table points, 5 mm lower.
    assert al.shift[0] == -3 and al.stats["rms_dz_mm"] < 0.4 * rms0
    al.on_key(Ev("u"))
    assert al.shift[0] == -2
    al.on_key(Ev("1"))
    al.on_key(Ev("j"))
    assert al.shift[0] == pytest.approx(-2.1) and "last key: j" in al.status.get_text()
    al.on_key(Ev("4"))
    al.on_key(Ev("w"))
    assert al.shift[2] == 100
    al.on_key(Ev("s"))
    al.on_key(Ev("pagedown"))
    assert al.shift[2] == -100
    al.on_key(Ev("2"))
    al.on_key(Ev("+"))
    al.on_key(Ev("i"))
    assert al.shift[2] == -99 and al.shift[1] == 1
    al.on_key(Ev("q"))
    assert "q (unused)" in al.status.get_text()
    al.on_key(Ev("f"))
    assert al.flip
    al.on_key(Ev("r"))
    al.on_key(Ev("]"))
    assert al.rot == 91.0
    al.on_key(Ev("0"))                                                   # reset to the start
    al.on_key(Ev("a"))                                                   # ICP folds into the base
    assert al.rot == 0 and al.stats["rms_dz_mm"] < 0.4 * rms0
    al.on_key(Ev("enter"))
    assert al.accepted
    plt.close("all")


def test_aligner_buttons_click_and_display(part_stl, aligned_scan):
    import matplotlib.pyplot as plt
    al = make_aligner(part_stl, aligned_scan, np.eye(4))
    labels = [b.label.get_text() for b in al.buttons]
    assert {"z +", "z -", "90°", "place", "fit", "accept"} <= set(labels)
    al.do(("move", 2, 1))                                # what the "z +" button calls
    assert al.shift[2] == 1
    al.do(("cycle",))
    assert al.step == 10.0
    al.on_click(Ev(inaxes=al.ax_h, xdata=20.0, ydata=70.0))
    assert abs(al.cx[al.col] - 20) <= al.cmp.geom.g * 1e3 and abs(al.cy[al.row] - 70) <= al.cmp.geom.g * 1e3
    # The displayed map has no holes over the part.
    shown = al.im_h.get_array()
    assert np.isfinite(np.ma.filled(shown, np.nan)[al.cmp.outline]).mean() > 0.99
    plt.close("all")


def test_display_cell_fits_a_sparse_scan(part_stl):
    from roughness.align import Comparison, display_cell, fill_for_display
    from roughness.params import RoughnessParams
    from roughness.validate import load_height_reference
    ref = load_height_reference(RoughnessParams(reference_def=str(part_stl)))
    rng = np.random.default_rng(3)
    xy = rng.uniform(0, 0.1, (7000, 2))                                # ~1 point per 1.4 mm^2
    pts = np.column_stack([xy, ref.height(xy[:, 0], xy[:, 1])])
    g = display_cell(ref, pts, spacing=0.6e-3)
    cmp = Comparison(ref, g=g, margin=0.3)
    z = cmp.scan_map(pts)
    assert cmp.fill_fraction(z) >= 0.9
    assert np.isfinite(fill_for_display(z)[cmp.outline]).all()


def camera_frame_scan(tmp_path, height_mm):
    """The part on a 500 x 500 mm bed, as a camera 700 mm above sees it: bed
    tilted 8 deg, turned 37 deg, offset, z pointing at the camera."""
    rng = np.random.default_rng(2)
    xy = rng.uniform(-200, 300, (900000, 2))
    on = np.all((xy >= 0) & (xy <= PART_MM), axis=1)
    z = np.where(on, height_mm(xy[:, 0], xy[:, 1]), 0.0) + rng.normal(0, 0.05, len(xy))
    part = np.column_stack([xy, z]) * 1e-3
    from scan_eval.registration import rotation_about, translation
    T = (translation(np.array([-0.12, 0.06, -0.70])) @ rotation_about(np.array([1.0, 0.3, 0]), np.radians(8), np.zeros(3))
         @ rotation_about(np.array([0, 0, 1.0]), np.radians(37), np.zeros(3)))
    path = tmp_path / "camera.ply"
    S.write_ply(path, apply(T, part))
    return path


def test_auto_place_finds_the_part_in_a_raw_scan(tmp_path):
    part_stl = write_part_stl(tmp_path / "bumps.stl", bumps_mm)
    scan = camera_frame_scan(tmp_path, bumps_mm)
    r = run_one(scan, part_stl, tmp_path / "o")
    assert r["align_auto_placed"] == "True"
    assert not {"ALIGN", "ALIGNCOVER", "UPSIDEDOWN"} & set(r["flags"].split())
    assert float(r["align_median_dz_mm"]) < 0.3
    assert abs(float(r["err_pct_Sq_mm"])) < 2.0
    off = run_one(scan, part_stl, tmp_path / "off", "--no-auto-place")
    assert off["align_auto_placed"] == "False" and "ALIGN" in off["flags"].split()


def test_height_field_mesh_keeps_stacked_faces():
    """PYRAMID_2X2.stl is pyramids stacked on a base plate; scan_eval's
    non-manifold repair drops 4 pyramid faces, which would leave diamond holes."""
    path = ROOT.parent / "Evaluate" / "References" / "PYRAMID_2X2.stl"
    if not path.is_file():
        pytest.skip("PYRAMID_2X2.stl not present")
    from roughness.params import RoughnessParams
    from roughness.validate import load_height_reference
    ref = load_height_reference(RoughnessParams(reference_def=str(path)))
    # On the face of the pyramid centred (25, 25) mm: apex 50 mm, base 10 mm at x = 50.
    h = ref.height(np.array([0.040, 0.045]), np.array([0.025, 0.025])) * 1e3
    assert h == pytest.approx([26.0, 18.0], abs=0.1)
