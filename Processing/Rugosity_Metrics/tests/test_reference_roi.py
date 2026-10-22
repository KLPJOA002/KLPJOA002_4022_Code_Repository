"""Comparison with a mesh (.stl) reference, the evaluated-area controls
(polygon ROI, reference footprint, --orient none, the ROI picker's logic) and
the detrend / reference plots."""

import csv
import json

import numpy as np
import pytest
from conftest import PART_MM, S, run

import roughness_scans


def rows(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def main(*argv):
    return roughness_scans.main([str(a) for a in argv])


def test_bare_stl_reference_crops_to_part_and_matches(part_stl, aligned_scan, tmp_path):
    out = tmp_path / "out"
    assert main(aligned_scan, "-o", out, "--reference", part_stl, "--grid-mm", "1", "--detrend",
                "planar,quadratic", "--bootstrap-samples", "50", "--label", "t") == 0
    table = rows(out / "roughness_results.csv")
    r = table[0]
    assert r["orientation"] == "none"
    assert float(r["footprint_x_mm"]) == pytest.approx(PART_MM, abs=2)     # the table is cropped away
    for k, tol in (("Sq_mm", 1.0), ("Sdr_pct", 5.0), ("dominant_wavelength_mm", 1.0)):
        # Sdr: the scan's 0.02 mm point noise adds a few % of slope, as it should.
        assert abs(float(r[f"err_pct_{k}"])) < tol, (k, r[f"err_pct_{k}"])
    assert not {"REFCOVER", "ALIGN", "UPSIDEDOWN"} & set(r["flags"].split())
    run_dir = out / "t__aligned"
    for name in ("roi_preview.png", "detrend_explained.png", "detrend_metrics.png",
                 "reference_comparison.png"):
        assert (run_dir / name).is_file(), name
    doc = json.loads((run_dir / "metrics.json").read_text())
    assert doc["validation"]["coverage"] > 0.98
    assert "_aux" not in doc["validation"]


def test_uncropped_comparison_uses_the_overlap(part_stl, aligned_scan, tmp_path):
    """With --reference-crop none the scan keeps the table, but the errors
    still compare the same cells (the scan is re-evaluated on the overlap)."""
    out = tmp_path / "out"
    assert main(aligned_scan, "-o", out, "--reference", part_stl, "--reference-crop", "none",
                "--grid-mm", "1", "--no-plots", "--bootstrap-samples", "50") == 0
    r = rows(out / "roughness_results.csv")[0]
    assert float(r["footprint_x_mm"]) > 150                    # whole scan evaluated
    assert "REFCOVER" in r["flags"]
    assert abs(float(r["err_pct_Sq_mm"])) < 3.0               # but like-for-like errors
    assert float(r["Sq_mm"]) != pytest.approx(float(r["ref_Sq_mm"]), rel=0.05)


def test_reference_inset_shrinks_area(part_stl, aligned_scan, tmp_path):
    out = tmp_path / "out"
    assert main(aligned_scan, "-o", out, "--reference", part_stl, "--reference-inset-mm", "10",
                "--grid-mm", "1", "--no-plots", "--bootstrap-samples", "50") == 0
    r = rows(out / "roughness_results.csv")[0]
    assert float(r["footprint_x_mm"]) == pytest.approx(PART_MM - 20, abs=2)


def test_polygon_roi(tmp_path):
    xy = S.lattice_xy(0.2, 0.2, 0.5e-3)
    pts = np.column_stack([xy, S.sinusoid(xy, 1e-3, 20e-3)])
    full = run(pts, grid_mm=1)
    tri = run(pts, grid_mm=1, roi_polygon_mm=[[-100, -100], [100, -100], [-100, 100]])
    assert len(tri.prep.points) == pytest.approx(len(full.prep.points) / 2, rel=0.02)
    assert tri.prep.crop_info["roi_polygon_mm"][1] == [100, -100]


def test_orient_none_keeps_input_frame():
    xy = S.lattice_xy(0.2, 0.16, 0.5e-3)
    z = S.sinusoid(xy, 1e-3, 20e-3) + 0.05 * xy[:, 0]          # ~3 deg tilt
    a = run(np.column_stack([xy, z]) + [0.3, 0.1, 0.0], grid_mm=1, orient_mode="none")
    assert a.prep.frame.tilt_deg == 0 and np.allclose(a.prep.frame.R, np.eye(3))
    assert a.prep.geom.x0 == pytest.approx(0.2, abs=2e-3)      # coordinates not re-centred
    # The planar detrend still removes the tilt.
    assert a.results["planar"]["Sq_mm"] == pytest.approx(1 / np.sqrt(2), rel=0.03)


def test_roi_helpers_and_saved_roi_round_trip(tmp_path):
    from roughness import roi
    assert roi.roi_from_rectangle(10, 5, -3.456, -8) == [-3.46, -8.0, 10.0, 5.0]
    poly = roi.roi_from_polygon([(0, 0), (10.004, 0), (0, 7)])
    assert roi.as_flags([1, 2, 3, 4], poly) == '--roi 1,2,3,4 --roi-polygon "0,0;10,0;0,7"'
    path = tmp_path / "roi.toml"
    roi.write_roi_toml(path, [-50, -40, 50, 40], poly, "scan.ply", "plane")
    args = roughness_scans.build_parser().parse_args(["--config", str(path), "x.ply"])
    params, _ = roughness_scans.resolve_params(args)
    assert params.roi_mm == [-50, -40, 50, 40]
    assert params.roi_polygon_mm == [[0, 0], [10, 0], [0, 7]]
    assert params.orient_mode == "plane"


def test_picker_selection_logic_headless():
    """The picker's callbacks, driven directly on the Agg backend."""
    import matplotlib.pyplot as plt

    from roughness.roi import RoiPicker
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(-0.1, 0.1, (20000, 2)), rng.normal(0, 1e-3, 20000)])
    picker = RoiPicker(pts, 1e-3, region=np.abs(pts[:, 0]) < 0.05)

    class Ev:
        def __init__(self, x, y, key=None):
            self.xdata, self.ydata, self.key = x, y, key
    picker.on_rectangle(Ev(30, 20), Ev(-30, -20))
    assert picker.roi_mm == [-30, -20, 30, 20] and picker.polygon_mm is None
    picker.on_key(Ev(0, 0, "p"))
    picker.on_polygon([(0, 0), (40, 0), (0, 40)])
    assert picker.polygon_mm == [[0, 0], [40, 0], [0, 40]] and picker.roi_mm is None
    picker.on_key(Ev(0, 0, "escape"))
    assert picker.accepted and picker.roi_mm is None and picker.polygon_mm is None
    plt.close("all")


def test_new_flags_validate(tmp_path, capsys):
    ply = tmp_path / "s.ply"
    S.write_ply(ply, np.column_stack([S.lattice_xy(0.1, 0.1, 1e-3), np.zeros(10000)]))
    assert main(ply, "-o", tmp_path / "o", "--roi-polygon", "0,0;1,1") == 1
    assert "at least 3" in capsys.readouterr().err
    assert main(ply, "-o", tmp_path / "o", "--reference-inset-mm", "-1") == 1
