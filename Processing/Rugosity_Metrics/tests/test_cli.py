"""End to end through roughness_scans.main: outputs, determinism, noise scan,
validation, and loud failures on bad input."""

import csv
import json

import numpy as np
import pytest
from conftest import ROOT, S

import roughness_scans


def rows(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def err(capsys):
    """stderr with the warning line-wrapping undone."""
    return " ".join(capsys.readouterr().err.split())


def main(*argv):
    return roughness_scans.main([str(a) for a in argv])


def test_outputs_and_csv(ripple_ply, tmp_path):
    out = tmp_path / "out"
    assert main(ripple_ply, "-o", out, "--detrend", "planar,quadratic", "--primary-detrend", "quadratic",
                "--grid-mm", "1", "--label", "t1", "--bootstrap-samples", "100") == 0
    run = out / "t1__ripples"
    for name in ["metrics.json", "run.log", "heightmap_planar.npy", "heightmap_quadratic.npy",
                 "heightmap_planar.png", "detrend_comparison.png", "psd.png", "multiscale.png",
                 "roi_preview.png", "detrend_explained.png", "detrend_metrics.png",
                 "psd_quadratic.csv", "multiscale_quadratic.csv"]:
        assert (run / name).is_file(), name
    table = rows(out / "roughness_results.csv")
    assert [r["detrend"] for r in table] == ["planar", "quadratic"]
    assert [r["primary"] for r in table] == ["False", "True"]
    head = list(table[0])
    assert head[:8] == ["timestamp", "label", "scan", "detrend", "primary", "grid_mm", "Sq_mm", "Sa_mm"]
    for col in ["n_points", "valid_fraction", "interpolated_fraction", "footprint_x_mm", "bandwidth_min_mm",
                "n_windows_used", "Sq_ci95_lo_mm", "Sdr_ci95_hi_pct", "Sal_ci95_lo_mm", "tilt_removed_deg"]:
        assert col in head, col
    doc = json.loads((run / "metrics.json").read_text())
    assert set(doc["metrics"]) == {"planar", "quadratic"}
    assert doc["detrend"]["quadratic"]["n_coefficients"] == 6
    assert doc["versions"]["roughness_code_sha256"]
    h = np.load(run / "heightmap_quadratic.npy")
    assert h.shape == (doc["grid"]["ny"], doc["grid"]["nx"])
    assert (out / "roughness_batch_summary.csv").is_file()
    # A second run appends to the shared table, and only this batch is in the summary.
    assert main(ripple_ply, "-o", out, "--grid-mm", "1", "--no-plots", "--bootstrap-samples", "100") == 0
    assert len(rows(out / "roughness_results.csv")) == 3
    assert len(rows(out / "roughness_batch_summary.csv")) == 1


def test_same_seed_same_csv(ripple_ply, tmp_path):
    def table(out, seed):
        assert main(ripple_ply, "-o", out, "--detrend", "all", "--grid-mm", "1", "--no-plots",
                    "--seed", seed, "--bootstrap-samples", "200") == 0
        return [{k: v for k, v in r.items() if k not in ("timestamp", "run_dir")}
                for r in rows(out / "roughness_results.csv")]
    a, b = table(tmp_path / "a", 7), table(tmp_path / "b", 7)
    assert a == b
    c = table(tmp_path / "c", 8)
    assert [r["Sq_ci95_lo_mm"] for r in c] != [r["Sq_ci95_lo_mm"] for r in a]     # bootstrap follows the seed
    assert [r["Sq_mm"] for r in c] == [r["Sq_mm"] for r in a]                     # nothing else does


def test_config_file_and_flag_override(ripple_ply, tmp_path):
    cfg = tmp_path / "c.toml"
    cfg.write_text('output = "%s"\n[detrend]\ndetrend = "all"\n[grid]\ngrid_mm = 2.0\n'
                   % (tmp_path / "cfg_out").as_posix())
    assert main("--config", cfg, "--grid-mm", "1.5", "--no-plots", "--bootstrap-samples", "50", ripple_ply) == 0
    table = rows(tmp_path / "cfg_out" / "roughness_results.csv")
    assert len(table) == 4 and {r["grid_mm"] for r in table} == {"1.5"}
    bad = tmp_path / "bad.toml"
    bad.write_text("grid_mn = 1\n")
    assert main("--config", bad, ripple_ply) == 1


def test_noise_scan(ripple_ply, tmp_path):
    flat = tmp_path / "flat.ply"
    S.write_ply(flat, S.make_ripple_scan(n_points=60000, size=(0.24, 0.18), amplitude=0.0,
                                         small_sq=1e-12, bowl=0.0, noise=0.0005))
    out = tmp_path / "out"
    assert main(ripple_ply, "-o", out, "--grid-mm", "1", "--no-plots", "--noise-scan", flat,
                "--bootstrap-samples", "50") == 0
    r = rows(out / "roughness_results.csv")[0]
    sigma, sq = float(r["noise_sigma_map_mm"]), float(r["Sq_mm"])
    assert 0.1 < sigma < 0.5                       # 0.5 mm per point, averaged over ~1.4 points / cell
    assert float(r["Sq_noisecorr_mm"]) == pytest.approx(np.sqrt(sq ** 2 - sigma ** 2), rel=1e-3)


def test_validation_against_analytic_reference(tmp_path):
    """Egg-carton definition from ../Evaluate; a noiseless 'scan' sampled from
    it in its own frame must reproduce the reference metrics."""
    from scan_eval.reference import UnitScaled
    from scan_eval.surface_def import load_reference
    defn = ROOT.parent / "Evaluate" / "surfaces" / "egg_carton.toml"
    ref = UnitScaled(load_reference(defn))
    rng = np.random.default_rng(0)
    xy = rng.uniform(0.0, 0.1, (200000, 2))
    scan = tmp_path / "egg.ply"
    S.write_ply(scan, np.column_stack([xy, ref.height(xy[:, 0], xy[:, 1])]))
    out = tmp_path / "out"
    assert main(scan, "-o", out, "--reference-def", defn, "--sensor-origin", "none", "--grid-mm", "1",
                "--no-plots", "--bootstrap-samples", "50") == 0
    r = rows(out / "roughness_results.csv")[0]
    for k in ("Sq_mm", "Sdr_pct", "dominant_wavelength_mm"):
        assert abs(float(r[f"err_pct_{k}"])) < 1.0, (k, r[f"err_pct_{k}"])
    assert float(r["ref_dominant_wavelength_mm"]) == pytest.approx(50 / np.sqrt(2), rel=0.03)
    assert "ALIGN" not in r["flags"]


# --- failures -----------------------------------------------------------------
def test_empty_scan_fails(tmp_path, capsys):
    empty = tmp_path / "empty.ply"
    empty.write_text("ply\nformat ascii 1.0\nelement vertex 0\nproperty float x\nproperty float y\n"
                     "property float z\nend_header\n")
    assert main(empty, "-o", tmp_path / "o") == 1
    assert "no points" in err(capsys)


def test_grid_too_coarse_fails(ripple_ply, tmp_path, capsys):
    assert main(ripple_ply, "-o", tmp_path / "o", "--grid-mm", "20", "--no-plots") == 1
    text = err(capsys)
    assert "grid too coarse" in text and "--grid-mm" in text


def test_empty_roi_fails(ripple_ply, tmp_path, capsys):
    assert main(ripple_ply, "-o", tmp_path / "o", "--roi", "1000,1000,1100,1100") == 1
    assert "contains no points" in err(capsys)


@pytest.mark.parametrize("args, message", [
    (["--detrend", "sextic"], "unknown detrend"),
    (["--detrend", "planar", "--primary-detrend", "cubic"], "not one of"),
    (["--roi", "10,0,0,10"], "X0 < X1"),
    (["--noise-sigma-mm", "0.1", "--noise-scan", "x.ply"], "mutually exclusive"),
    (["--ruggedness-window", "4"], "odd"),
])
def test_bad_parameters_fail(ripple_ply, tmp_path, capsys, args, message):
    assert main(ripple_ply, "-o", tmp_path / "o", *args) == 1
    assert message in err(capsys)
