"""Unsafe / invalid input fails with a clear message; the CLI runs end to end."""

import csv
import json

import numpy as np
import open3d as o3d
import pytest
from conftest import SURFACES, synthetic_scan

import evaluate_scans
from scan_eval.expression import ExpressionError, parse_expression
from scan_eval.pipeline import ScanError, check_units, load_scan
from scan_eval.params import EvalParams
from scan_eval.reference import SurfaceError, UnitScaled
from scan_eval.surface_def import load_reference


@pytest.mark.parametrize("text, message", [
    ("__import__('os').system('echo pwned')", "only plain function calls"),
    ("x.__class__", "not allowed"),
    ("(lambda: 1)()", "only plain function calls"),
    ("'abc'", "numeric"),
    ("x^2", "use **"),
    ("[x, y]", "not allowed"),
    ("open('f')", "unknown function"),
    ("foo(x)", "unknown function"),
    ("A*x + y2", "unknown name 'y2'"),
    ("9**9**9", "unreasonably large"),
    ("sin(x, y)", "wrong number"),
    ("x <", "cannot parse"),
    ("x < 1", "only allowed in a domain mask"),
])
def test_unsafe_or_invalid_expressions_rejected(text, message):
    with pytest.raises(ExpressionError, match=message.replace("*", r"\*").replace("(", r"\(")):
        parse_expression(text, ("x", "y"), {"A": 1.0})


def test_valid_expression():
    e = parse_expression("c - A*sin(2*pi*x/L)*cos(2*pi*y/L) + hypot(x, y)**2", ("x", "y"),
                         {"A": 1, "L": 2, "c": 3})
    assert {str(s) for s in e.free_symbols} == {"x", "y", "A", "L", "c"}


def _write(tmp_path, name, text):
    p = tmp_path / name
    p.write_text(text)
    return p


def test_expression_undefined_over_domain(tmp_path):
    body = ('name="cap"\ntype="height_field"\nunits="mm"\n[params]\nR=50.0\n[surface]\n'
            'z="sqrt(R**2 - x**2 - y**2)"\n[domain]\nx=[-60,60]\ny=[-60,60]\n')
    with pytest.raises(SurfaceError, match="undefined"):
        load_reference(_write(tmp_path, "cap.toml", body))
    # The same surface with a mask that keeps it where it is defined is fine.
    ok = load_reference(_write(tmp_path, "cap2.toml", body + 'mask="x**2 + y**2 <= 45**2"\n'))
    assert ok.is_height_field


def test_definition_errors(tmp_path):
    with pytest.raises(SurfaceError, match="not found"):
        load_reference(tmp_path / "missing.stl")
    with pytest.raises(SurfaceError, match="unknown key"):
        load_reference(_write(tmp_path, "a.toml", 'type="height_field"\nunits="mm"\nzz=1\n'))
    with pytest.raises(SurfaceError, match="units"):
        load_reference(_write(tmp_path, "b.toml", 'type="height_field"\n[surface]\nz="x"\n'
                                                  '[domain]\nx=[0,1]\ny=[0,1]\n'))
    with pytest.raises(SurfaceError, match="TOML"):
        load_reference(_write(tmp_path, "c.yaml", "z: x"))
    with pytest.raises(SurfaceError, match="must be a number"):
        load_reference(_write(tmp_path, "d.toml", 'type="height_field"\nunits="mm"\n[params]\n'
                                                  'A="import os"\n[surface]\nz="A*x"\n'
                                                  '[domain]\nx=[0,1]\ny=[0,1]\n'))
    # A point cloud is not a reference surface.
    cloud = tmp_path / "cloud.ply"
    o3d.io.write_point_cloud(str(cloud), o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(np.random.default_rng(0).random((100, 3)))))
    with pytest.raises(SurfaceError, match="no faces"):
        load_reference(cloud)


def test_non_watertight_mesh_warns(tmp_path, capsys):
    from scan_eval.export import export_reference
    path = tmp_path / "open.stl"
    export_reference(load_reference(SURFACES / "paraboloid.toml"), path, resolution=2.0)
    load_reference(path)
    assert "not watertight" in capsys.readouterr().err


def test_scan_errors(tmp_path):
    with pytest.raises(ScanError, match="not found"):
        load_scan(tmp_path / "nope.ply", "m", 1.0)
    empty = tmp_path / "empty.ply"
    empty.write_text("ply\nformat ascii 1.0\nelement vertex 0\nproperty float x\n"
                     "property float y\nproperty float z\nend_header\n")
    with pytest.raises(ScanError, match="no points"):
        load_scan(empty, "m", 1.0)


def test_units_mismatch_is_loud(egg, capsys):
    scan, _ = synthetic_scan(egg, n=2000)
    params = EvalParams(units="m")
    assert check_units(egg, scan * 1000, params) is not None   # mm read as m
    assert "mm vs m" in capsys.readouterr().err
    assert check_units(egg, scan, params) is None


def test_export_solid_is_watertight(tmp_path):
    from scan_eval.export import export_reference
    ref = load_reference(SURFACES / "egg_carton.toml")
    mesh = export_reference(ref, tmp_path / "egg.stl", resolution=1.0, base_z=0.0)
    from scan_eval.mesh_reference import is_closed, signed_volume
    assert is_closed(np.asarray(mesh.triangles))
    # Outward-facing and the right size: the mean height above z=0 is c = 25 mm.
    volume = signed_volume(np.asarray(mesh.vertices), np.asarray(mesh.triangles))
    assert volume == pytest.approx(100 * 100 * 25, rel=0.002)


def test_definition_hash_is_stable_and_specific(tmp_path):
    a = load_reference(SURFACES / "egg_carton.toml")
    text = (SURFACES / "egg_carton.toml").read_text()
    spaced = load_reference(_write(tmp_path, "s.toml", text.replace(
        '"c - A*sin(2*pi*x/L)*cos(2*pi*y/L)"', '"c-A * sin(2*pi*x/L) * cos(2*pi*y/L)"')))
    changed = load_reference(_write(tmp_path, "t.toml", text.replace("A = 15.0", "A = 15.5")))
    assert a.definition_hash == spaced.definition_hash
    assert a.definition_hash != changed.definition_hash


def test_cli_end_to_end(tmp_path, egg):
    scans = []
    for k in range(2):
        pts, _ = synthetic_scan(egg, n=20000, sigma=0.0002 * (k + 1), seed=30 + k)
        path = tmp_path / "scans" / f"scan{k}.ply"
        path.parent.mkdir(exist_ok=True)
        o3d.io.write_point_cloud(str(path), o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts)))
        scans.append(path)
    out = tmp_path / "out"
    base = ["--reference-def", str(SURFACES / "egg_carton.toml"), "-o", str(out),
            "--reference-samples", "40000", "--label", "synthetic"]
    assert evaluate_scans.main(base + [str(tmp_path / "scans")]) == 0
    run = out / "synthetic__scan0"
    for name in ("metrics.json", "transform.txt", "aligned_error.ply", "error_histogram.png",
                 "error_map.png", "run.log"):
        assert (run / name).is_file(), name
    doc = json.loads((run / "metrics.json").read_text())
    assert doc["metrics"]["acc_rmse_mm"] == pytest.approx(0.2, rel=0.1)
    assert doc["reference"]["definition_hash"] == egg.definition_hash

    # Re-run one scan seeded from its saved transform; rows accumulate.
    assert evaluate_scans.main(base + ["--align", "none", "--init-transform",
                                       str(run / "transform.txt"), str(scans[0])]) == 0
    rows = list(csv.DictReader(open(out / "results.csv")))
    assert len(rows) == 3
    assert {r["reference_hash"] for r in rows} == {egg.definition_hash[:16]}
    assert {r["reference_type"] for r in rows} == {"analytic"}
    assert float(rows[2]["acc_rmse_mm"]) == pytest.approx(float(rows[0]["acc_rmse_mm"]), abs=1e-4)

    assert evaluate_scans.main(base + [str(tmp_path / "missing.ply")]) == 1
