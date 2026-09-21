"""A mesh exported from an analytic definition must give (nearly) the same
answers as the analytic definition itself; the remaining difference is the
mesh's discretisation error, which is quantified here and must shrink as
O(h^2) with the vertex spacing h (chord error of a smooth surface)."""

import numpy as np
import pytest
from conftest import SURFACES, fast_params, synthetic_scan

from scan_eval.export import export_reference
from scan_eval.mesh_reference import MeshReference
from scan_eval.metrics import compute_metrics
from scan_eval.reference import UnitScaled
from scan_eval.surface_def import load_reference


def _mesh_at(tmp_path, resolution):
    raw = load_reference(SURFACES / "egg_carton.toml")
    raw.export_options = {}                  # the surface alone, not the printable solid
    path = tmp_path / f"egg_{resolution}.ply"
    export_reference(raw, path, resolution=resolution)
    return UnitScaled(MeshReference(path, units="mm", height_field=True))


def test_discretisation_error_and_metric_agreement(tmp_path, egg, capsys):
    rng = np.random.default_rng(0)
    pts, nrm = egg.sample(50000, 42)
    probe = pts + rng.normal(0, 0.002, (len(pts), 1)) * nrm   # within +-6 mm of the surface

    # Unsigned: just past the open rim the sign is ill-defined for both
    # representations (the offset is nearly tangent there), which would
    # swamp the discretisation error being measured.
    exact = egg.distance(probe)
    errors = {}
    meshes = {}
    for h in (2.0, 1.0, 0.5):
        meshes[h] = _mesh_at(tmp_path, h)
        errors[h] = np.abs(meshes[h].distance(probe) - exact)
    # Chord error of a smooth surface ~ curvature * h^2 / 8: halving h
    # should cut the error about 4x.
    for coarse, fine in ((2.0, 1.0), (1.0, 0.5)):
        ratio = errors[coarse].max() / errors[fine].max()
        assert 3.0 < ratio < 5.0, f"max error ratio {ratio:.2f} for h {coarse}->{fine}"
    # Bound: max curvature of the egg-carton is A (2 pi / L)^2 = 0.237 /mm.
    kappa = 15 * (2 * np.pi / 50) ** 2
    for h, e in errors.items():
        assert e.max() * 1e3 <= kappa * h**2 / 8 * 1.5 + 1e-4

    # Same noisy scan, already aligned: every metric agrees to well under the
    # discretisation error at h = 0.5 mm.
    scan, T = synthetic_scan(egg, sigma=0.0003, seed=21)
    aligned = scan @ T[:3, :3].T + T[:3, 3]
    params = fast_params()
    m_exact, _ = compute_metrics(egg, aligned, params, 0)
    m_mesh, _ = compute_metrics(meshes[0.5], aligned, params, 0)
    for key in ("acc_rmse_mm", "acc_mae_mm", "signed_bias_mm", "signed_std_mm", "z_rmse_mm",
                "completeness@1mm", "fscore@1mm"):
        assert m_mesh[key] == pytest.approx(m_exact[key], abs=0.003), key

    with capsys.disabled():
        print("\n  mesh vs analytic egg-carton, discretisation error (|d_mesh - d_exact|, um):")
        for h, e in errors.items():
            print(f"    h = {h:4.2f} mm   mean {e.mean() * 1e6:7.2f}   p99 {np.percentile(e, 99) * 1e6:7.2f}"
                  f"   max {e.max() * 1e6:7.2f}")
        print(f"    RMSE analytic {m_exact['acc_rmse_mm']:.4f} mm, mesh(0.5) {m_mesh['acc_rmse_mm']:.4f} mm")
