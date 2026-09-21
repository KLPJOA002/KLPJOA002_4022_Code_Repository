"""Shared fixtures: references built from the example definitions, and a
synthetic scanner that samples a reference, adds noise and puts the result in
a random camera frame (camera above the part, as on the rig)."""

import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scan_eval.params import EvalParams  # noqa: E402
from scan_eval.reference import UnitScaled  # noqa: E402
from scan_eval.surface_def import load_reference  # noqa: E402

SURFACES = ROOT / "surfaces"


def fast_params(**overrides) -> EvalParams:
    p = EvalParams(reference_samples=60000, write_plots=False, write_ply=False)
    for k, v in overrides.items():
        setattr(p, k, v)
    return p


def synthetic_scan(ref_m, n=40000, sigma=0.0003, seed=0, shift=None, keep=None,
                   max_tilt_deg=25.0, scale=1.0):
    """Returns (scan points in their camera frame, T_true scan->reference).

    ref_m is a metre-unit reference. Points are area-uniform samples of the
    visible surface plus isotropic Gaussian noise of `sigma` metres; `shift`
    moves them in the reference frame first (e.g. by one period); `keep`
    optionally selects a sub-region (callable on reference-frame points)."""
    rng = np.random.default_rng(seed)
    pts, nrm = ref_m.sample(int(n * 1.6), seed + 100)
    vis = ref_m.visible_mask(pts, nrm)
    pts = pts[vis]
    if keep is not None:
        pts = pts[keep(pts)]
    pts = pts[:n] + rng.normal(0, sigma, (min(n, len(pts)), 3))
    if shift is not None:
        pts = pts + np.asarray(shift)
    pts = pts * scale
    # Camera 0.6 m above the part's centre looking down, randomly tilted and
    # spun about its optical axis - the frame a RealSense capture is in.
    lo, hi = ref_m.bounds()
    centre = 0.5 * (lo + hi)
    up = ref_m.visible_from if ref_m.visible_from is not None else np.array([0, 0, 1.0])
    camera = centre + 0.6 * up
    tilt = Rotation.from_rotvec(np.radians(rng.uniform(-max_tilt_deg, max_tilt_deg)) *
                                np.array([1.0, 0, 0]))
    spin = Rotation.from_euler("z", rng.uniform(0, 360), degrees=True)
    R_cam = (spin * tilt).as_matrix()            # reference -> camera rotation
    T = np.eye(4)                                 # reference -> scan
    T[:3, :3] = R_cam
    T[:3, 3] = -R_cam @ camera
    scan = pts @ T[:3, :3].T + T[:3, 3]
    return scan, np.linalg.inv(T)


def pose_error(T_est, T_true, points):
    """Max displacement (m) between the two poses over `points`, and rotation (deg)."""
    a = points @ T_est[:3, :3].T + T_est[:3, 3]
    b = points @ T_true[:3, :3].T + T_true[:3, 3]
    R = T_est[:3, :3] @ T_true[:3, :3].T
    R = R / np.cbrt(np.linalg.det(R))
    angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    return float(np.max(np.linalg.norm(a - b, axis=1))), float(angle)


@pytest.fixture(scope="session")
def egg():
    return UnitScaled(load_reference(SURFACES / "egg_carton.toml"))


@pytest.fixture(scope="session")
def egg_mesh(tmp_path_factory):
    """MeshReference of the egg-carton exported from its analytic definition."""
    from scan_eval.export import export_reference
    from scan_eval.mesh_reference import MeshReference
    raw = load_reference(SURFACES / "egg_carton.toml")
    raw.export_options = {}                  # the surface alone, not the printable solid
    path = tmp_path_factory.mktemp("mesh") / "egg_05.ply"
    export_reference(raw, path, resolution=0.5)
    return UnitScaled(MeshReference(path, units="mm", height_field=True))
