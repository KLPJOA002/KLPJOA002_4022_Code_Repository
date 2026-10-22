"""Optional validation against a known reference (--reference): a mesh
(.stl / .ply / .obj, taken as the height field z = h(x, y) seen from above)
or an analytic height-field definition (.toml, as in ../Evaluate/surfaces).

The scan must already be aligned into the reference's frame (e.g. Evaluate's
aligned_error.ply). The reference is sampled on a dense lattice over the
scan's footprint, pushed through the scan's own levelling transform,
restricted to the scan's valid cells, and then run through exactly the same
detrend, grid and metric code. Where the reference does not cover every valid
scan cell, the scan's metrics are recomputed on the shared cells, so each
error (scan minus reference) always compares the same area with the same
processing, and the error budget of the whole capture + fusion + roughness
chain is measured directly.
"""

from __future__ import annotations

import numpy as np
from scipy import ndimage

from scan_eval.log import log
from scan_eval.reference import UnitScaled

from .detrend import fit_trend
from .grid import apply_fill, cell_counts, cell_means, plan_fill
from .metrics import surface_metrics
from .params import DETREND_ORDERS

# Metrics compared (scalars that mean the same thing on both surfaces).
KEYS = ["Sq_mm", "Sa_mm", "Ssk", "Sku", "Sp_mm", "Sv_mm", "Sz_mm", "rugosity", "Sdr_pct", "Sdq",
        "slope_mean_deg", "Sal_mm", "Str", "Std_deg", "dominant_wavelength_mm",
        "dominant_direction_deg", "anisotropy_ratio", "hurst", "fractal_dimension", "TRI_mm", "VRM"]


def load_height_reference(params):
    from scan_eval.surface_def import load_reference
    # height_field=True: a bare mesh is read as the surface seen from above
    # (rays cast down onto it), which is what a height map of a scan is.
    ref = load_reference(params.reference_def, units_override=params.reference_units, height_field=True)
    if not ref.is_height_field:
        raise ValueError(f"{params.reference_def}: validation needs a height-field reference "
                         f"(z = h(x, y)); this one is {ref.kind}")
    restore_removed_faces(ref)
    ref_m = UnitScaled(ref)
    log(f"[reference] {ref.name}: height field, units {ref.units}, hash {ref.definition_hash[:16]}")
    return ref_m


def restore_removed_faces(ref) -> None:
    """Undo scan_eval's non-manifold repair on a mesh reference.

    A part modelled as separate shells (e.g. a base slab with pyramids
    stacked on it, not merged) has edges shared by more than two faces.
    scan_eval's MeshReference drops faces on those edges, and Open3D can
    pick real top faces: PYRAMID_2X2.stl loses 4 of its pyramid faces, and
    rays then fall through two diamond-shaped holes to the base. Seen from
    above only the topmost surface matters, so the raw triangles are what a
    height field needs. Orientation does not matter here either: heights
    come from rays, and ICP uses the normals only up to sign."""
    import open3d as o3d
    from scan_eval.mesh_reference import MeshReference, _face_normals
    from scan_eval.log import warn
    if not isinstance(ref, MeshReference) or not ref.topology.get("removed_non_manifold_faces"):
        return
    mesh = o3d.io.read_triangle_mesh(str(ref.path))
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    restored = len(mesh.triangles) - len(ref.triangles)
    ref.mesh = mesh
    ref.vertices = np.asarray(mesh.vertices, dtype=float)
    ref.triangles = np.asarray(mesh.triangles, dtype=np.int64)
    ref.face_normals = _face_normals(ref.vertices, ref.triangles)
    tri = ref.vertices[ref.triangles]
    ref.face_areas = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    tmesh = o3d.t.geometry.TriangleMesh()
    tmesh.vertex.positions = o3d.core.Tensor((ref.vertices - ref._centre).astype(np.float32))
    tmesh.triangle.indices = o3d.core.Tensor(ref.triangles.astype(np.int32))
    ref._scene = o3d.t.geometry.RaycastingScene()
    ref._scene.add_triangles(tmesh)
    ref.topology["restored_faces_for_height_field"] = int(restored)
    warn(f"reference {ref.path.name}: put back the {restored} face(s) the non-manifold repair removed - "
         "they are real surface (separate shells stacked, e.g. features on a base plate), and the "
         "top-down height field needs them. NB Evaluate itself still drops them, which leaves holes "
         "in its reference for this part")


def compare_to_reference(ref, prep, valid, params, methods, results, scan_raw) -> dict:
    """`scan_raw` is the scan's cell-mean height map before detrending, so the
    scan side can be re-detrended and re-evaluated on the shared cells."""
    frame, geom, g = prep.frame, prep.geom, prep.geom.g
    warnings = []
    # Scan footprint back in the reference (= original scan) frame.
    scan_ref = prep.points @ frame.R + frame.centroid
    lo, hi = scan_ref[:, :2].min(axis=0), scan_ref[:, :2].max(axis=0)
    rlo, rhi = ref.bounds()
    lo, hi = np.maximum(lo, rlo[:2]), np.minimum(hi, rhi[:2])
    if np.any(hi <= lo):
        raise ValueError("the scan does not overlap the reference in xy - is it aligned into the "
                         "reference frame (e.g. Evaluate's aligned output)?")

    # Is the scan really in the reference frame? Heights should agree.
    sub = scan_ref[:: max(1, len(scan_ref) // 50000)]
    href = ref.height(sub[:, 0], sub[:, 1])
    ok = np.isfinite(href)
    offset = float(np.median(np.abs(sub[ok, 2] - href[ok]))) if ok.any() else float("inf")

    # Dense lattice (~2 samples per cell each way) on the reference surface.
    step = g / 2
    xs = np.arange(lo[0], hi[0] + step / 2, step)
    ys = np.arange(lo[1], hi[1] + step / 2, step)
    X, Y = np.meshgrid(xs, ys)
    Z = ref.height(X.ravel(), Y.ravel())
    keep = np.isfinite(Z)
    pts = frame.apply(np.column_stack([X.ravel()[keep], Y.ravel()[keep], Z[keep]]))
    idx = geom.index(pts[:, :2])
    on = idx >= 0
    on[on] = valid.ravel()[idx[on]]
    pts, idx = pts[on], idx[on]
    if len(pts) < 100:
        raise ValueError("the reference covers almost none of the scan's valid cells")

    counts = cell_counts(idx, geom)
    plan = plan_fill(counts >= 1, params.max_hole_cells)
    raw = apply_fill(cell_means(idx, pts[:, 2], counts, geom), plan)
    cx, cy = geom.centres()
    CX, CY = np.meshgrid(cx, cy)
    ref_results, scan_results, columns, ref_h = {}, {}, {}, {}
    for m in methods:
        fit = fit_trend(pts, m, params.detrend_clip_sigma, params.detrend_iterations)
        r = pts[:, 2] - fit.evaluate(pts[:, 0], pts[:, 1])
        h = apply_fill(cell_means(idx, r, counts, geom), plan)
        mask = valid & np.isfinite(h)
        if mask.sum() < valid.sum():
            # The scan reaches past the reference: its cells on the reference's
            # edge mix part and surroundings (e.g. a step down to the table),
            # which the reference never sees. Leave that one-cell rim out.
            mask &= ndimage.binary_erosion(mask, structure=np.ones((3, 3), bool))
        rng = np.random.default_rng([params.seed, DETREND_ORDERS[m]])
        res, _, aux, _ = surface_metrics(np.where(mask, h, np.nan), mask, g, params, rng)
        ref_results[m], ref_h[m] = res, aux["h"]
        if mask.sum() < valid.sum():
            # Same cells and same processing on both sides: the scan's trend is
            # refitted on the overlap (its own fit also saw the cells outside),
            # then its metrics are computed over the overlap only.
            cells = np.column_stack([CX[mask], CY[mask], scan_raw[mask]])
            sfit = fit_trend(cells, m, params.detrend_clip_sigma, params.detrend_iterations)
            hs = np.where(mask, scan_raw - sfit.evaluate(CX, CY), np.nan)
            rng = np.random.default_rng([params.seed, DETREND_ORDERS[m]])
            scan_results[m], _, _, _ = surface_metrics(hs, mask, g, params, rng)
        else:
            scan_results[m] = results[m]
        cols = {}
        for k in KEYS:
            s, t = scan_results[m].get(k), res.get(k)
            if s is None or t is None or not (np.isfinite(s) and np.isfinite(t)):
                continue
            cols[f"ref_{k}"] = t
            cols[f"err_{k}"] = s - t
            cols[f"err_pct_{k}"] = 100 * (s - t) / abs(t) if t != 0 else float("nan")
        columns[m] = cols
    coverage = float(mask.sum() / max(valid.sum(), 1))
    log(f"[validate] {len(pts)} reference samples over {coverage:.1%} of the scan's valid cells; "
        f"median |scan z - reference z| {offset * 1e3:.3f} mm")
    pm = params.primary
    c = columns[pm]
    for k in ("Sq_mm", "Sa_mm", "Sdr_pct", "Sdq", "Sal_mm", "dominant_wavelength_mm", "hurst"):
        if f"ref_{k}" in c:
            log(f"  {pm:9s} {k:24s} scan {scan_results[pm][k]:10.4f}  reference {c[f'ref_{k}']:10.4f}  "
                f"error {c[f'err_{k}']:+10.4f} ({c[f'err_pct_{k}']:+.1f}%)")
    up = ref.visible_from if ref.visible_from is not None else np.array([0, 0, 1.0])
    if frame.R[2] @ up < 0:
        warnings.append(("UPSIDEDOWN", "the levelled +Z points away from the reference's visible side, "
                         "so heights are inverted (Ssk, Sp/Sv flip). For scans aligned into the "
                         "reference frame pass --sensor-origin none"))
    if coverage < 0.95:
        warnings.append(("REFCOVER", f"the reference covers only {coverage:.0%} of the scan's valid "
                         "cells; the err_ columns compare the scan and the reference on that overlap "
                         "(the scan's own columns are still the whole evaluated area)"))
    # Misalignment (ALIGN / UPSIDEDOWN / ALIGNCOVER) is checked, with the fix
    # named, by the align stage before anything is computed.
    overlap = {m: scan_results[m] for m in methods if scan_results[m] is not results[m]}
    return {"reference": ref.describe(), "coverage": coverage, "samples": int(len(pts)),
            "height_offset_median_mm": offset * 1e3, "metrics": ref_results,
            "scan_metrics_on_overlap": overlap or None, "columns": columns, "warnings": warnings,
            "_aux": {"h": ref_h, "raw": np.where(mask, raw, np.nan), "mask": mask}}
