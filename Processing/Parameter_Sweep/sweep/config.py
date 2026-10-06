"""Paths, object -> reference mapping, and the fixed settings of the sweep
evaluation. Everything a re-run must agree on to reuse a cached result is
collected in eval_settings(), which feeds the cache fingerprint."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
PROCESSING = HERE.parent
EVALUATE_DIR = PROCESSING / "Evaluate"
CROP_DIR = PROCESSING / "Crop"

DEFAULT_DATA = Path("/run/media/joab/Seagate/roughness/Joab_Characterisation/Parameter_Error")
DEFAULT_OUT = HERE / "results"

# Bump when anything in the crop / seed / evaluation chain changes in a way
# that alters results, so cached rows are recomputed.
PIPELINE_VERSION = 1


@dataclass(frozen=True)
class ObjectSpec:
    folder: str        # top-level folder under the data directory
    label: str         # name used in plots and tables
    reference: Path    # ground-truth mesh, mm, height field z = h(x, y)


OBJECTS = {
    "Big_Pyramid": ObjectSpec("Big_Pyramid", "Pyramid", EVALUATE_DIR / "References" / "PYRAMID_1.stl"),
    "Big_Wave": ObjectSpec("Big_Wave", "Wave", EVALUATE_DIR / "References" / "EGG_1PK.stl"),
}

# --- Crop (../Crop/crop_pointclouds.py, automatic AprilTag crop) -------------
# Pixels the crop quad is pulled in from the tags' inner corners.
CROP_MARGIN_PX = 10.0
# Detections up to this many frames from the exported one fill in missing tags.
CROP_FRAME_WINDOW = 15

# --- Seed pose ----------------------------------------------------------------
# Even cropped, the part is only a few % of the cloud and Evaluate's moments
# start locks onto the table. The seed puts the table plane at z = 0 and the
# raised object's footprint over the reference's centre; Evaluate's full
# multi-start search (turns, flips, slides, RANSAC) still runs around it.
SEED_MIN_HEIGHT_M = 0.005
SEED_MAX_HEIGHT_M = 0.12
SEED_CLUSTER_EPS_M = 0.006

# --- Registration sanity check -------------------------------------------------
# A pose slid off the part covers too little of the reference, even when its
# RMSE looks good (a slid pose can drop the hardest points). Measured on this
# data: misregistered wave scans (no seed) covered 69-81%; correctly aligned
# scans cover >= 90% (low-res ones with holes included), so 85% splits them. The chosen pose's
# reference overlap (`align_overlap_ref`) is kept in the table but not used:
# it also drops with noise and holes (63-77% on correctly aligned 640x360
# wave scans), so it can't tell a bad pose from bad data.
MIN_COVERED_FRACTION = 0.85

# --- Evaluate settings (scan_eval.params.EvalParams overrides) ---------------
EVAL_OVERRIDES = dict(
    units="m",               # rs_marker PLYs are metres
    reference_units="mm",    # the STLs are mm
    seed=0,
    align="full",
    write_ply=False,         # skip the big per-scan PLYs; plots + metrics.json kept
    write_overlay=False,
    write_plots=True,
)
HEIGHT_FIELD = True

# Main metric for rankings and heatmaps (a results column).
PRIMARY_METRIC = "acc_rmse_mm"


def eval_settings() -> dict:
    return {
        "pipeline_version": PIPELINE_VERSION,
        "crop_margin_px": CROP_MARGIN_PX,
        "crop_frame_window": CROP_FRAME_WINDOW,
        "seed": [SEED_MIN_HEIGHT_M, SEED_MAX_HEIGHT_M, SEED_CLUSTER_EPS_M],
        "eval": EVAL_OVERRIDES,
        "height_field": HEIGHT_FIELD,
    }
