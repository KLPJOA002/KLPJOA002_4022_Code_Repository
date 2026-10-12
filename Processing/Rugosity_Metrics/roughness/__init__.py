"""roughness - surface-roughness metrics from fused 3D scans of a sediment bed.

The sibling of ../Evaluate: it reuses scan_eval (scan loading, console log,
config files, results table) by putting ../Evaluate on sys.path, the same way
Parameter_Sweep and Filter_Sweep do. Nothing in Evaluate depends on this.
"""

from __future__ import annotations

import sys
from pathlib import Path

EVALUATE_DIR = Path(__file__).resolve().parents[2] / "Evaluate"
if not (EVALUATE_DIR / "scan_eval").is_dir():
    raise ImportError(f"roughness needs ../Evaluate (scan_eval) next to it; not found at {EVALUATE_DIR}")
if str(EVALUATE_DIR) not in sys.path:
    sys.path.insert(0, str(EVALUATE_DIR))
