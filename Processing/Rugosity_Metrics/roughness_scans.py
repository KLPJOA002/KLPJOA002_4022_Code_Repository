#!/usr/bin/env python3
"""roughness_scans - surface-roughness metrics of fused 3D scans of a sediment bed.

Usage:
  roughness_scans.py scan.ply ...
  roughness_scans.py --detrend all --grid-mm 1 --edge-trim-mm 10 Fused_Scans/
  roughness_scans.py --config roughness_config.example.toml --noise-scan flat.ply scans/
  roughness_scans.py --reference part.stl --pick-roi --detrend all aligned_error.ply

The sibling of ../Evaluate/evaluate_scans.py: where that measures a scan's
error against a known part, this measures the roughness of the surface
itself, with no reference needed. Scans are in an arbitrary camera / fusion
frame, so each is first levelled onto its best-fit plane.

Stages per scan (printed like Evaluate's):

  load -> [align] -> orient -> crop/ROI -> detrend -> grid -> metrics -> [validate] -> write

Every scan gets its own folder under -o with metrics.json, height maps,
spectra, plots and a log; one row per (scan, detrend method) is appended to
<out>/roughness_results.csv so runs tabulate side by side. A TOML --config
sets baseline values; flags on the command line win. All distances on the
command line are millimetres.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import roughness  # noqa: E402,F401  (puts ../Evaluate on sys.path)
from evaluate_scans import UNIT_CHOICES, find_scans  # noqa: E402
from scan_eval import log as L  # noqa: E402
from scan_eval.log import log, warn  # noqa: E402
from scan_eval.params import load_config  # noqa: E402
from scan_eval.reference import SurfaceError  # noqa: E402

from roughness.params import (DETREND_ORDERS, RoughnessParams, param_names, parse_detrend,  # noqa: E402
                              parse_polygon)


def csv_numbers(count):
    def parse(text):
        try:
            values = [float(t) for t in str(text).split(",") if t.strip()]
        except ValueError:
            raise argparse.ArgumentTypeError(f"expected comma-separated numbers, got '{text}'") from None
        if len(values) != count:
            raise argparse.ArgumentTypeError(f"expected {count} comma-separated numbers, got '{text}'")
        return values
    return parse


def build_parser():
    p = argparse.ArgumentParser(
        prog="roughness_scans.py",
        description="Surface-roughness metrics (ISO 25178 height / area / spatial, spectral, "
                    "multi-scale) of fused 3D scans (.ply), per detrend method.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""examples:
  # one scan, planar detrend, automatic grid
  roughness_scans.py --label d435_close scan.ply

  # every detrend order, trim 10 mm of ragged border, 1 mm grid, a whole folder
  roughness_scans.py --detrend all --primary-detrend quadratic --edge-trim-mm 10 --grid-mm 1 Fused_Scans/

  # crop to a region (mm, in the levelled frame - read it off heightmap_*.png)
  roughness_scans.py --roi -150,-100,150,100 scan.ply

  # draw the area by hand (rectangle, or p for a polygon); saved to <run>/roi.toml
  roughness_scans.py --pick-roi --detrend all scan.ply

  # compare with the printed part: first align with Evaluate, then
  roughness_scans.py --reference ../Evaluate/References/PYRAMID_2X2.stl --detrend all \
      ../Evaluate/evaluation_out/<run>/aligned_error.ply

  # the scan doesn't line up with the part: move it by hand (keys), or by flags
  roughness_scans.py --reference part.stl --adjust aligned_error.ply
  roughness_scans.py --reference part.stl --flip --rotate-deg 90 --shift-mm 2,0,0 --refine aligned_error.ply
  # ... and rerun exactly from the saved pose
  roughness_scans.py --reference part.stl --transform roughness_out/<run>/transform.txt aligned_error.ply

  # subtract the sensor's noise floor, measured on a flat plate
  roughness_scans.py --noise-scan flat_plate.ply scans/
""")
    p.add_argument("scans", nargs="*", help=".ply files and/or directories (searched recursively)")
    p.add_argument("-o", "--output", help="output directory (default roughness_out)")
    p.add_argument("--label", help="label for this run in roughness_results.csv")
    p.add_argument("--config", metavar="TOML", help="baseline parameters; flags override it "
                                                   "(see roughness_config.example.toml)")
    p.add_argument("--seed", type=int, help="random seed (default 0); fixes every random choice")
    p.add_argument("--list-only", action="store_true", help="print the scans found and exit")
    p.add_argument("-v", "--verbose", action="store_true", help="more detail on the console")

    u = p.add_argument_group("units and orientation")
    u.add_argument("--units", choices=UNIT_CHOICES, help="units of the scan PLYs (default m)")
    u.add_argument("--scale", type=float, help="extra factor applied to the scans (default 1)")
    u.add_argument("--orient", choices=["plane", "none"], dest="orient_mode",
                   help="plane: level the scan onto its best-fit plane; none: keep the input frame "
                        "(scans aligned to a reference). Default: none with --reference, else plane")
    u.add_argument("--sensor-origin", metavar="X,Y,Z",
                   help="sensor position in the scan's frame, mm (default 0,0,0 - true for RealSense "
                        "and Fuse output): +Z is turned to face it. 'none' = input +Z side up")
    u.add_argument("--inplane-axis", choices=["input", "pca"],
                   help="levelled x axis: the scan's x projected onto the plane (default) or the "
                        "footprint's long axis")

    c = p.add_argument_group("crop and grid (mm)")
    c.add_argument("--roi", type=csv_numbers(4), dest="roi_mm", metavar="X0,Y0,X1,Y1",
                   help="region of interest in the analysis frame, mm (default: whole scan)")
    c.add_argument("--roi-polygon", dest="roi_polygon_mm", metavar="X,Y;X,Y;...",
                   help="polygon region of interest in the analysis frame, mm (quote it)")
    c.add_argument("--pick-roi", action="store_true", default=None,
                   help="draw the region by hand on a top-down view of each scan (needs a display "
                        "and PyQt6 or tkinter); saved to <run>/roi.toml for exact re-runs")
    c.add_argument("--edge-trim-mm", type=float, metavar="MM",
                   help="drop points within this distance of the scan's outer border (default 0)")
    c.add_argument("--grid-mm", type=float, metavar="MM",
                   help="height-map cell size (default 0 = auto, 2 x median point spacing)")
    c.add_argument("--max-hole-cells", type=int, metavar="N",
                   help="interpolate interior holes up to N cells across (default 2); larger stay masked")
    c.add_argument("--min-points-per-cell", type=int, metavar="N",
                   help="points a cell needs to count as measured (default 1)")

    d = p.add_argument_group("detrending")
    d.add_argument("--detrend", metavar="METHODS",
                   help=f"{', '.join(DETREND_ORDERS)}, a comma list, or all (default planar)")
    d.add_argument("--primary-detrend", choices=list(DETREND_ORDERS),
                   help="method for the main plots and the 'primary' column (default: first listed)")
    d.add_argument("--clip-sigma", type=float, dest="detrend_clip_sigma",
                   help="robust fit: reject points beyond this many sigma (default 3)")
    d.add_argument("--clip-iterations", type=int, dest="detrend_iterations",
                   help="robust fit: sigma-clipping iterations (default 5)")
    d.add_argument("--expected-wavelength-mm", type=float, metavar="MM",
                   help="expected ripple wavelength for the over-fitting check (default: measured)")

    s = p.add_argument_group("metrics")
    s.add_argument("--acf-threshold", type=float, help="autocorrelation threshold s for Sal/Str (default 0.2)")
    s.add_argument("--psd-band-mm", type=csv_numbers(2), metavar="MIN,MAX",
                   help="wavelength band of the PSD power-law fit, mm (default 4 x grid, footprint/4; "
                        "0 = that end's default)")
    s.add_argument("--multiscale-sizes", type=int, metavar="N",
                   help="number of window sizes for the multi-scale curves (default 8)")
    s.add_argument("--window-detrend", choices=["plane", "mean"],
                   help="per-window detrend for the multi-scale curves (default plane)")
    s.add_argument("--bootstrap-samples", type=int, metavar="N", help="bootstrap resamples (default 1000)")
    s.add_argument("--bootstrap-window-mm", type=float, metavar="MM",
                   help="window size the bootstrap resamples (default auto)")
    s.add_argument("--ruggedness-window", type=int, metavar="CELLS",
                   help="TRI / VRM window, odd number of cells (default 3; 0 = off)")
    s.add_argument("--footprint-wavelengths-min", type=float, metavar="N",
                   help="warn if the footprint is fewer than N dominant wavelengths (default 5)")

    n = p.add_argument_group("sensor noise (one of)")
    n.add_argument("--noise-sigma-mm", type=float, metavar="MM",
                   help="known per-point sensor noise, mm (averaged down per cell before subtracting)")
    n.add_argument("--noise-scan", metavar="PLY",
                   help="scan of a flat surface, same setup: its detrended Sq / Sdq is the noise floor")

    v = p.add_argument_group("comparison with a known reference (optional)")
    v.add_argument("--reference", "--reference-def", dest="reference_def", metavar="STL|TOML",
                   help="the reference object: a mesh (.stl/.ply/.obj, read as the surface seen from "
                        "above) or a height-field definition (../Evaluate/surfaces/*.toml). The scans "
                        "must be aligned into its frame (Evaluate's aligned_error.ply). Adds ref_ / "
                        "err_ / err_pct_ columns and reference_comparison.png")
    v.add_argument("--reference-units", choices=UNIT_CHOICES,
                   help="units of the reference (meshes default to mm)")
    v.add_argument("--reference-crop", choices=["footprint", "none"],
                   help="footprint: evaluate only the scan over the reference's outline (default); "
                        "none: the whole scan")
    v.add_argument("--reference-inset-mm", type=float, metavar="MM",
                   help="shrink the reference outline by this much, e.g. to drop side walls (default 0)")

    al = p.add_argument_group("moving the scan onto the reference (applied in this order)")
    al.add_argument("--transform", metavar="FILE",
                    help="4x4 scan -> reference matrix (Evaluate's or a previous run's transform.txt); "
                         "lets you pass the raw scan instead of aligned_error.ply")
    al.add_argument("--flip", action="store_true", default=None,
                    help="turn the scan over (180 deg about x through the reference centre)")
    al.add_argument("--rotate-deg", type=float, metavar="DEG",
                    help="rotate about the vertical axis through the reference centre")
    al.add_argument("--shift-mm", type=csv_numbers(3), metavar="DX,DY,DZ", help="translate the scan, mm")
    al.add_argument("--refine", action="store_true", default=None,
                    help="then fit it to the reference with ICP (nearest fit: get close first on a "
                         "periodic part)")
    al.add_argument("--no-auto-place", action="store_false", dest="auto_place", default=None,
                    help="don't auto-place a scan that is nowhere near the part (by default a raw scan is "
                         "levelled on the bed and the part found and centred automatically)")
    al.add_argument("--align-corr-mm", type=float, metavar="MM",
                    help="ICP ignores scan points farther than this from the reference (default 5)")
    al.add_argument("--adjust", action="store_true", default=None,
                    help="move the scan by hand in a window with a live scan - reference difference map "
                         "(needs a display and PyQt6 or tkinter). The final pose is saved to "
                         "<run>/transform.txt")

    o = p.add_argument_group("outputs")
    o.add_argument("--no-plots", action="store_false", dest="write_plots", default=None,
                   help="don't write the PNG plots")
    o.add_argument("--profile-y-mm", type=float, metavar="MM",
                   help="y of the profile cut in the detrend / reference plots (default: middle)")
    return p


def resolve_params(args) -> tuple[RoughnessParams, dict]:
    """Defaults <- config file <- command line, as in evaluate_scans."""
    params = RoughnessParams()
    extra = {}
    if args.config:
        config = load_config(args.config)
        unknown = set(config) - param_names() - {"scans"}
        if unknown:
            raise ValueError(f"{args.config}: unknown setting(s) {sorted(unknown)}")
        for key, value in config.items():
            if key == "scans":
                extra[key] = value
            else:
                setattr(params, key, value)
    for key in param_names():
        value = getattr(args, key, None)
        if value is not None:
            setattr(params, key, value)
    validate_params(params)
    return params, extra


def validate_params(params):
    params.detrend = parse_detrend(params.detrend)
    if params.primary_detrend is not None:
        params.primary_detrend = str(params.primary_detrend).lower()
        if params.primary_detrend not in params.detrend:
            raise ValueError(f"--primary-detrend {params.primary_detrend} is not one of the "
                             f"--detrend methods {params.detrend}")
    if params.units not in UNIT_CHOICES:
        raise ValueError(f"units must be one of {UNIT_CHOICES}")
    if params.scale <= 0:
        raise ValueError("--scale must be positive")
    if params.grid_mm < 0 or params.edge_trim_mm < 0:
        raise ValueError("--grid-mm and --edge-trim-mm must not be negative")
    if params.roi_mm is not None:
        if len(params.roi_mm) != 4 or params.roi_mm[0] >= params.roi_mm[2] or params.roi_mm[1] >= params.roi_mm[3]:
            raise ValueError(f"--roi must be X0,Y0,X1,Y1 with X0 < X1 and Y0 < Y1 (got {params.roi_mm})")
    if params.roi_polygon_mm is not None:
        params.roi_polygon_mm = parse_polygon(params.roi_polygon_mm)
    if params.orient_mode not in (None, "plane", "none"):
        raise ValueError("orient_mode must be plane or none")
    if params.reference_crop not in ("footprint", "none"):
        raise ValueError("reference_crop must be footprint or none")
    if (params.refine or params.adjust) and not params.reference_def:
        raise ValueError("--refine and --adjust need a --reference to align to")
    if params.transform and not Path(params.transform).is_file():
        raise FileNotFoundError(f"transform file not found: {params.transform}")
    if params.shift_mm is not None and len(params.shift_mm) != 3:
        raise ValueError(f"shift_mm needs three numbers dx, dy, dz (got {params.shift_mm})")
    if params.align_corr_mm <= 0:
        raise ValueError("--align-corr-mm must be positive")
    if params.reference_inset_mm < 0:
        raise ValueError("--reference-inset-mm must not be negative")
    if params.psd_band_mm is not None:
        lo, hi = params.psd_band_mm
        if lo < 0 or hi < 0 or (lo and hi and lo >= hi):
            raise ValueError(f"--psd-band-mm must be MIN,MAX with 0 <= MIN < MAX (got {params.psd_band_mm})")
    if params.noise_scan and params.noise_sigma_mm:
        raise ValueError("--noise-sigma-mm and --noise-scan are mutually exclusive")
    if params.noise_scan and not Path(params.noise_scan).is_file():
        raise FileNotFoundError(f"noise scan not found: {params.noise_scan}")
    if params.noise_sigma_mm < 0:
        raise ValueError("--noise-sigma-mm must not be negative")
    if not 0 < params.acf_threshold < 1:
        raise ValueError("--acf-threshold must be between 0 and 1")
    if params.ruggedness_window and (params.ruggedness_window < 3 or params.ruggedness_window % 2 == 0):
        raise ValueError("--ruggedness-window must be an odd number of cells >= 3 (or 0 to disable)")
    if params.window_detrend not in ("plane", "mean"):
        raise ValueError("window_detrend must be plane or mean")
    if params.inplane_axis not in ("input", "pca"):
        raise ValueError("inplane_axis must be input or pca")
    if params.max_hole_cells < 0 or params.min_points_per_cell < 1:
        raise ValueError("--max-hole-cells must be >= 0 and --min-points-per-cell >= 1")
    if params.bootstrap_samples < 10:
        raise ValueError("--bootstrap-samples must be at least 10")


def print_summary(rows):
    if not rows:
        return
    log(f"\n=== Summary: {len(rows)} row(s) (scan x detrend method) ===")
    log(f"  {'label':20s} {'scan':24s} {'detrend':10s} {'Sq':>8s} {'Sa':>8s} {'Ssk':>6s} {'Sku':>6s} "
        f"{'Sdr%':>7s} {'Sdq':>6s} {'Sal':>7s} {'Str':>5s} {'lambda':>7s} {'H':>5s} {'valid':>6s}  flags")
    for r in rows:
        star = "*" if r["primary"] else " "
        log(f"  {str(r['label'])[:20]:20s} {Path(r['scan']).name[:24]:24s} {r['detrend'][:9]:9s}{star} "
            f"{r['Sq_mm']:8.4f} {r['Sa_mm']:8.4f} {r['Ssk']:+6.2f} {r['Sku']:6.2f} {r['Sdr_pct']:7.3f} "
            f"{r['Sdq']:6.3f} {r['Sal_mm']:7.2f} {r['Str']:5.2f} {r['dominant_wavelength_mm']:7.2f} "
            f"{r['hurst']:5.2f} {r['valid_fraction']:6.1%}  {r['flags']}")
    log("  (mm; * = primary detrend; lambda = dominant wavelength; H = Hurst exponent)")


# Options whose value is a coordinate list that may start with a minus sign
# ("--roi -150,-100,150,100"): argparse would take it for another option.
COORD_OPTIONS = {"--roi", "--roi-polygon", "--shift-mm", "--psd-band-mm"}


def join_negative_values(argv):
    out, it = [], iter(argv)
    for a in it:
        if a in COORD_OPTIONS:
            nxt = next(it, None)
            if nxt is not None and re.match(r"^-\d|^-\.\d", nxt):
                out.append(f"{a}={nxt}")
                continue
            out.append(a)
            if nxt is not None:
                out.append(nxt)
        else:
            out.append(a)
    return out


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(join_negative_values(sys.argv[1:] if argv is None else list(argv)))
    L.set_verbose(args.verbose)
    try:
        params, extra = resolve_params(args)
        scans = find_scans(args.scans or extra.get("scans", []))
        if args.list_only:
            for s in scans:
                print(s)
            print(f"{len(scans)} scan(s)")
            return 0
        if not scans:
            parser.print_usage(sys.stderr)
            print("error: no scans given (pass .ply files or directories)", file=sys.stderr)
            return 1

        from roughness.pipeline import LEAD_COLUMNS, NoiseModel, process_scan
        from scan_eval.pipeline import unique_run_names
        from scan_eval.report import append_results

        reference = None
        if params.reference_def:
            from roughness.validate import load_height_reference
            reference = load_height_reference(params)

        out_root = Path(params.output)
        out_root.mkdir(parents=True, exist_ok=True)
        names = unique_run_names(scans, params.label)
        noise = NoiseModel(params)
        rows, failures = [], []
        for scan, name in zip(scans, names):
            try:
                rows.extend(process_scan(scan, params, out_root, name, noise, reference))
            except (ValueError, FileNotFoundError, SurfaceError) as exc:
                warn(f"{scan}: {exc}")
                failures.append((scan, str(exc)))
        order = {m: i for i, m in enumerate(DETREND_ORDERS)}
        rows.sort(key=lambda r: (str(r["scan"]), order[r["detrend"]]))
        print_summary(rows)
        if rows:
            batch = out_root / "roughness_batch_summary.csv"
            batch.unlink(missing_ok=True)
            for r in rows:
                append_results(batch, r, lead_columns=LEAD_COLUMNS)
            log(f"\n[write] this batch, sorted by scan then method: {batch}")
        log(f"[write] all runs: {out_root / 'roughness_results.csv'}")
        if failures:
            log(f"\n{len(failures)} scan(s) failed:")
            for scan, msg in failures:
                log(f"  {scan}: {msg}")
            return 1
        return 0
    except (SurfaceError, FileNotFoundError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
