#!/usr/bin/env python3
"""evaluate_scans - geometric error of 3D scans against a ground-truth reference.

Usage:
  evaluate_scans.py --reference part.stl [--reference-units mm] scan.ply ...
  evaluate_scans.py --reference-def surfaces/egg_carton.toml scans_dir/ ...
  evaluate_scans.py --reference-def surfaces/egg_carton.toml --export egg.stl

The sibling of Fuse's fuse_scans: where Fuse aligns scans to each other and
fuses them, this aligns each scan (single capture or Fuse output) rigidly to a
known reference and measures how far it is from the truth. The reference may be
a mesh or an exact analytic surface (see surfaces/ and scan_eval/reference.py).

Stages per scan (printed like Fuse's):

  load -> units check -> align (multi-start + exact refine) -> metrics -> write

Every scan gets its own folder under -o with metrics.json, the transform, an
error-coloured PLY, plots and a log, and one row appended to <out>/results.csv
so runs of different hardware and fusing algorithms tabulate side by side.
A TOML --config sets baseline values; flags on the command line win, as with
Fuse's --rig-config. All distances on the command line are millimetres.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from scan_eval import log as L  # noqa: E402
from scan_eval.expression import ExpressionError  # noqa: E402
from scan_eval.log import log, stage, warn  # noqa: E402
from scan_eval.params import EvalParams, load_config, param_names  # noqa: E402
from scan_eval.reference import SurfaceError, UnitScaled  # noqa: E402

UNIT_CHOICES = ["m", "cm", "mm", "um", "in"]


def csv_floats(text):
    try:
        values = [float(t) for t in str(text).split(",") if t.strip()]
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected comma-separated numbers, got '{text}'") from None
    if not values or any(v <= 0 for v in values):
        raise argparse.ArgumentTypeError(f"expected positive numbers, got '{text}'")
    return values


def build_parser():
    p = argparse.ArgumentParser(
        prog="evaluate_scans.py",
        description="Evaluate the geometric error of 3D scans (.ply) against a ground-truth "
                    "reference (mesh or analytic surface).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""examples:
  # CAD mesh as truth (meshes carry no units: mm is assumed unless told otherwise)
  evaluate_scans.py --reference part.stl --reference-units mm --label d435_close scan.ply

  # exact analytic surface, a whole folder of Fuse outputs, one comparison table
  evaluate_scans.py --reference-def surfaces/egg_carton.toml --label fuse_heightfield Fused_Scans/

  # reproduce / seed an alignment
  evaluate_scans.py --reference-def surfaces/egg_carton.toml --init-transform prev/transform.txt scan.ply

  # export the analytic reference as a printable solid
  evaluate_scans.py --reference-def surfaces/egg_carton.toml --export egg.stl --export-base 0
""")
    ref = p.add_argument_group("reference (one of)")
    ref.add_argument("--reference", metavar="PATH",
                     help="mesh (.stl/.ply/.obj) or definition (.toml); type from the extension")
    ref.add_argument("--reference-def", metavar="PATH",
                     help="analytic / mesh / plugin definition (.toml); see surfaces/")
    ref.add_argument("--reference-units", choices=UNIT_CHOICES,
                     help="units of the reference (bare meshes default to mm; overrides a definition)")
    ref.add_argument("--height-field", action="store_true", default=None,
                     help="treat a bare mesh as a height field z=h(x,y): region = outline seen "
                          "from above, Z-error stats, only upward-facing surface counts")
    ref.add_argument("--visible-from", metavar="DIR",
                     help="side of the reference a scanner sees: +z, -x, 'x,y,z' or none "
                          "(default: from the definition; height fields +z)")

    p.add_argument("scans", nargs="*", help=".ply files and/or directories (searched recursively)")
    p.add_argument("-o", "--output", help="output directory (default evaluation_out)")
    p.add_argument("--label", help="label for this run in results.csv, e.g. d435_close_icp")
    p.add_argument("--config", metavar="TOML", help="baseline parameters; flags override it "
                                                   "(see eval_config.example.toml)")
    p.add_argument("--seed", type=int, help="random seed (default 0); fixes every random choice")
    p.add_argument("--list-only", action="store_true", help="print the scans found and exit")
    p.add_argument("-v", "--verbose", action="store_true", help="more detail on the console")

    u = p.add_argument_group("units and scale")
    u.add_argument("--units", choices=UNIT_CHOICES, help="units of the scan PLYs (default m)")
    u.add_argument("--scale", type=float, help="extra factor applied to the scans (default 1). "
                                               "Scale is never estimated unless --allow-scale")
    u.add_argument("--allow-scale", action="store_true", default=None,
                   help="similarity alignment: also solve for and report a uniform scale")

    a = p.add_argument_group("alignment")
    a.add_argument("--align", choices=["full", "refine", "none"],
                   help="full: feature matching + multi-start + ICP (default); refine: ICP from "
                        "--init-transform/identity only; none: use --init-transform as-is")
    a.add_argument("--init-transform", metavar="FILE",
                   help="4x4 matrix (scan metres -> reference metres), e.g. a previous transform.txt")
    a.add_argument("--align-voxel", type=float, metavar="MM", dest="align_voxel_mm",
                   help="voxel for the alignment copy of the scan (default: reference diagonal/60)")
    a.add_argument("--eval-distance", type=float, metavar="MM", dest="eval_distance_mm",
                   help="correspondence distance for scoring poses (default 3 x voxel)")
    a.add_argument("--starts", type=int, dest="ransac_starts",
                   help="independent feature-matching starts (default 5)")
    a.add_argument("--no-perturb", action="store_false", dest="perturb", default=None,
                   help="skip the turn/flip/slide multi-start around the best pose")
    a.add_argument("--sensor-origin", metavar="X,Y,Z",
                   help="sensor position in the scan's frame, mm (default 0,0,0 - true for RealSense "
                        "and Fuse output), or 'none' to not use which side the scan was seen from")
    a.add_argument("--ambiguous-margin", type=float,
                   help="flag the alignment if the runner-up is within this fraction (default 0.15)")

    m = p.add_argument_group("metrics (distances in mm)")
    m.add_argument("--thresholds", type=csv_floats, dest="thresholds_mm", metavar="MM,MM,...",
                   help="completeness / precision / F-score thresholds (default 0.5,1,2,5)")
    m.add_argument("--outlier-distance", type=float, dest="outlier_distance_mm", metavar="MM",
                   help="scan points farther than this are outliers (default 5)")
    m.add_argument("--coverage-radius", type=float, dest="coverage_radius_mm", metavar="MM",
                   help="reference counts as covered within this distance of the scan (default 10)")
    m.add_argument("--region-margin", type=float, dest="region_margin_mm", metavar="MM",
                   help="score scan points up to this far outside the part's outline (default 0)")
    m.add_argument("--trim", type=csv_floats, dest="trim_percent", metavar="PCT,PCT",
                   help="percent of worst points dropped for the robust stats (default 1,5)")
    m.add_argument("--reference-samples", type=int,
                   help="reference samples for completeness (default 200000)")

    o = p.add_argument_group("outputs")
    o.add_argument("--no-ply", action="store_false", dest="write_ply", default=None,
                   help="don't write the error-coloured aligned PLY")
    o.add_argument("--no-plots", action="store_false", dest="write_plots", default=None,
                   help="don't write the histogram / error map")
    o.add_argument("--export", metavar="PATH",
                   help="export the reference to .stl/.ply/.obj (in its own units) and exit")
    o.add_argument("--export-resolution", type=float, metavar="R",
                   help="vertex spacing for --export, in the reference's units")
    o.add_argument("--export-base", type=float, metavar="Z",
                   help="height fields: close the surface into a solid down to z = Z")
    return p


def resolve_params(args) -> tuple[EvalParams, dict]:
    """Defaults <- config file <- command line, as in Fuse."""
    params = EvalParams()
    extra = {}
    if args.config:
        config = load_config(args.config)
        reference_keys = {"reference", "reference_def", "height_field", "scans"}
        unknown = set(config) - param_names() - reference_keys
        if unknown:
            raise ValueError(f"{args.config}: unknown setting(s) {sorted(unknown)}")
        for key, value in config.items():
            if key in reference_keys:
                extra[key] = value
            else:
                setattr(params, key, value)
    for key in param_names():
        value = getattr(args, key, None)
        if value is not None:
            setattr(params, key, value)
    if params.units not in UNIT_CHOICES:
        raise ValueError(f"units must be one of {UNIT_CHOICES}")
    if params.scale <= 0:
        raise ValueError("--scale must be positive")
    if params.align not in ("full", "refine", "none"):
        raise ValueError("align must be full, refine or none")
    return params, extra


def find_scans(inputs) -> list[Path]:
    found = []
    for item in inputs:
        path = Path(item)
        if path.is_dir():
            hits = sorted(q for q in path.rglob("*") if q.suffix.lower() == ".ply" and q.is_file())
            if not hits:
                warn(f"no .ply files under {path}")
            found.extend(hits)
        elif path.is_file():
            found.append(path)
        else:
            raise FileNotFoundError(f"scan not found: {path}")
    unique = []
    for p in found:
        if p.resolve() not in {q.resolve() for q in unique}:
            unique.append(p)
    return unique


def load_reference_from_args(args, extra, params):
    from scan_eval.surface_def import load_reference
    path = args.reference or args.reference_def or extra.get("reference") or extra.get("reference_def")
    if args.reference and args.reference_def:
        raise ValueError("--reference and --reference-def are mutually exclusive")
    if not path:
        raise ValueError("no reference given: use --reference MESH or --reference-def DEF.toml")
    height_field = args.height_field if args.height_field is not None else extra.get("height_field")
    return load_reference(path, units_override=params.reference_units, height_field=height_field,
                          visible_from=params.visible_from)


def print_summary(rows, params):
    if not rows:
        return
    mid = params.thresholds_mm[min(len(params.thresholds_mm) - 1, len(params.thresholds_mm) // 2)]
    t = f"{mid:g}mm"
    rows = sorted(rows, key=lambda r: r.get("acc_rmse_mm", float("inf")))
    log(f"\n=== Summary: {len(rows)} scan(s), sorted by RMSE (mm) ===")
    head = (f"  {'label':24s} {'scan':28s} {'RMSE':>7s} {'MAE':>7s} {'bias':>7s} {'std':>7s} "
            f"{'trim5':>7s} {'cover':>6s} {'compl@' + t:>12s} {'F@' + t:>9s}  flags")
    log(head)
    for r in rows:
        flags = []
        if r.get("align_ambiguous"):
            flags.append("AMBIGUOUS")
        if r.get("align_unconstrained"):
            flags.append("DEGENERATE")
        if r.get("units_warning"):
            flags.append("UNITS?")
        log(f"  {str(r['label'])[:24]:24s} {Path(r['scan']).name[:28]:28s} {r['acc_rmse_mm']:7.3f} "
            f"{r['acc_mae_mm']:7.3f} {r['signed_bias_mm']:+7.3f} {r['signed_std_mm']:7.3f} "
            f"{r['acc_rmse_trim5_mm']:7.3f} {r['covered_fraction']:6.1%} "
            f"{r.get(f'completeness@{t}', float('nan')):12.1%} {r.get(f'fscore@{t}', float('nan')):9.1%}"
            f"  {' '.join(flags)}")


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    L.set_verbose(args.verbose)
    try:
        params, extra = resolve_params(args)
        scans = find_scans(args.scans or extra.get("scans", []))
        if args.list_only:
            for s in scans:
                print(s)
            print(f"{len(scans)} scan(s)")
            return 0

        stage("reference")
        ref = load_reference_from_args(args, extra, params)
        if args.export:
            from scan_eval.export import export_reference
            export_reference(ref, args.export, args.export_resolution, args.export_base)
            return 0
        if not scans:
            parser.print_usage(sys.stderr)
            print("error: no scans given (pass .ply files or directories)", file=sys.stderr)
            return 1

        ref_m = UnitScaled(ref)
        ref_m.units_label = f"{ref.units}, from {getattr(ref, 'units_source', 'definition')}"
        info = ref_m.describe()
        log(f"[reference] {ref.name}: {ref.kind}{' (' + info.get('form', '') + ')' if info.get('form') else ''}, "
            f"{'exact analytic' if ref.exact else 'mesh (approximate truth)'}, units {ref_m.units_label}")
        log(f"[reference] definition hash {ref.definition_hash}")
        area = ref_m.surface_area()
        if area:
            log(f"[reference] visible surface area {area * 1e4:.2f} cm^2"
                + (f" (seen from {ref.visible_from.round(3).tolist()})" if ref.visible_from is not None else ""))

        init = None
        if params.init_transform:
            from scan_eval.pipeline import read_transform
            init = read_transform(params.init_transform)

        from scan_eval.pipeline import evaluate_scan, unique_run_names
        out_root = Path(params.output)
        out_root.mkdir(parents=True, exist_ok=True)
        names = unique_run_names(scans, params.label)
        rows, failures = [], []
        for scan, name in zip(scans, names):
            try:
                rows.append(evaluate_scan(ref_m, scan, params, out_root, name, init))
            except (ValueError, FileNotFoundError, SurfaceError) as exc:
                warn(f"{scan}: {exc}")
                failures.append((scan, str(exc)))
        print_summary(rows, params)
        if len(rows) > 1:
            from scan_eval.report import append_results
            batch = out_root / "batch_summary.csv"
            batch.unlink(missing_ok=True)
            for r in sorted(rows, key=lambda r: r.get("acc_rmse_mm", float("inf"))):
                append_results(batch, r)
            log(f"\n[write] this batch sorted by RMSE: {batch}")
        log(f"[write] all runs: {out_root / 'results.csv'}")
        if failures:
            log(f"\n{len(failures)} scan(s) failed:")
            for scan, msg in failures:
                log(f"  {scan}: {msg}")
            return 1
        return 0
    except (SurfaceError, ExpressionError, FileNotFoundError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
