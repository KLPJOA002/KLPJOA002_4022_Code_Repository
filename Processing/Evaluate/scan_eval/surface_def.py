"""Loading a reference from the command line: a mesh file, or a TOML
definition of an analytic surface, a mesh with metadata, or a plugin.

    name  = "egg-carton-3cm"
    type  = "height_field"          # height_field | parametric | implicit | mesh | plugin
    units = "mm"

    [params]                        # named numbers used in the expressions
    A = 15.0

    [surface]                       # z = ... | x/y/z = ... of (u, v) | F = ...
    z = "c - A*sin(2*pi*x/L)*cos(2*pi*y/L)"

    [domain]
    x = [0, 100]
    y = [0, 100]

See surfaces/ for one example of each type.
"""

from __future__ import annotations

import hashlib
import importlib.util
import inspect
import json
import tomllib
from pathlib import Path

import numpy as np

from .analytic import HeightFieldReference, ImplicitReference, ParametricReference
from .expression import ExpressionError, validate_params
from .mesh_reference import MESH_EXTENSIONS, MeshReference
from .reference import ReferenceSurface, SurfaceError, parse_direction, unit_factor

DEFINITION_EXTENSIONS = {".toml"}

_COMMON_KEYS = {"name", "type", "units", "description", "visible_from", "flip_normals",
                "params", "export"}
_TYPE_KEYS = {
    "height_field": {"surface", "domain"},
    "parametric": {"surface", "domain"},
    "implicit": {"surface", "domain"},
    "mesh": {"path", "height_field"},
    "plugin": {"path", "class"},
}


def load_reference(path, units_override: str | None = None, height_field: bool | None = None,
                   visible_from=None) -> ReferenceSurface:
    """Build a reference from a mesh file or a definition file, by extension."""
    path = Path(path)
    if not path.exists():
        raise SurfaceError(f"reference not found: {path}")
    ext = path.suffix.lower()
    if ext in MESH_EXTENSIONS:
        ref = MeshReference(path, units=units_override or "mm",
                            height_field=bool(height_field), visible_from=visible_from)
        ref.units_source = "command line" if units_override else "default for mesh files"
        return ref
    if ext in DEFINITION_EXTENSIONS:
        return load_definition(path, units_override, height_field, visible_from)
    if ext in (".yaml", ".yml"):
        raise SurfaceError(f"{path}: definitions are TOML (key = value, like Fuse's rig config); "
                           "see surfaces/*.toml for the format")
    raise SurfaceError(f"{path}: cannot tell what kind of reference this is from '{ext}' "
                       f"(meshes: {', '.join(sorted(MESH_EXTENSIONS))}; definitions: .toml)")


def load_definition(path, units_override=None, height_field=None, visible_from=None):
    path = Path(path)
    try:
        with open(path, "rb") as fh:
            spec = tomllib.load(fh)
    except tomllib.TOMLDecodeError as exc:
        raise SurfaceError(f"{path}: invalid TOML: {exc}") from None

    kind = spec.get("type")
    if kind not in _TYPE_KEYS:
        raise SurfaceError(f"{path}: 'type' must be one of {', '.join(_TYPE_KEYS)} (got {kind!r})")
    unknown = set(spec) - _COMMON_KEYS - _TYPE_KEYS[kind]
    if unknown:
        raise SurfaceError(f"{path}: unknown key(s) {sorted(unknown)} for type '{kind}'")

    name = spec.get("name", path.stem)
    units = units_override or spec.get("units")
    if units is None:
        raise SurfaceError(f"{path}: 'units' is required (e.g. units = \"mm\")")
    unit_factor(units)  # validates
    view = visible_from if visible_from is not None else spec.get("visible_from")
    flip = bool(spec.get("flip_normals", False))
    try:
        params = validate_params(spec.get("params", {}), str(path))
    except ExpressionError as exc:
        raise SurfaceError(str(exc)) from None
    surface = spec.get("surface", {})
    domain = spec.get("domain", {})

    def need(table, key, what):
        if key not in table:
            raise SurfaceError(f"{path}: [{what}] needs '{key}'")
        return table[key]

    try:
        if kind == "height_field":
            ref = HeightFieldReference(name, units, params, need(surface, "z", "surface"),
                                       _domain(domain, ("x", "y"), path), mask=domain.get("mask"),
                                       flip_normals=flip,
                                       visible_from=view if view is not None else "+z", source=spec)
        elif kind == "parametric":
            exprs = {k: need(surface, k, "surface") for k in ("x", "y", "z")}
            ref = ParametricReference(name, units, params, exprs, _domain(domain, ("u", "v"), path),
                                      mask=domain.get("mask"), flip_normals=flip,
                                      visible_from=view, source=spec)
        elif kind == "implicit":
            ref = ImplicitReference(name, units, params, need(surface, "F", "surface"),
                                    _domain(domain, ("x", "y", "z"), path), flip_normals=flip,
                                    visible_from=view, source=spec)
        elif kind == "mesh":
            mesh_path = (path.parent / need(spec, "path", "top level")).resolve()
            hf = height_field if height_field is not None else bool(spec.get("height_field", False))
            ref = MeshReference(mesh_path, units=units, name=name, height_field=hf, visible_from=view)
        else:
            ref = _load_plugin(path, spec, name, units, params, view)
    except ExpressionError as exc:
        raise SurfaceError(f"{path}: {exc}") from None
    ref.definition_path = path
    ref.units_source = "command line" if units_override else f"definition file {path.name}"
    ref.export_options = spec.get("export", {})
    return ref


def _domain(table, axes, path):
    missing = [a for a in axes if a not in table]
    if missing:
        raise SurfaceError(f"{path}: [domain] needs {', '.join(missing)} = [min, max]")
    extra = set(table) - set(axes) - {"mask"}
    if extra:
        raise SurfaceError(f"{path}: [domain] has unexpected key(s) {sorted(extra)} "
                           f"(expected {', '.join(axes)} and optional mask)")
    return table


def _load_plugin(path, spec, name, units, params, view):
    plugin_path = (path.parent / spec.get("path", "")).resolve()
    if not plugin_path.is_file():
        raise SurfaceError(f"{path}: plugin file not found: {plugin_path}")
    class_name = spec.get("class")
    if not class_name:
        raise SurfaceError(f"{path}: plugin definitions need class = \"YourClassName\"")

    module_name = f"scan_eval_plugin_{hashlib.sha1(str(plugin_path).encode()).hexdigest()[:10]}"
    module_spec = importlib.util.spec_from_file_location(module_name, plugin_path)
    module = importlib.util.module_from_spec(module_spec)
    try:
        module_spec.loader.exec_module(module)
    except Exception as exc:
        raise SurfaceError(f"plugin {plugin_path.name} failed to import: {type(exc).__name__}: {exc}") from exc
    cls = getattr(module, class_name, None)
    if cls is None or not inspect.isclass(cls):
        raise SurfaceError(f"plugin {plugin_path.name} has no class '{class_name}'")
    if not issubclass(cls, ReferenceSurface):
        raise SurfaceError(f"plugin class {class_name} must subclass scan_eval.ReferenceSurface")
    try:
        ref = cls(**params)
    except TypeError as exc:
        raise SurfaceError(f"plugin {class_name}({', '.join(params)}): {exc}") from None

    ref.name = name
    ref.units = units
    ref.kind = "plugin"
    if view is not None:
        ref.visible_from = parse_direction(view)
    source_hash = hashlib.sha256(plugin_path.read_bytes()).hexdigest()
    inner_definition = ref.definition

    def definition():
        d = inner_definition()
        d.update({"kind": "plugin", "plugin_sha256": source_hash, "class": class_name,
                  "params": {k: repr(v) for k, v in sorted(params.items())}, "units": units})
        return d

    ref.definition = definition
    _smoke_test_plugin(ref, plugin_path.name)
    return ref


def _smoke_test_plugin(ref, label):
    """Catch a plugin that returns the wrong shapes now, not mid-evaluation."""
    try:
        pts, normals = ref.sample(64, 0)
        closest, cn = ref.closest_points(pts + 1e-3 * normals)
        lo, hi = ref.bounds()
    except Exception as exc:
        raise SurfaceError(f"plugin {label}: calling its methods failed: {type(exc).__name__}: {exc}") from exc
    for what, arr in (("sample() points", pts), ("sample() normals", normals),
                      ("closest_points() points", closest), ("closest_points() normals", cn)):
        arr = np.asarray(arr)
        if arr.shape != (64, 3) or not np.all(np.isfinite(arr)):
            raise SurfaceError(f"plugin {label}: {what} must be a finite (n, 3) array, got shape {arr.shape}")
    if np.shape(lo) != (3,) or np.shape(hi) != (3,):
        raise SurfaceError(f"plugin {label}: bounds() must return two length-3 arrays")
    if np.max(np.linalg.norm(closest - pts, axis=1)) > 1e-2 * np.linalg.norm(np.subtract(hi, lo)):
        raise SurfaceError(f"plugin {label}: closest_points() does not return points on the surface "
                           "it samples (a point just off a sample should project back onto it)")


def definition_summary(ref) -> str:
    return json.dumps(ref.definition(), sort_keys=True, default=str)
