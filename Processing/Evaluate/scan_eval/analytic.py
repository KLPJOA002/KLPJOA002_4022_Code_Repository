"""Analytic references: the ground truth is the exact mathematical surface.

Three forms are supported, all built from expression strings (see
expression.py for what is allowed) plus named numeric parameters:

    height field   z = f(x, y)               over a rectangle, optional mask
    parametric     (x, y, z) = S(u, v)       over a rectangle, optional mask
    implicit       F(x, y, z) = 0            inside a bounding box

sympy differentiates the expressions, so normals and the Jacobians needed for
closest-point projection come for free and exactly; everything is then
lambdified to vectorised numpy.

Distances are true point-to-surface distances, found by projecting each point
onto the surface from a nearest-sample initial guess - a damped Newton
iteration in (u, v) for height fields and parametric surfaces, and foot-point
iteration (tangent-plane projection + Newton back onto F=0) for implicit
surfaces. The vertical difference z - f(x, y) of a height field is reported
separately as the Z error; it is not the distance.

A height field is treated as the parametric surface S(x, y) = (x, y, f(x, y)),
so it shares one closest-point solver with the general parametric case.
"""

from __future__ import annotations

import numpy as np
import sympy as sp
from scipy.spatial import cKDTree

from .expression import (ExpressionError, lambdify, parse_expression,
                         substitute_params)
from .log import log, warn
from .reference import ReferenceSurface, SurfaceError, parse_direction

# Resolution of the grid used to check an expression over its domain, seed
# closest-point searches and integrate area. 600x600 puts the seeds ~0.2 mm
# apart on a 100 mm part, well inside Newton's basin of convergence.
_GRID = 600


def _span(pair, label):
    try:
        lo, hi = float(pair[0]), float(pair[1])
    except (TypeError, ValueError, IndexError):
        raise SurfaceError(f"domain '{label}' must be [min, max], got {pair!r}") from None
    if not (np.isfinite(lo) and np.isfinite(hi) and hi > lo):
        raise SurfaceError(f"domain '{label}' must satisfy min < max, got {pair!r}")
    return lo, hi


class ParametricReference(ReferenceSurface):
    """(x, y, z) = S(u, v) over a rectangular (u, v) domain, optionally masked."""

    kind = "analytic"
    exact = True
    form = "parametric"

    def __init__(self, name, units, params, exprs, domain, variables=("u", "v"),
                 mask=None, flip_normals=False, visible_from=None, source=None):
        self.name = name
        self.units = units
        self.params = dict(params)
        self.variables = tuple(variables)
        self.flip = -1.0 if flip_normals else 1.0
        self.visible_from = parse_direction(visible_from)
        self.source = source or {}
        u, v = self.variables
        self.u_range = _span(domain.get(u), u)
        self.v_range = _span(domain.get(v), v)

        # Symbolic, with parameters still as symbols: this is what gets hashed.
        self.symbolic = {k: parse_expression(exprs[k], self.variables, self.params)
                         for k in ("x", "y", "z")}
        self.mask_symbolic = (parse_expression(mask, self.variables, self.params, allow_logic=True)
                              if mask else None)
        self._build()
        self._check_domain()

    # --- construction ----------------------------------------------------
    def _build(self):
        us, vs = (sp.Symbol(n, real=True) for n in self.variables)
        comps = [substitute_params(self.symbolic[k], self.params) for k in ("x", "y", "z")]
        leftover = set().union(*(c.free_symbols for c in comps)) - {us, vs}
        if leftover:
            raise ExpressionError(f"{self.name}: unresolved symbols {sorted(map(str, leftover))}")

        def vec(exprs):
            fns = [lambdify(self.variables, e) for e in exprs]
            return lambda a, b: np.stack([f(a, b) for f in fns], axis=-1)

        self._S = vec(comps)
        self._Su = vec([sp.diff(c, us) for c in comps])
        self._Sv = vec([sp.diff(c, vs) for c in comps])
        self._Suu = vec([sp.diff(c, us, 2) for c in comps])
        self._Suv = vec([sp.diff(c, us, vs) for c in comps])
        self._Svv = vec([sp.diff(c, vs, 2) for c in comps])
        if self.mask_symbolic is not None:
            m = lambdify(self.variables, substitute_params(self.mask_symbolic, self.params))
            self._mask = lambda a, b: m(a, b) > 0.5
        else:
            self._mask = lambda a, b: np.ones(np.broadcast_shapes(np.shape(a), np.shape(b)), bool)

    def _grid(self, n=_GRID):
        u = np.linspace(*self.u_range, n)
        v = np.linspace(*self.v_range, n)
        return np.meshgrid(u, v, indexing="ij")

    def _check_domain(self):
        """Fail now, with a location, rather than produce NaN errors later."""
        U, V = self._grid(257)
        inside = self._mask(U, V)
        if not inside.any():
            raise SurfaceError(f"{self.name}: the domain mask excludes the whole domain")
        for label, fn in (("surface", self._S), ("first derivative (d/du)", self._Su),
                          ("first derivative (d/dv)", self._Sv)):
            val = fn(U, V)
            bad = inside & ~np.all(np.isfinite(val), axis=-1)
            if bad.any():
                i = np.argwhere(bad)[0]
                u_n, v_n = self.variables
                raise SurfaceError(
                    f"{self.name}: the {label} is undefined (NaN/inf) at {u_n}={U[tuple(i)]:.6g}, "
                    f"{v_n}={V[tuple(i)]:.6g} and {int(bad.sum())} other grid points of the domain "
                    "(e.g. sqrt of a negative number, log of zero, division by zero). "
                    "Shrink [domain] or add a mask that excludes that region.")
        cross = np.cross(self._Su(U, V), self._Sv(U, V))
        degenerate = inside & (np.linalg.norm(cross, axis=-1) < 1e-12)
        if degenerate.mean() > 0.001:
            warn(f"{self.name}: the parametrisation is degenerate (zero area element) on "
                 f"{100 * degenerate.mean():.1f}% of the domain - normals are undefined there")

    # --- geometry helpers -------------------------------------------------
    def _normal(self, u, v):
        n = np.cross(self._Su(u, v), self._Sv(u, v)) * self.flip
        return n / np.maximum(np.linalg.norm(n, axis=-1, keepdims=True), 1e-300)

    def _seed_pool(self):
        if getattr(self, "_pool", None) is None:
            U, V = self._grid()
            keep = self._mask(U, V)
            uv = np.stack([U[keep], V[keep]], axis=1)
            pts = self._S(uv[:, 0], uv[:, 1])
            self._pool = (uv, pts, cKDTree(pts))
        return self._pool

    def _clamp(self, uv):
        uv[:, 0] = np.clip(uv[:, 0], *self.u_range)
        uv[:, 1] = np.clip(uv[:, 1], *self.v_range)
        return uv

    def _project(self, points):
        """Closest (u, v) for each point: damped Newton on |S(u,v) - p|^2 with
        a backtracking line search, box-constrained to the domain."""
        uv_pool, _, tree = self._seed_pool()
        _, idx = tree.query(points, workers=-1)
        uv = uv_pool[idx].copy()
        span = np.array([np.ptp(self.u_range), np.ptp(self.v_range)])
        tol = 1e-12 * span.max()

        def objective(uv_, p):
            r = self._S(uv_[:, 0], uv_[:, 1]) - p
            return np.einsum("ij,ij->i", r, r)

        active = np.arange(len(points))
        f = objective(uv, points)
        for _ in range(60):
            if active.size == 0:
                break
            p, x = points[active], uv[active]
            a, b = x[:, 0], x[:, 1]
            r = self._S(a, b) - p
            Su, Sv = self._Su(a, b), self._Sv(a, b)
            g = np.stack([np.einsum("ij,ij->i", Su, r), np.einsum("ij,ij->i", Sv, r)], axis=1)
            H = np.empty((len(active), 2, 2))
            H[:, 0, 0] = np.einsum("ij,ij->i", Su, Su) + np.einsum("ij,ij->i", self._Suu(a, b), r)
            H[:, 0, 1] = H[:, 1, 0] = np.einsum("ij,ij->i", Su, Sv) + np.einsum("ij,ij->i", self._Suv(a, b), r)
            H[:, 1, 1] = np.einsum("ij,ij->i", Sv, Sv) + np.einsum("ij,ij->i", self._Svv(a, b), r)
            # Full Newton where the Hessian is positive definite, Gauss-Newton
            # (always PD) elsewhere, e.g. on the concave side past a focal point.
            det = H[:, 0, 0] * H[:, 1, 1] - H[:, 0, 1] ** 2
            bad = (det <= 0) | (H[:, 0, 0] <= 0)
            if bad.any():
                H[bad, 0, 0] = np.einsum("ij,ij->i", Su[bad], Su[bad])
                H[bad, 0, 1] = H[bad, 1, 0] = np.einsum("ij,ij->i", Su[bad], Sv[bad])
                H[bad, 1, 1] = np.einsum("ij,ij->i", Sv[bad], Sv[bad])
            H[:, 0, 0] += 1e-12 * (H[:, 0, 0] + 1)
            H[:, 1, 1] += 1e-12 * (H[:, 1, 1] + 1)
            step = -np.linalg.solve(H, g[..., None])[..., 0]

            f_old = f[active]
            t = np.ones(len(active))
            accepted = np.zeros(len(active), bool)
            new = x.copy()
            for _ in range(12):
                todo = ~accepted
                trial = self._clamp(x[todo] + t[todo, None] * step[todo])
                f_trial = objective(trial, p[todo])
                ok = f_trial <= f_old[todo] + 1e-18
                sel = np.flatnonzero(todo)[ok]
                new[sel] = trial[ok]
                f[active[sel]] = f_trial[ok]
                accepted[sel] = True
                if accepted.all():
                    break
                t[~accepted] *= 0.5
            moved = np.abs(new - x).max(axis=1)
            uv[active] = new
            active = active[(moved > tol) & accepted]
        return uv

    # --- interface --------------------------------------------------------
    def closest_points(self, points):
        points = np.asarray(points, dtype=float)
        if len(points) == 0:
            return np.empty((0, 3)), np.empty((0, 3))
        uv = self._project(points)
        outside = ~self._mask(uv[:, 0], uv[:, 1])
        if outside.any():
            # Masked (non-rectangular) domains: the unconstrained optimum left
            # the region. Fall back to the nearest in-region seed - exact to
            # the seed spacing, and only ever at the region's rim.
            uv_pool, _, tree = self._seed_pool()
            _, idx = tree.query(points[outside], workers=-1)
            uv[outside] = uv_pool[idx]
        return self._S(uv[:, 0], uv[:, 1]), self._normal(uv[:, 0], uv[:, 1])

    def sample(self, n, seed=0):
        rng = np.random.default_rng(seed)
        U, V = self._grid(200)
        area = np.linalg.norm(np.cross(self._Su(U, V), self._Sv(U, V)), axis=-1)
        a_max = 1.1 * area[self._mask(U, V)].max()
        out_uv = []
        have = 0
        while have < n:
            m = max(1024, 2 * (n - have))
            u = rng.uniform(*self.u_range, m)
            v = rng.uniform(*self.v_range, m)
            a = np.linalg.norm(np.cross(self._Su(u, v), self._Sv(u, v)), axis=-1)
            keep = self._mask(u, v) & (rng.random(m) * a_max < a)
            out_uv.append(np.stack([u[keep], v[keep]], axis=1))
            have += int(keep.sum())
        uv = np.concatenate(out_uv)[:n]
        return self._S(uv[:, 0], uv[:, 1]), self._normal(uv[:, 0], uv[:, 1])

    def bounds(self):
        _, pts, _ = self._seed_pool()
        return pts.min(axis=0), pts.max(axis=0)

    def surface_area(self):
        n = _GRID
        U, V = self._grid(n)
        du = np.ptp(self.u_range) / (n - 1)
        dv = np.ptp(self.v_range) / (n - 1)
        a = np.linalg.norm(np.cross(self._Su(U, V), self._Sv(U, V)), axis=-1)
        w = np.ones_like(a)
        w[[0, -1], :] *= 0.5
        w[:, [0, -1]] *= 0.5
        keep = self._mask(U, V)
        if self.visible_from is not None:
            keep &= self._normal(U, V) @ self.visible_from > 0.05
        return float((a * w * keep).sum() * du * dv)

    def definition(self):
        return {
            "kind": "analytic", "form": self.form, "units": self.units,
            "variables": list(self.variables),
            "expressions": {k: sp.srepr(e) for k, e in self.symbolic.items()},
            "mask": None if self.mask_symbolic is None else sp.srepr(self.mask_symbolic),
            "params": {k: repr(v) for k, v in sorted(self.params.items())},
            "domain": {self.variables[0]: list(self.u_range), self.variables[1]: list(self.v_range)},
            "flip_normals": self.flip < 0,
        }

    def describe(self):
        d = super().describe()
        d.update({"form": self.form, "params": self.params,
                  "expressions": {k: str(e) for k, e in self.symbolic.items()},
                  "mask": None if self.mask_symbolic is None else str(self.mask_symbolic)})
        return d


class HeightFieldReference(ParametricReference):
    """z = f(x, y): the parametric surface S(x, y) = (x, y, f(x, y))."""

    form = "height_field"
    is_height_field = True

    def __init__(self, name, units, params, z_expr, domain, mask=None,
                 flip_normals=False, visible_from="+z", source=None):
        super().__init__(name, units, params, {"x": "x", "y": "y", "z": z_expr}, domain,
                         variables=("x", "y"), mask=mask, flip_normals=flip_normals,
                         visible_from=visible_from, source=source)
        self._f = lambdify(("x", "y"), substitute_params(self.symbolic["z"], self.params))

    def height(self, x, y):
        x, y = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float))
        inside = ((x >= self.u_range[0]) & (x <= self.u_range[1]) &
                  (y >= self.v_range[0]) & (y <= self.v_range[1]))
        inside &= self._mask(x, y)
        z = np.full(x.shape, np.nan)
        z[inside] = self._f(x[inside], y[inside])
        return z

    def region_mask(self, points, margin=0.0):
        p = np.asarray(points, dtype=float)
        x, y = p[:, 0], p[:, 1]
        inside = ((x >= self.u_range[0] - margin) & (x <= self.u_range[1] + margin) &
                  (y >= self.v_range[0] - margin) & (y <= self.v_range[1] + margin))
        if self.mask_symbolic is not None:
            xc = np.clip(x, *self.u_range)
            yc = np.clip(y, *self.v_range)
            inside &= self._mask(xc, yc) | (margin > 0) & (np.hypot(x - xc, y - yc) <= margin)
        return inside


class ImplicitReference(ReferenceSurface):
    """F(x, y, z) = 0, restricted to a bounding box."""

    kind = "analytic"
    exact = True
    form = "implicit"

    def __init__(self, name, units, params, f_expr, domain, flip_normals=False,
                 visible_from=None, source=None):
        self.name = name
        self.units = units
        self.params = dict(params)
        self.flip = -1.0 if flip_normals else 1.0
        self.visible_from = parse_direction(visible_from)
        self.source = source or {}
        self.box = np.array([_span(domain.get(a), a) for a in "xyz"])  # (3, 2)
        self.symbolic = parse_expression(f_expr, ("x", "y", "z"), self.params)
        F = substitute_params(self.symbolic, self.params)
        xs = [sp.Symbol(a, real=True) for a in "xyz"]
        leftover = F.free_symbols - set(xs)
        if leftover:
            raise ExpressionError(f"{self.name}: unresolved symbols {sorted(map(str, leftover))}")
        self._F = lambdify("xyz", F)
        grads = [lambdify("xyz", sp.diff(F, s)) for s in xs]
        self._G = lambda x, y, z: np.stack([g(x, y, z) for g in grads], axis=-1)
        self._check_domain()
        self._pool = None

    def _check_domain(self):
        axes = [np.linspace(lo, hi, 65) for lo, hi in self.box]
        X, Y, Z = np.meshgrid(*axes, indexing="ij")
        F = self._F(X, Y, Z)
        G = self._G(X, Y, Z)
        # F itself must be defined everywhere in the box. Its gradient only
        # matters where the surface is - e.g. a torus written with
        # sqrt(x^2+y^2) has no gradient on its axis, which it never touches -
        # so a bad gradient is only an error in a cell F=0 passes through.
        sign = np.sign(np.nan_to_num(F))
        near = np.zeros_like(sign, dtype=bool)
        for axis in range(3):
            change = np.diff(sign, axis=axis) != 0
            lo = [slice(None)] * 3
            hi = [slice(None)] * 3
            lo[axis] = slice(0, -1)
            hi[axis] = slice(1, None)
            near[tuple(lo)] |= change
            near[tuple(hi)] |= change
        bad = ~np.isfinite(F) | (near & ~np.all(np.isfinite(G), axis=-1))
        if bad.any():
            i = tuple(np.argwhere(bad)[0])
            raise SurfaceError(
                f"{self.name}: F or its gradient is undefined (NaN/inf) at x={X[i]:.6g}, "
                f"y={Y[i]:.6g}, z={Z[i]:.6g} ({int(bad.sum())} of {bad.size} grid points). "
                "Shrink the [domain] box or rewrite F so it is defined everywhere in it.")
        if F.min() > 0 or F.max() < 0:
            raise SurfaceError(f"{self.name}: F never changes sign inside the [domain] box, "
                               "so the surface F=0 does not pass through it")

    def _newton_to_surface(self, q, iterations=40):
        q = q.copy()
        for _ in range(iterations):
            F = self._F(q[:, 0], q[:, 1], q[:, 2])
            G = self._G(q[:, 0], q[:, 1], q[:, 2])
            gg = np.maximum(np.einsum("ij,ij->i", G, G), 1e-300)
            q -= (F / gg)[:, None] * G
            if np.all(np.abs(F) / np.sqrt(gg) < 1e-12 * np.ptp(self.box)):
                break
        return q

    def _normal(self, q):
        G = self._G(q[:, 0], q[:, 1], q[:, 2]) * self.flip
        return G / np.maximum(np.linalg.norm(G, axis=1, keepdims=True), 1e-300)

    def _seed_pool(self):
        """Approximately area-uniform points on the surface: random points in
        the box projected onto F=0, then thinned to one per small voxel (a
        projection alone crowds points where the surface is nearest to more of
        the box)."""
        if self._pool is None:
            rng = np.random.default_rng(12345)
            diag = np.linalg.norm(np.ptp(self.box, axis=1))
            h = diag / 500
            pts = []
            for _ in range(3):
                q = rng.uniform(self.box[:, 0], self.box[:, 1], size=(400_000, 3))
                q = self._newton_to_surface(q)
                F = self._F(q[:, 0], q[:, 1], q[:, 2])
                G = np.linalg.norm(self._G(q[:, 0], q[:, 1], q[:, 2]), axis=1)
                ok = (np.abs(F) / np.maximum(G, 1e-300) < 1e-6 * diag)
                ok &= np.all((q >= self.box[:, 0]) & (q <= self.box[:, 1]), axis=1)
                pts.append(q[ok])
            pts = np.concatenate(pts)
            if len(pts) == 0:
                raise SurfaceError(f"{self.name}: could not find any points on F=0 inside the box")
            keys = np.floor((pts - self.box[:, 0]) / h).astype(np.int64)
            _, first = np.unique(keys, axis=0, return_index=True)
            pts = pts[np.sort(first)]
            self._pool = (pts, cKDTree(pts), h)
        return self._pool

    def closest_points(self, points):
        points = np.asarray(points, dtype=float)
        if len(points) == 0:
            return np.empty((0, 3)), np.empty((0, 3))
        pool, tree, h = self._seed_pool()
        _, idx = tree.query(points, workers=-1)
        s = pool[idx].copy()
        tol = 1e-10 * np.ptp(self.box)
        active = np.arange(len(points))
        for _ in range(100):
            if active.size == 0:
                break
            p, x = points[active], s[active]
            n = self._normal(x)
            q = p - np.einsum("ij,ij->i", p - x, n)[:, None] * n   # onto tangent plane
            x_new = self._newton_to_surface(q, iterations=8)
            moved = np.linalg.norm(x_new - x, axis=1)
            s[active] = x_new
            active = active[moved > tol]
        return s, self._normal(s)

    def sample(self, n, seed=0):
        pool, _, _ = self._seed_pool()
        rng = np.random.default_rng(seed)
        idx = rng.choice(len(pool), size=n, replace=n > len(pool))
        pts = pool[idx]
        return pts, self._normal(pts)

    def bounds(self):
        pool, _, _ = self._seed_pool()
        return pool.min(axis=0), pool.max(axis=0)

    def region_mask(self, points, margin=0.0):
        p = np.asarray(points, dtype=float)
        return np.all((p >= self.box[:, 0] - margin) & (p <= self.box[:, 1] + margin), axis=1)

    def surface_area(self):
        try:
            from .export import implicit_mesh
        except ImportError:
            return None
        vertices, faces = implicit_mesh(self, resolution=np.linalg.norm(np.ptp(self.box, axis=1)) / 200)
        tri = vertices[faces]
        cross = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        area = 0.5 * np.linalg.norm(cross, axis=1)
        if self.visible_from is not None:
            normal = self._normal(tri.mean(axis=1))
            area = area[normal @ self.visible_from > 0.05]
        return float(area.sum())

    def definition(self):
        return {"kind": "analytic", "form": "implicit", "units": self.units,
                "expressions": {"F": sp.srepr(self.symbolic)},
                "params": {k: repr(v) for k, v in sorted(self.params.items())},
                "domain": {a: list(self.box[i]) for i, a in enumerate("xyz")},
                "flip_normals": self.flip < 0}

    def describe(self):
        d = super().describe()
        d.update({"form": "implicit", "params": self.params, "expressions": {"F": str(self.symbolic)}})
        return d
