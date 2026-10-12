"""Polynomial detrending: fit z = p(x, y) of total degree n and subtract it.

x and y are normalised to [-1, 1] over the footprint before building the
monomials x^i y^j (i + j <= n), which keeps the least-squares problem well
conditioned up to quartic. The fit is robust: points beyond clip_sigma
(MAD-based) are dropped and the fit repeated, so ripple crests, troughs and
outliers do not drag the trend surface.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .orient import robust_sigma
from .params import DETREND_ORDERS


def exponents(order: int) -> list[tuple[int, int]]:
    return [(i, d - i) for d in range(order + 1) for i in range(d, -1, -1)]


def design(xn: np.ndarray, yn: np.ndarray, order: int) -> np.ndarray:
    return np.stack([xn ** i * yn ** j for i, j in exponents(order)], axis=1)


@dataclass
class TrendFit:
    method: str
    order: int
    centre: np.ndarray        # (2,) m, normalisation centre
    half: np.ndarray          # (2,) m, normalisation half-range
    coefficients: np.ndarray  # in normalised coordinates, metres of z
    inlier_fraction: float
    iterations: int

    def evaluate(self, x, y):
        xn = (np.asarray(x) - self.centre[0]) / self.half[0]
        yn = (np.asarray(y) - self.centre[1]) / self.half[1]
        out = np.zeros(np.broadcast(xn, yn).shape)
        for c, (i, j) in zip(self.coefficients, exponents(self.order)):
            out += c * xn ** i * yn ** j
        return out

    def as_dict(self):
        return {"method": self.method, "order": self.order,
                "n_coefficients": len(self.coefficients),
                "terms": [f"x^{i} y^{j}" for i, j in exponents(self.order)],
                "coefficients_m": self.coefficients,
                "normalisation": "xn = (x - centre_x) / half_x, likewise y; metres, oriented frame",
                "centre_m": self.centre, "half_m": self.half,
                "inlier_fraction": self.inlier_fraction, "iterations": self.iterations}


def fit_trend(points: np.ndarray, method: str, clip_sigma: float, iterations: int) -> TrendFit:
    order = DETREND_ORDERS[method]
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    lo, hi = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
    centre = 0.5 * (lo + hi)
    half = np.maximum(0.5 * (hi - lo), 1e-9)
    A = design((x - centre[0]) / half[0], (y - centre[1]) / half[1], order)
    n_coef = A.shape[1]
    if len(z) < 5 * n_coef:
        raise ValueError(f"{method} detrend needs at least {5 * n_coef} points, have {len(z)}")
    keep = np.ones(len(z), dtype=bool)
    coef, *_ = np.linalg.lstsq(A, z, rcond=None)
    done = 1
    while done < iterations:
        r = z - A @ coef
        sigma = robust_sigma(r[keep])
        if sigma <= 0:
            break
        new = np.abs(r) <= clip_sigma * sigma
        if new.sum() < 5 * n_coef or np.array_equal(new, keep):
            break
        keep = new
        coef, *_ = np.linalg.lstsq(A[keep], z[keep], rcond=None)
        done += 1
    return TrendFit(method, order, centre, half, coef, float(keep.mean()), done)
