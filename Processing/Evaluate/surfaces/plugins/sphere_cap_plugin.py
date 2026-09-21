"""Example plugin reference: a spherical cap with a closed-form closest point.

A plugin is any class that subclasses scan_eval.ReferenceSurface and
implements sample(), closest_points() and bounds() in the units declared by
its .toml file. Everything else (distances, signed error, region, hashing) is
inherited. Override height() and set is_height_field = True to get Z-error
stats, and override region_mask() if the default bounding box is wrong.

The cap: the part of a sphere of radius `radius` above the plane z = 0, whose
base circle has radius `base_radius`. The sphere's centre is therefore at
z = -sqrt(radius^2 - base_radius^2).
"""

import numpy as np

from scan_eval import ReferenceSurface


class SphereCap(ReferenceSurface):
    exact = True
    is_height_field = True
    visible_from = np.array([0.0, 0.0, 1.0])

    def __init__(self, radius: float, base_radius: float):
        if not 0 < base_radius <= radius:
            raise ValueError("need 0 < base_radius <= radius")
        self.R = float(radius)
        self.a = float(base_radius)
        self.d = np.sqrt(self.R**2 - self.a**2)          # centre sits at z = -d
        self.centre = np.array([0.0, 0.0, -self.d])
        self.cos_cap = self.d / self.R                    # polar angle of the rim

    def sample(self, n, seed=0):
        # Uniform by area on a sphere zone: z is uniform between the rim and the pole.
        rng = np.random.default_rng(seed)
        zc = rng.uniform(self.cos_cap, 1.0, n)            # cos(polar angle)
        phi = rng.uniform(0, 2 * np.pi, n)
        s = np.sqrt(1 - zc**2)
        normals = np.stack([s * np.cos(phi), s * np.sin(phi), zc], axis=1)
        return self.centre + self.R * normals, normals

    def closest_points(self, points):
        p = np.asarray(points, dtype=float) - self.centre
        r = np.linalg.norm(p, axis=1)
        direction = p / np.maximum(r, 1e-300)[:, None]
        direction[r < 1e-12] = [0.0, 0.0, 1.0]
        on_cap = direction[:, 2] >= self.cos_cap
        # Outside the cap's cone the nearest point is on the rim circle.
        xy = np.linalg.norm(p[:, :2], axis=1)
        rim_dir = np.zeros_like(p)
        rim_dir[:, 0] = np.where(xy > 1e-12, p[:, 0] / np.maximum(xy, 1e-300), 1.0)
        rim_dir[:, 1] = np.where(xy > 1e-12, p[:, 1] / np.maximum(xy, 1e-300), 0.0)
        rim_normal = rim_dir * (self.a / self.R)
        rim_normal[:, 2] = self.cos_cap
        normals = np.where(on_cap[:, None], direction, rim_normal)
        return self.centre + self.R * normals, normals

    def bounds(self):
        return np.array([-self.a, -self.a, 0.0]), np.array([self.a, self.a, self.R - self.d])

    def height(self, x, y):
        rr = np.asarray(x, float) ** 2 + np.asarray(y, float) ** 2
        z = np.sqrt(np.maximum(self.R**2 - rr, 0)) - self.d
        return np.where(rr <= self.a**2, z, np.nan)

    def region_mask(self, points, margin=0.0):
        p = np.asarray(points, float)
        return np.hypot(p[:, 0], p[:, 1]) <= self.a + margin

    def surface_area(self):
        return 2 * np.pi * self.R * (self.R - self.d)
