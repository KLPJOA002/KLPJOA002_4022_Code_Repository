#!/usr/bin/env python3
"""
view_ply_interactive.py — Load a .ply and display it with an interactive
settings panel: switch between point cloud / mesh, choose point coloring
(RGB / height colormap / solid gray), toggle shading, and crop by depth
(Z-axis range) with live sliders.

Usage:
    python3 view_ply_interactive.py path/to/file.ply

If you hit a Wayland/GLFW window-creation error, either run:
    GDK_BACKEND=x11 python3 view_ply_interactive.py file.ply
or log into an "Ubuntu on Xorg" session instead of Wayland.
"""

import sys
from pathlib import Path

try:
    import open3d as o3d
    import open3d.visualization.gui as gui
    import open3d.visualization.rendering as rendering
except ImportError:
    sys.exit(
        "open3d is not installed.\n"
        "Install it with:\n"
        "    pip install open3d --break-system-packages"
    )

import numpy as np


# ---------------------------------------------------------------- colormaps

def _turbo_colormap(norm: np.ndarray) -> np.ndarray:
    """Approximate the 'turbo' colormap without needing matplotlib.
    norm: (N,) array in [0, 1]. Returns (N, 3) RGB in [0, 1]."""
    # A handful of turbo control points (position, R, G, B), linearly
    # interpolated. Good enough for depth/height visualization purposes.
    stops = np.array([
        [0.00, 0.19, 0.07, 0.23],
        [0.13, 0.27, 0.41, 0.85],
        [0.25, 0.16, 0.71, 0.96],
        [0.38, 0.13, 0.91, 0.61],
        [0.50, 0.58, 0.98, 0.24],
        [0.63, 0.96, 0.80, 0.13],
        [0.75, 0.98, 0.50, 0.13],
        [0.88, 0.87, 0.20, 0.09],
        [1.00, 0.48, 0.02, 0.01],
    ])
    positions = stops[:, 0]
    colors = stops[:, 1:]
    r = np.interp(norm, positions, colors[:, 0])
    g = np.interp(norm, positions, colors[:, 1])
    b = np.interp(norm, positions, colors[:, 2])
    return np.stack([r, g, b], axis=1)


def height_colors(values: np.ndarray, vmin: float, vmax: float) -> np.ndarray:
    if vmax - vmin < 1e-9:
        norm = np.zeros_like(values)
    else:
        norm = np.clip((values - vmin) / (vmax - vmin), 0.0, 1.0)
    try:
        import matplotlib.cm as cm
        return cm.get_cmap("turbo")(norm)[:, :3]
    except ImportError:
        return _turbo_colormap(norm)


class PlyViewer:
    GEOM_NAME = "geometry"
    COLOR_RGB = "RGB (file colors)"
    COLOR_HEIGHT = "Height colormap (Z)"
    COLOR_GRAY = "Solid gray"

    def __init__(self, ply_path: Path):
        self.ply_path = ply_path

        # --- Load once, keep both representations available ---
        mesh = o3d.io.read_triangle_mesh(str(ply_path))
        self.has_mesh = mesh.has_triangles()
        if self.has_mesh:
            if not mesh.has_vertex_normals():
                mesh.compute_vertex_normals()
            self.mesh_orig = mesh
            pcd = o3d.geometry.PointCloud()
            pcd.points = mesh.vertices
            if mesh.has_vertex_colors():
                pcd.colors = mesh.vertex_colors
            if mesh.has_vertex_normals():
                pcd.normals = mesh.vertex_normals
            self.pcd_orig = pcd
        else:
            self.mesh_orig = None
            pcd = o3d.io.read_point_cloud(str(ply_path))
            if len(pcd.points) == 0:
                sys.exit(f"Error: '{ply_path}' has no readable points or triangles.")
            if not pcd.has_normals():
                pcd.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(
                        radius=0.1, max_nn=30
                    )
                )
            self.pcd_orig = pcd

        self.has_color = self.pcd_orig.has_colors()
        self.has_normals = self.pcd_orig.has_normals()
        pts = np.asarray(self.pcd_orig.points)
        self.bbox_min = pts.min(axis=0)
        self.bbox_max = pts.max(axis=0)
        self.z_min, self.z_max = float(self.bbox_min[2]), float(self.bbox_max[2])
        if self.z_min == self.z_max:  # degenerate flat cloud, avoid zero-range slider
            self.z_max = self.z_min + 1e-3

        print(f"Loaded '{ply_path.name}': "
              f"{len(self.pcd_orig.points)} points"
              + (f", {len(self.mesh_orig.triangles)} triangles" if self.has_mesh else "")
              + f" | colors: {self.has_color} | normals: {self.has_normals}")

        # --- State ---
        self.display_mode = "mesh" if self.has_mesh else "pointcloud"
        self.color_mode = self.COLOR_RGB if self.has_color else self.COLOR_HEIGHT
        self.shading_on = True
        self.crop_lo = self.z_min
        self.crop_hi = self.z_max

        self._build_ui()
        self._update_geometry()

    # ------------------------------------------------------------------ UI

    def _build_ui(self):
        self.window = gui.Application.instance.create_window(
            f"PLY Viewer — {self.ply_path.name}", 1280, 800
        )
        w = self.window
        em = w.theme.font_size

        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(w.renderer)
        self.scene.scene.set_background([0.05, 0.05, 0.05, 1.0])

        self.mat_point_unlit = rendering.MaterialRecord()
        self.mat_point_unlit.shader = "defaultUnlit"
        self.mat_point_unlit.point_size = 3.0

        self.mat_point_lit = rendering.MaterialRecord()
        self.mat_point_lit.shader = "defaultLit"
        self.mat_point_lit.point_size = 3.0

        self.mat_mesh = rendering.MaterialRecord()
        self.mat_mesh.shader = "defaultLit"

        panel = gui.Vert(0.5 * em, gui.Margins(0.5 * em, 0.5 * em, 0.5 * em, 0.5 * em))

        panel.add_child(gui.Label("Display mode"))
        self.mode_combo = gui.Combobox()
        self.mode_combo.add_item("Point cloud")
        if self.has_mesh:
            self.mode_combo.add_item("Mesh")
        self.mode_combo.selected_text = "Mesh" if self.display_mode == "mesh" else "Point cloud"
        self.mode_combo.set_on_selection_changed(self._on_mode_changed)
        panel.add_child(self.mode_combo)

        panel.add_fixed(1.0 * em)
        panel.add_child(gui.Label("Coloring"))
        self.color_combo = gui.Combobox()
        self.color_combo.add_item(self.COLOR_RGB)
        self.color_combo.add_item(self.COLOR_HEIGHT)
        self.color_combo.add_item(self.COLOR_GRAY)
        self.color_combo.selected_text = self.color_mode
        if not self.has_color:
            # Still show the item but users can't meaningfully pick it;
            # selecting it will just fall back to gray in _update_geometry.
            pass
        self.color_combo.set_on_selection_changed(self._on_color_mode_changed)
        panel.add_child(self.color_combo)

        self.shading_check = gui.Checkbox("Shading (lit, uses normals)")
        self.shading_check.checked = self.shading_on
        self.shading_check.enabled = self.has_normals
        self.shading_check.set_on_checked(self._on_shading_changed)
        panel.add_child(self.shading_check)

        panel.add_fixed(1.0 * em)
        panel.add_child(gui.Label(f"Depth crop (Z: {self.z_min:.3f} to {self.z_max:.3f})"))

        panel.add_child(gui.Label("Min Z"))
        self.slider_lo = gui.Slider(gui.Slider.DOUBLE)
        self.slider_lo.set_limits(self.z_min, self.z_max)
        self.slider_lo.double_value = self.z_min
        self.slider_lo.set_on_value_changed(self._on_lo_changed)
        panel.add_child(self.slider_lo)

        panel.add_child(gui.Label("Max Z"))
        self.slider_hi = gui.Slider(gui.Slider.DOUBLE)
        self.slider_hi.set_limits(self.z_min, self.z_max)
        self.slider_hi.double_value = self.z_max
        self.slider_hi.set_on_value_changed(self._on_hi_changed)
        panel.add_child(self.slider_hi)

        panel.add_fixed(0.5 * em)
        reset_btn = gui.Button("Reset crop")
        reset_btn.set_on_clicked(self._on_reset_crop)
        panel.add_child(reset_btn)

        self.info_label = gui.Label("")
        panel.add_fixed(1.0 * em)
        panel.add_child(self.info_label)

        w.add_child(self.scene)
        w.add_child(panel)

        panel_width = 22 * em

        def on_layout(theme):
            r = w.content_rect
            panel.frame = gui.Rect(r.get_right() - panel_width, r.y, panel_width, r.height)
            self.scene.frame = gui.Rect(r.x, r.y, r.width - panel_width, r.height)

        w.set_on_layout(on_layout)

    # -------------------------------------------------------------- events

    def _on_mode_changed(self, text, index):
        self.display_mode = "mesh" if text == "Mesh" else "pointcloud"
        self._update_geometry()

    def _on_color_mode_changed(self, text, index):
        self.color_mode = text
        self._update_geometry()

    def _on_shading_changed(self, checked):
        self.shading_on = checked
        self._update_geometry()

    def _on_lo_changed(self, value):
        self.crop_lo = min(value, self.crop_hi)
        self._update_geometry()

    def _on_hi_changed(self, value):
        self.crop_hi = max(value, self.crop_lo)
        self._update_geometry()

    def _on_reset_crop(self):
        self.crop_lo, self.crop_hi = self.z_min, self.z_max
        self.slider_lo.double_value = self.z_min
        self.slider_hi.double_value = self.z_max
        self._update_geometry()

    # ------------------------------------------------------------ geometry

    def _crop_bbox(self):
        lo = np.array([self.bbox_min[0], self.bbox_min[1], self.crop_lo]) - 1e-6
        hi = np.array([self.bbox_max[0], self.bbox_max[1], self.crop_hi]) + 1e-6
        return o3d.geometry.AxisAlignedBoundingBox(lo, hi)

    def _apply_coloring(self, geom, is_mesh: bool):
        """Set vertex/point colors on geom according to self.color_mode.
        Uses the GLOBAL z-range for the height colormap so colors stay
        consistent as you crop, rather than rescaling to the crop window."""
        n = len(geom.vertices) if is_mesh else len(geom.points)
        if n == 0:
            return

        mode = self.color_mode
        if mode == self.COLOR_RGB and not self.has_color:
            mode = self.COLOR_GRAY  # fall back gracefully

        if mode == self.COLOR_RGB:
            return  # colors already present from crop of original geometry

        pts = np.asarray(geom.vertices if is_mesh else geom.points)

        if mode == self.COLOR_HEIGHT:
            colors = height_colors(pts[:, 2], self.crop_lo, self.crop_hi)
        else:  # COLOR_GRAY
            colors = np.tile([0.7, 0.7, 0.7], (n, 1))

        if is_mesh:
            geom.vertex_colors = o3d.utility.Vector3dVector(colors)
        else:
            geom.colors = o3d.utility.Vector3dVector(colors)

    def _update_geometry(self):
        bbox = self._crop_bbox()

        if self.display_mode == "mesh" and self.has_mesh:
            geom = self.mesh_orig.crop(bbox)
            self._apply_coloring(geom, is_mesh=True)
            material = self.mat_mesh
            n_shown = len(geom.triangles)
            unit = "triangles"
        else:
            geom = self.pcd_orig.crop(bbox)
            self._apply_coloring(geom, is_mesh=False)
            use_lit = self.shading_on and self.has_normals
            material = self.mat_point_lit if use_lit else self.mat_point_unlit
            n_shown = len(geom.points)
            unit = "points"

        self.scene.scene.clear_geometry()
        self.scene.scene.add_geometry(self.GEOM_NAME, geom, material)

        bounds = geom.get_axis_aligned_bounding_box() if n_shown > 0 else bbox
        self.scene.setup_camera(60, bounds, bounds.get_center())

        self.info_label.text = f"Showing {n_shown} {unit}"


def main():
    if len(sys.argv) != 2:
        sys.exit(f"Usage: python3 {Path(sys.argv[0]).name} path/to/file.ply")

    ply_path = Path(sys.argv[1])
    if not ply_path.exists():
        sys.exit(f"Error: file not found: {ply_path}")

    gui.Application.instance.initialize()
    PlyViewer(ply_path)
    gui.Application.instance.run()


if __name__ == "__main__":
    main()