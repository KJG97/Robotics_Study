# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Forward Kinematics UI - Joint Slider Control"""

import numpy as np
import omni.ui as ui
from .scenario import ForwardKinematicsScenario

SLIDER_STYLE = {
    "draw_mode": ui.SliderDrawMode.FILLED,
    "background_color": 0xFF2A2A2A,
    "secondary_color": 0xFF0088CC,
    "border_radius": 3,
}


class ForwardKinematicsUI:
    """UI for Forward Kinematics joint slider control."""

    def __init__(self):
        self._scenario = ForwardKinematicsScenario()
        self._window = None
        self._sliders = []
        self._ee_label = None

    # ─────────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ─────────────────────────────────────────────────────────────────────────
    def setup(self, articulation):
        self._scenario.setup(articulation)

    def teardown(self):
        self._scenario.teardown()
        self._close_window()

    def reset(self):
        if self._scenario.articulation:
            self._scenario.articulation.set_joint_positions(np.zeros(self._scenario.num_dof))
        self._set_all_sliders(0.0)

    def on_stage_opened(self):
        self._scenario.teardown()
        self._close_window()

    def _close_window(self):
        if self._window:
            self._window.visible = False
            self._window = None
        self._sliders.clear()

    # ─────────────────────────────────────────────────────────────────────────
    # UI Build
    # ─────────────────────────────────────────────────────────────────────────
    def build_ui(self):
        ui.Button("Forward Kinematics", height=30, clicked_fn=self._open_window)

    def _open_window(self):
        if not self._scenario.articulation:
            print("[FK] Load robot first!")
            return

        if not self._window:
            self._create_window()
        
        self._window.visible = True
        self._update_fk()

    def _create_window(self):
        self._window = ui.Window("Forward Kinematics", width=400, height=420)
        self._sliders.clear()

        with self._window.frame:
            with ui.VStack(spacing=2, style={"margin": 8}):
                ui.Button("Reset to Zero", height=24, clicked_fn=self._on_reset)
                
                # Joint sliders
                for i, name in enumerate(self._scenario.dof_names):
                    lo, hi = self._scenario.joint_limits[i]
                    self._create_slider(i, name, np.degrees(lo), np.degrees(hi))

                ui.Spacer(height=6)
                ui.Line(height=1, style={"color": 0xFF555555})
                ui.Spacer(height=4)
                
                self._ee_label = ui.Label("", word_wrap=True, style={"font_size": 14})

    def _create_slider(self, idx: int, name: str, lo_deg: float, hi_deg: float):
        with ui.HStack(height=22, spacing=0):
            ui.Label(name, width=35, style={"font_size": 14})
            slider = ui.FloatSlider(min=lo_deg, max=hi_deg, step=0.5, style=SLIDER_STYLE)
            slider.model.set_value(0.0)
            slider.model.add_value_changed_fn(lambda m, i=idx: self._on_slider_change(i, m.as_float))
            self._sliders.append(slider)

    # ─────────────────────────────────────────────────────────────────────────
    # Event Handlers
    # ─────────────────────────────────────────────────────────────────────────
    def _on_slider_change(self, idx: int, deg_val: float):
        art = self._scenario.articulation
        if not art:
            return
        positions = art.get_joint_positions()
        if positions is not None:
            positions[idx] = np.radians(deg_val)
            art.set_joint_positions(positions)
        self._update_fk()

    def _on_reset(self):
        if self._scenario.articulation:
            self._scenario.articulation.set_joint_positions(np.zeros(self._scenario.num_dof))
        self._set_all_sliders(0.0)
        self._update_fk()

    def _set_all_sliders(self, value: float):
        for slider in self._sliders:
            slider.model.set_value(value)

    # ─────────────────────────────────────────────────────────────────────────
    # FK Display
    # ─────────────────────────────────────────────────────────────────────────
    def _update_fk(self):
        if not self._ee_label:
            return

        if not self._scenario.is_ready:
            self._ee_label.text = "Solver not ready"
            return

        result = self._scenario.compute_fk()
        if not result:
            self._ee_label.text = "FK failed"
            return

        p = result["position"]
        r, pt, y = [np.degrees(v) for v in result["rpy"]]

        self._ee_label.text = (
            f"Position (m):\n"
            f"  X: {p[0]:.5f}   Y: {p[1]:.5f}   Z: {p[2]:.5f}\n\n"
            f"Orientation (RPY, deg):\n"
            f"  Roll: {r:>8.3f}°   Pitch: {pt:>8.3f}°   Yaw: {y:>8.3f}°"
        )
