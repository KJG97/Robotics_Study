# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Inverse Kinematics UI - Follow Target Cube"""

import numpy as np
import omni.ui as ui
import omni.timeline
from isaacsim.core.api.world import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.util.debug_draw import _debug_draw

from .scenario import InverseKinematicsScenario

GREEN = (0.0, 1.0, 0.0, 0.6)
RED = (1.0, 0.0, 0.0, 0.3)


class InverseKinematicsUI:
    """UI for Inverse Kinematics - Follow Target."""

    TARGET_PRIM_PATH = "/World/IK_Target"

    def __init__(self):
        self._scenario = InverseKinematicsScenario()
        self._timeline = omni.timeline.get_timeline_interface()
        self._window = None
        self._status_label = None
        self._toggle_btn = None
        self._pos_only_btn = None
        self._target = None
        self._physics_sub = None

    # ─────────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ─────────────────────────────────────────────────────────────────────────
    def setup(self, articulation):
        self._scenario.setup(articulation, self._target)

    def teardown(self):
        self._stop_physics_sub()
        self._scenario.teardown()
        if self._window:
            self._window.visible = False
            self._window = None
        self._target = None

    def reset(self):
        self._scenario.stop()
        if self._toggle_btn:
            self._toggle_btn.text = "Start IK [OFF]"

    def on_stage_opened(self):
        self.teardown()

    def _stop_physics_sub(self):
        if self._physics_sub:
            self._physics_sub.unsubscribe()
            self._physics_sub = None

    # ─────────────────────────────────────────────────────────────────────────
    # UI Build
    # ─────────────────────────────────────────────────────────────────────────
    def build_ui(self):
        ui.Button("Inverse Kinematics", height=30, clicked_fn=self._open_window)

    def _open_window(self):
        if not self._scenario.articulation:
            print("[IK] Load robot first!")
            return
        if not self._window:
            self._create_window()
        self._window.visible = True

    def _create_window(self):
        self._window = ui.Window("Inverse Kinematics", width=400, height=380)

        with self._window.frame:
            with ui.VStack(spacing=5, style={"margin": 10}):
                ui.Label("IK: Robot follows target cube", height=22,
                        style={"font_size": 14, "color": 0xFF00D4FF})

                ui.Button("Create Target Cube", height=26, clicked_fn=self._create_target)

                self._toggle_btn = ui.Button(
                    "Start IK [OFF]", height=30, clicked_fn=self._toggle_ik,
                    style={"background_color": 0xFF006600}
                )

                self._pos_only_btn = ui.Button(
                    "Mode: Position + Orientation", height=26,
                    clicked_fn=self._toggle_position_only,
                    style={"background_color": 0xFF0066AA}
                )

                ui.Spacer(height=5)
                ui.Line(height=1, style={"color": 0xFF555555})

                ui.Label("Workspace Visualization", height=18,
                        style={"font_size": 12, "color": 0xFFFFAA00})

                with ui.HStack(height=26, spacing=5):
                    ui.Button("Draw Pos Only", clicked_fn=lambda: self._draw_workspace(True),
                             style={"background_color": 0xFF005500})
                    ui.Button("Draw Pos+Ori", clicked_fn=lambda: self._draw_workspace(False),
                             style={"background_color": 0xFF000055})

                ui.Button("Clear", height=22, clicked_fn=self._clear_workspace,
                         style={"background_color": 0xFF333333})

                ui.Spacer(height=5)
                ui.Line(height=1, style={"color": 0xFF555555})

                self._status_label = ui.Label("Create target cube to start.", word_wrap=True,
                                              style={"font_size": 11, "color": 0xFFCCCCCC})

    # ─────────────────────────────────────────────────────────────────────────
    # Target Management
    # ─────────────────────────────────────────────────────────────────────────
    def _create_target(self):
        stage = get_current_stage()
        if not stage:
            return

        if stage.GetPrimAtPath(self.TARGET_PRIM_PATH).IsValid():
            self._target = VisualCuboid(self.TARGET_PRIM_PATH)
        else:
            try:
                self._target = VisualCuboid(
                    prim_path=self.TARGET_PRIM_PATH, name="ik_target",
                    position=np.array([0.4, 0.0, 0.4]),
                    orientation=np.array([0.0, 1.0, 0.0, 0.0]),
                    size=0.05, color=np.array([1.0, 0.2, 0.2])
                )
            except Exception as e:
                print(f"[IK] Target creation failed: {e}")
                return

        self._scenario.target = self._target
        print("[IK] Target ready")

    # ─────────────────────────────────────────────────────────────────────────
    # IK Control
    # ─────────────────────────────────────────────────────────────────────────
    def _toggle_position_only(self):
        new_val = not self._scenario.position_only
        self._scenario.set_position_only(new_val)
        self._pos_only_btn.text = f"Mode: {'Position Only' if new_val else 'Position + Orientation'}"

    def _toggle_ik(self):
        if not self._scenario.is_ready:
            print("[IK] Create target first!")
            return

        if self._scenario.is_active:
            self._scenario.stop()
            self._stop_physics_sub()
            self._toggle_btn.text = "Start IK [OFF]"
            self._toggle_btn.set_style({"background_color": 0xFF006600})
        else:
            self._scenario.start()
            self._physics_sub = World.instance().add_physics_callback("ik_update", lambda _: self._scenario.update())
            self._timeline.play()
            self._toggle_btn.text = "Stop IK [ON]"
            self._toggle_btn.set_style({"background_color": 0xFF660000})

    # ─────────────────────────────────────────────────────────────────────────
    # Workspace Visualization
    # ─────────────────────────────────────────────────────────────────────────
    def _draw_workspace(self, position_only: bool):
        if not self._scenario.articulation:
            print("[IK] Load robot first!")
            return

        mode = "Pos Only" if position_only else "Pos+Ori"
        print(f"[Workspace] Computing {mode}...")
        if self._status_label:
            self._status_label.text = f"Computing {mode} workspace..."

        result = self._scenario.compute_workspace(position_only=position_only, resolution=15)
        draw = _debug_draw.acquire_debug_draw_interface()

        for points, color, size in [(result["reachable"], GREEN, 60), (result["unreachable"], RED, 40)]:
            if points:
                draw.draw_points(points, [color] * len(points), [size] * len(points))

        if self._status_label:
            self._status_label.text = f"Reachable: {len(result['reachable'])}, Unreachable: {len(result['unreachable'])}"

    def _clear_workspace(self):
        draw = _debug_draw.acquire_debug_draw_interface()
        draw.clear_points()
        draw.clear_lines()
        print("[Workspace] Cleared")
