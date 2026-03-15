# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Assignment2 UI: ALLEX Robot Jacobian - Load ALLEX model
"""

import os
from typing import Optional

import numpy as np
import omni.timeline
import omni.ui as ui
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage, get_current_stage
from isaacsim.examples.extension.core_connectors import LoadButton
from isaacsim.gui.components.ui_utils import get_style
from pxr import Sdf, Usd, UsdLux

from .scenario import Assignment2Scenario, POSES

# Extension root directory (Robotics_Study/)
_EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def _stage_time_code(stage) -> "Usd.TimeCode":
    """Current stage time code for consistent transform sampling."""
    return Usd.TimeCode(
        omni.timeline.get_timeline_interface().get_current_time() * stage.GetTimeCodesPerSecond()
    )


def _transform_item_to_matrix(item) -> Optional[np.ndarray]:
    """Convert (label, rows) from get_link_transforms to 4x4 numpy matrix."""
    _, rows = item
    if not rows or len(rows) != 4:
        return None
    return np.array(rows, dtype=float)


class Assignment2UI:
    """Assignment2 UI Builder - LOAD ALLEX for Jacobian study."""

    ROBOT_USD = os.path.join(_EXT_ROOT, "asset", "ALLEX.usd")
    ROBOT_PRIM = "/ALLEX"

    def __init__(self):
        self._scenario = Assignment2Scenario()
        self._articulation = None
        self._status_label = None
        self.wrapped_ui_elements = []

    def build_ui(self, window):
        """Build UI for Assignment2 window."""
        with window.frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                ui.Label(
                    "Assignment2: ALLEX Jacobian",
                    alignment=ui.Alignment.CENTER,
                    height=30,
                )
                ui.Spacer(height=5)

                load_btn = LoadButton(
                    "Assignment2 Load",
                    "LOAD ALLEX",
                    setup_scene_fn=self._setup_scene,
                    setup_post_load_fn=self._post_load,
                )
                load_btn.set_world_settings(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
                self.wrapped_ui_elements.append(load_btn)

                with ui.HStack(spacing=5, height=28):
                    self._get_tf_btn = ui.Button("Get Transform", height=28, clicked_fn=self._on_get_transform)
                    self._get_jacobian_btn = ui.Button("Get Jacobian", height=28, clicked_fn=self._on_get_jacobian)

                ui.Spacer(height=5)
                ui.Label("Pose (then Draw ellipsoid for current pose):", height=20)
                with ui.HStack(spacing=5, height=28):
                    ui.Button("Pose1", height=28, clicked_fn=lambda: self._on_pose_clicked("pose1"))
                    ui.Button("Pose2", height=28, clicked_fn=lambda: self._on_pose_clicked("pose2"))
                    ui.Button("Pose3", height=28, clicked_fn=lambda: self._on_pose_clicked("pose3"))
                with ui.HStack(spacing=5, height=28):
                    ui.Button("Draw Velocity Ellipsoid", height=28, clicked_fn=lambda: self._on_draw_ellipsoid("velocity"))
                    ui.Button("Draw Torque Ellipsoid", height=28, clicked_fn=lambda: self._on_draw_ellipsoid("force"))

                ui.Spacer(height=10)
                ui.Label("Status / T matrices:", height=20)
                with ui.ScrollingFrame(height=280):
                    self._status_label = ui.Label(
                        "Click LOAD ALLEX to load the robot.",
                        alignment=ui.Alignment.LEFT_TOP,
                        word_wrap=True,
                        style={"font_size": 12, "color": 0xFFCCCCCC},
                    )

    def _setup_scene(self):
        """Setup scene - load ALLEX robot."""
        create_new_stage()
        UsdLux.DomeLight.Define(get_current_stage(), Sdf.Path("/World/DomeLight")).CreateIntensityAttr(1000)

        add_reference_to_stage(self.ROBOT_USD, self.ROBOT_PRIM)
        self._articulation = SingleArticulation(self.ROBOT_PRIM)
        World.instance().scene.add(self._articulation)
        print("[Assignment2] ALLEX robot loaded")

    def _post_load(self):
        """Post-load callback."""
        World.instance().get_physics_context().set_gravity(value=0.0)
        print("[Assignment2] Gravity: 0")

        if self._articulation:
            self._scenario.setup(self._articulation)
            if self._status_label:
                self._status_label.text = "Robot loaded. Click 'Get Transform' to show 4x4 T matrices."

    def _on_get_transform(self):
        """Get world T, convert to link-to-link (4x4x9), print to terminal."""
        if not self._articulation:
            if self._status_label:
                self._status_label.text = "[ERROR] Load ALLEX first, then click Get Transform."
            return
        stage = get_current_stage()
        if not stage:
            if self._status_label:
                self._status_label.text = "[ERROR] No stage."
            return
        world_transforms = self._scenario.get_link_transforms(stage, _stage_time_code(stage))
        T_left, T_right = self._scenario.world_to_link_to_link(world_transforms)
        lines = [
            "=== Link-to-link T (4x4x9) for J_SerialLink ===",
            "  T(:,:,1)=bTo, T(:,:,2)=oT1, ..., T(:,:,8)=6T7, T(:,:,9)=7Tt",
            "",
        ]
        if T_left is None or T_right is None:
            lines.append("[ERROR] Could not build link-to-link T (missing prims?).")
            print("\n".join(lines))
            if self._status_label:
                self._status_label.text = "Link-to-link T failed (see terminal)."
            return
        names = ["bTo", "oT1", "1T2", "2T3", "3T4", "4T5", "5T6", "6T7", "7Tt"]
        for arm_name, T in [("Left arm", T_left), ("Right arm", T_right)]:
            lines.append(f"--- {arm_name} ---")
            for k in range(9):
                lines.append(f"  T(:,:,{k + 1}) = {names[k]}")
                for r in T[:, :, k]:
                    lines.append("    [" + ", ".join(f"{x:12.6f}" for x in r) + "]")
                lines.append("")
            lines.append("")
        print("\n".join(lines))
        if self._status_label:
            self._status_label.text = "Link-to-link T (4x4x9) printed to terminal."

    def _on_get_jacobian(self):
        """Compute 6x7 Jacobian from current pose and print to terminal."""
        if not self._articulation:
            if self._status_label:
                self._status_label.text = "[ERROR] Load ALLEX first, then click Get Jacobian."
            return
        stage = get_current_stage()
        if not stage:
            if self._status_label:
                self._status_label.text = "[ERROR] No stage."
            return
        world_transforms = self._scenario.get_link_transforms(stage, _stage_time_code(stage))
        T_left, T_right = self._scenario.world_to_link_to_link(world_transforms)
        if T_left is None or T_right is None:
            print("[Assignment2] Could not build link-to-link T (missing prims?).")
            if self._status_label:
                self._status_label.text = "Get Jacobian failed (see terminal)."
            return
        J_left = Assignment2Scenario.jacobian_serial_link(T_left)
        J_right = Assignment2Scenario.jacobian_serial_link(T_right)
        lines = [
            "=== J (6x7 Jacobian) for J_SerialLink(T) ===",
            "  [v; w] = J * qd",
            "",
        ]
        for arm_name, J in [("Left arm", J_left), ("Right arm", J_right)]:
            lines.append(f"--- {arm_name} J ---")
            for i in range(6):
                lines.append("  [" + ", ".join(f"{J[i, j]:12.6f}" for j in range(7)) + "]")
            lines.append("")
        print("\n".join(lines))
        if self._status_label:
            self._status_label.text = "6x7 Jacobian printed to terminal."

    def _on_pose_clicked(self, pose_name: str):
        """Apply pose1/pose2/pose3 to the robot."""
        if not self._articulation:
            if self._status_label:
                self._status_label.text = "[ERROR] Load ALLEX first."
            return
        if self._scenario.apply_pose(self._articulation, pose_name):
            desc = POSES.get(pose_name, {}).get("description", pose_name)
            if self._status_label:
                self._status_label.text = f"Applied {pose_name}: {desc}. Click Draw Ellipsoid to visualize."
        else:
            if self._status_label:
                self._status_label.text = f"[ERROR] Failed to apply {pose_name}."

    def _on_draw_ellipsoid(self, which: str):
        """Draw velocity (green) or force/torque (red) manipulability ellipsoid at Hand Base. which: 'velocity' | 'force'."""
        if not self._articulation:
            if self._status_label:
                self._status_label.text = "[ERROR] Load ALLEX first, then Draw Ellipsoid."
            return
        stage = get_current_stage()
        if not stage:
            if self._status_label:
                self._status_label.text = "[ERROR] No stage."
            return
        world_transforms = self._scenario.get_link_transforms(stage, _stage_time_code(stage))
        T_left, T_right = self._scenario.world_to_link_to_link(world_transforms)
        if T_left is None or T_right is None:
            if self._status_label:
                self._status_label.text = "Draw Ellipsoid failed (missing prims?)."
            return
        W_origin = _transform_item_to_matrix(world_transforms[0])
        W_L_tool = _transform_item_to_matrix(world_transforms[8])
        W_R_tool = _transform_item_to_matrix(world_transforms[16])
        if W_origin is not None and W_L_tool is not None and W_R_tool is not None:
            origin_T_left = np.linalg.inv(W_origin) @ W_L_tool
            origin_T_right = np.linalg.inv(W_origin) @ W_R_tool
            left_ell = Assignment2Scenario.ellipsoid_axes_data(T_left)
            right_ell = Assignment2Scenario.ellipsoid_axes_data(T_right)
            lines = [
                "=== Frames in Origin_Body ===",
                "",
            ]
            for name, origin_T_hand, ell_data in [
                ("Left Hand", origin_T_left, left_ell),
                ("Right Hand", origin_T_right, right_ell),
            ]:
                pos = origin_T_hand[3, :3]
                R = origin_T_hand[:3, :3]
                lines.append(f"--- {name} (Hand Base) ---")
                lines.append("  position: [" + ", ".join(f"{x:.6f}" for x in pos) + "]")
                lines.append("  R (3x3):")
                for r in R:
                    lines.append("    [" + ", ".join(f"{x:.6f}" for x in r) + "]")
                if ell_data:
                    p_ee = ell_data["p_ee"]
                    lines.append("  Ellipsoid center: [" + ", ".join(f"{x:.6f}" for x in p_ee) + "]")
                    for kind, axes_key, len_key in [
                        ("Velocity", "vel_axes", "vel_lengths"),
                        ("Force", "force_axes", "force_lengths"),
                    ]:
                        lines.append(f"  {kind} axes (cols) & lengths:")
                        for i in range(3):
                            ax = ", ".join(f"{ell_data[axes_key][j, i]:.6f}" for j in range(3))
                            lines.append(f"    axis{i + 1} {ax}  len={ell_data[len_key][i]:.6f}")
                lines.append("")
            print("\n".join(lines))
        try:
            from isaacsim.util.debug_draw import _debug_draw

            draw = _debug_draw.acquire_debug_draw_interface()
            draw.clear_points()
            draw.clear_lines()
            if W_origin is not None and W_L_tool is not None and W_R_tool is not None:
                R_origin = W_origin[:3, :3]
                fill_color = (0.0, 0.8, 0.2, 0.4) if which == "velocity" else (0.9, 0.2, 0.2, 0.4)
                point_size = 20.0
                for T, center_w in [(T_left, W_L_tool[3, :3]), (T_right, W_R_tool[3, :3])]:
                    segments = Assignment2Scenario.ellipsoid_principal_axes_from_T(T, which=which)
                    for start, end, color in segments:
                        vec_body = np.array(end) - np.array(start)
                        start_w = center_w.tolist()
                        end_w = (center_w + R_origin @ vec_body).tolist()
                        draw.draw_lines([start_w], [end_w], [color], [6.0])
                    p_ee_body = np.array(segments[0][0]) if segments else None
                    if p_ee_body is not None:
                        surface_points_body = Assignment2Scenario.ellipsoid_surface_points_from_T(T, which=which)
                        if surface_points_body:
                            points_world = [
                                (center_w + R_origin @ (np.array(pt) - p_ee_body)).tolist()
                                for pt in surface_points_body
                            ]
                            draw.draw_points(points_world, [fill_color] * len(points_world), [point_size] * len(points_world))
            else:
                for T in (T_left, T_right):
                    segments = Assignment2Scenario.ellipsoid_principal_axes_from_T(T, which=which)
                    for start, end, color in segments:
                        draw.draw_lines([start], [end], [color], [6.0])
                    surface_points_body = Assignment2Scenario.ellipsoid_surface_points_from_T(T, which=which)
                    if surface_points_body:
                        fill_color = (0.0, 0.8, 0.2, 0.4) if which == "velocity" else (0.9, 0.2, 0.2, 0.4)
                        draw.draw_points(surface_points_body, [fill_color] * len(surface_points_body), [4.0] * len(surface_points_body))
            label = "Velocity" if which == "velocity" else "Torque"
            if self._status_label:
                self._status_label.text = f"{label} ellipsoid drawn at Hand Base."
        except Exception as e:
            if self._status_label:
                self._status_label.text = f"Draw failed: {e}"

    def cleanup(self):
        """Cleanup UI elements."""
        self._scenario.teardown()
        self._articulation = None
        for elem in self.wrapped_ui_elements:
            elem.cleanup()
        self.wrapped_ui_elements.clear()
