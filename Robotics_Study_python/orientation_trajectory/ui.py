# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Orientation Trajectory UI - Euler Angles & Angle-Axis Methods

Two trajectory generation methods:
1. Euler (ZYZ): tr2eul -> jtraj -> eul2r
2. Angle-Axis: tr2angvec -> jtraj -> angvec2r (shortest path, no singularity)
"""

import numpy as np
import omni.ui as ui
import omni.timeline
from isaacsim.core.api.world import World

from .scenario import OrientationTrajectoryScenario


class OrientationTrajectoryUI:
    """UI for orientation trajectory using Euler angles and Angle-Axis."""

    def __init__(self):
        self._scenario = OrientationTrajectoryScenario()
        self._timeline = omni.timeline.get_timeline_interface()
        self._window = None
        self._physics_sub = None
        
        # UI elements (initialized in _create_window)
        self._status_label = None
        self._ui_elements = {}  # Store all UI elements by key

    def setup(self, articulation):
        self._scenario.setup(articulation)

    def teardown(self):
        self._scenario.teardown()
        self._remove_physics_callback()
        if self._window:
            self._window.visible = False
            self._window = None

    def reset(self):
        self._scenario.stop_execution()
        self._remove_physics_callback()
        for key in ["euler_execute_btn", "aa_execute_btn"]:
            btn = self._ui_elements.get(key)
            if btn:
                btn.text = "Execute"
                btn.set_style({"background_color": 0xFF006600})
        self._update_status("Ready")

    def on_stage_opened(self):
        self.teardown()

    def _remove_physics_callback(self):
        if self._physics_sub:
            self._physics_sub.unsubscribe()
            self._physics_sub = None
        try:
            world = World.instance()
            if world and world.physics_callback_exists("orientation_traj_update"):
                world.remove_physics_callback("orientation_traj_update")
        except Exception:
            pass

    def build_ui(self):
        ui.Button("Orientation Trajectory", height=30, clicked_fn=self._open_window)

    def _open_window(self):
        if not self._scenario.is_ready:
            print("[OrientationTraj] Load robot first!")
            return
        if not self._window:
            self._create_window()
        self._window.visible = True

    def _create_window(self):
        self._window = ui.Window("Orientation Trajectory", width=430, height=340)
        
        # Styles
        btn = {"border_radius": 3, "margin": 1, "font_size": 14,
               ":hovered": {"background_color": 0xFF0088AA},
               ":pressed": {"background_color": 0xFF004455}}
        label = {"font_size": 14}
        field = {"background_color": 0xFF111111, "font_size": 14}
        info = {"font_size": 14, "color": 0xFF888888}
        
        with self._window.frame:
            with ui.VStack(spacing=3, style={"margin": 5}):
                self._status_label = ui.Label("Ready", height=18, style={"font_size": 14, "color": 0xFFCCCCCC})
                
                # ═══════════════════════════════════════════════════════════════
                # Euler Angle (ZYZ) Frame
                # ═══════════════════════════════════════════════════════════════
                with ui.CollapsableFrame("RPY - Euler Angle (ZYZ)", collapsed=False):
                    with ui.VStack(spacing=2, style={"margin": 3}):
                        # Row 1: Ri, Rf
                        with ui.HStack(height=22, spacing=2):
                            ui.Label("Ri: rotx(", width=50, style=label)
                            self._ui_elements["euler_ri_angle"] = ui.FloatField(width=40, style=field)
                            self._ui_elements["euler_ri_angle"].model.set_value(22.5)
                            ui.Label(")", width=8, style=label)
                            ui.Spacer(width=8)
                            ui.Label("Rf: rotx(", width=50, style=label)
                            self._ui_elements["euler_rf_angle"] = ui.FloatField(width=40, style=field)
                            self._ui_elements["euler_rf_angle"].model.set_value(-22.5)
                            ui.Label(")", width=8, style=label)
                            ui.Spacer()
                        
                        # Row 2: Euler info + tf
                        with ui.HStack(height=22, spacing=2):
                            self._ui_elements["euler_i_label"] = ui.Label("Ei=(0,0,0)", width=100, style={"font_size": 14, "color": 0xFF88FF88})
                            self._ui_elements["euler_f_label"] = ui.Label("Ef=(0,45,0)", width=100, style={"font_size": 14, "color": 0xFFFF8888})
                            ui.Label("tf:", width=16, style=label)
                            self._ui_elements["euler_tf"] = ui.FloatField(width=40, style=field)
                            self._ui_elements["euler_tf"].model.set_value(5.0)
                            ui.Spacer()
                        
                        # Row 3: Buttons
                        with ui.HStack(height=26, spacing=2):
                            ui.Button("Spawn", width=55, height=24, clicked_fn=lambda: self._on_spawn("euler"),
                                      style={**btn, "background_color": 0xFF006688})
                            ui.Button("Generate", width=65, height=24, clicked_fn=lambda: self._on_generate("euler"),
                                      style={**btn, "background_color": 0xFF005588})
                            self._ui_elements["euler_execute_btn"] = ui.Button("Execute", width=65, height=24,
                                clicked_fn=lambda: self._on_execute("euler"), style={**btn, "background_color": 0xFF006600})
                            ui.Button("Clear", width=45, height=24, clicked_fn=self._scenario.clear_debug_draw,
                                      style={**btn, "background_color": 0xFF444444})
                            ui.Spacer()
                        
                        ui.Label("R(t)=eul2r(phi,theta,psi) | Traj: X(R) Y(G) Z(B)", height=16, style=info)
                
                # ═══════════════════════════════════════════════════════════════
                # Angle-Axis Frame
                # ═══════════════════════════════════════════════════════════════
                with ui.CollapsableFrame("Angle-Axis (Shortest Path)", collapsed=False):
                    with ui.VStack(spacing=2, style={"margin": 3}):
                        # Row 1: Ri, Rf
                        with ui.HStack(height=22, spacing=2):
                            ui.Label("Ri: rotx(", width=50, style=label)
                            self._ui_elements["aa_ri_angle"] = ui.FloatField(width=40, style=field)
                            self._ui_elements["aa_ri_angle"].model.set_value(22.5)
                            ui.Label(")", width=8, style=label)
                            ui.Spacer(width=8)
                            ui.Label("Rf: rotx(", width=50, style=label)
                            self._ui_elements["aa_rf_angle"] = ui.FloatField(width=40, style=field)
                            self._ui_elements["aa_rf_angle"].model.set_value(-22.5)
                            ui.Label(")", width=8, style=label)
                            ui.Spacer()
                        
                        # Row 2: Angle-Axis info + tf
                        with ui.HStack(height=22, spacing=2):
                            self._ui_elements["aa_angle_label"] = ui.Label("theta=- deg", width=95, style={"font_size": 14, "color": 0xFF88FFFF})
                            self._ui_elements["aa_axis_label"] = ui.Label("v=(-,-,-)", width=105, style={"font_size": 14, "color": 0xFFFFFF88})
                            ui.Label("tf:", width=16, style=label)
                            self._ui_elements["aa_tf"] = ui.FloatField(width=40, style=field)
                            self._ui_elements["aa_tf"].model.set_value(5.0)
                            ui.Spacer()
                        
                        # Row 3: Buttons
                        with ui.HStack(height=26, spacing=2):
                            ui.Button("Spawn", width=55, height=24, clicked_fn=lambda: self._on_spawn("aa"),
                                      style={**btn, "background_color": 0xFF006688})
                            ui.Button("Generate", width=65, height=24, clicked_fn=lambda: self._on_generate("aa"),
                                      style={**btn, "background_color": 0xFF005588})
                            self._ui_elements["aa_execute_btn"] = ui.Button("Execute", width=65, height=24,
                                clicked_fn=lambda: self._on_execute("aa"), style={**btn, "background_color": 0xFF006600})
                            ui.Button("Clear", width=45, height=24, clicked_fn=self._scenario.clear_debug_draw,
                                      style={**btn, "background_color": 0xFF444444})
                            ui.Spacer()
                        
                        ui.Label("R(t)=Ri*angvec2r(theta(t),v) | No singularity", height=16, style=info)

    # ─────────────────────────────────────────────────────────────────────────
    # Event Handlers
    # ─────────────────────────────────────────────────────────────────────────
    def _get_ui_values(self, mode: str) -> tuple:
        """Get UI values for given mode (euler/aa)."""
        prefix = "euler" if mode == "euler" else "aa"
        ri_angle = self._ui_elements[f"{prefix}_ri_angle"].model.as_float
        rf_angle = self._ui_elements[f"{prefix}_rf_angle"].model.as_float
        tf = self._ui_elements[f"{prefix}_tf"].model.as_float
        return ri_angle, rf_angle, tf

    def _on_spawn(self, mode: str):
        ri_angle, rf_angle, _ = self._get_ui_values(mode)
        self._scenario.set_Ri_rotx(ri_angle)
        self._scenario.set_Rf_rotx(rf_angle)
        
        if self._scenario.spawn_xyz():
            self._update_labels(mode)
            self._update_status(f"XYZ spawned ({mode.upper()})")
        else:
            self._update_status("Failed to spawn XYZ.usd")

    def _on_generate(self, mode: str):
        if not self._scenario._xyz_prim:
            self._update_status("Spawn XYZ first!")
            return
        
        ri_angle, rf_angle, tf = self._get_ui_values(mode)
        self._scenario.set_Ri_rotx(ri_angle)
        self._scenario.set_Rf_rotx(rf_angle)
        
        if mode == "euler":
            result = self._scenario.generate_euler_trajectory(tf)
            if result["success"]:
                self._update_labels("euler")
                ei, ef = self._scenario.get_euler_i_deg(), self._scenario.get_euler_f_deg()
                self._update_status(f"[Euler] theta {ei[1]:.0f} to {ef[1]:.0f} deg, tf={tf:.1f}s")
        else:
            result = self._scenario.generate_angle_axis_trajectory(tf)
            if result["success"]:
                self._update_labels("aa")
                self._update_status(f"[Angle-Axis] theta={result['angle_deg']:.1f} deg, tf={tf:.1f}s")
        
        if not result["success"]:
            self._update_status(f"Failed: {result['message']}")

    def _on_execute(self, mode: str):
        btn_key = f"{mode}_execute_btn" if mode == "euler" else "aa_execute_btn"
        btn = self._ui_elements.get(btn_key)
        
        if self._scenario.is_executing:
            self._scenario.stop_execution()
            self._remove_physics_callback()
            if btn:
                btn.text = "Execute"
                btn.set_style({"background_color": 0xFF006600})
            self._update_status("Stopped")
        else:
            if self._scenario.trajectory_duration <= 0:
                self._update_status("Generate trajectory first!")
                return
            
            self._remove_physics_callback()
            self._scenario.start_execution()
            
            world = World.instance()
            if world:
                self._physics_sub = world.add_physics_callback("orientation_traj_update", self._on_physics_step)
            
            self._timeline.play()
            if btn:
                btn.text = "Stop"
                btn.set_style({"background_color": 0xFF660000})
            self._update_status(f"Executing ({mode.upper()})...")

    def _on_physics_step(self, dt: float):
        self._scenario.update(dt)
        state = self._scenario.get_state()
        
        if not state:
            return
        
        mode = state.get('mode')
        if mode == 'euler':
            angles = [np.degrees(state[k]) for k in ['phi', 'theta', 'psi']]
            self._update_status(f"t={state['time']:.2f}s | Euler: ({angles[0]:.1f}, {angles[1]:.1f}, {angles[2]:.1f})")
            btn_key = "euler_execute_btn"
        else:
            angle_deg = np.degrees(state['angle'])
            final_deg = np.degrees(state['angle_final'])
            self._update_status(f"t={state['time']:.2f}s | Angle: {angle_deg:.1f}/{final_deg:.1f} deg")
            btn_key = "aa_execute_btn"
        
        if not self._scenario.is_executing:
            self._remove_physics_callback()
            btn = self._ui_elements.get(btn_key)
            if btn:
                btn.text = "Execute"
                btn.set_style({"background_color": 0xFF006600})
            
            if mode == 'euler':
                self._update_status(f"[Euler] Done! ({angles[0]:.1f}, {angles[1]:.1f}, {angles[2]:.1f})")
            else:
                self._update_status(f"[Angle-Axis] Done! theta={final_deg:.1f} deg")

    def _update_labels(self, mode: str):
        """Update display labels for given mode."""
        Ri, Rf = self._scenario.get_Ri(), self._scenario.get_Rf()
        
        if mode == "euler":
            ei = self._scenario.tr2eul(Ri)
            ef = self._scenario.tr2eul(Rf)
            self._ui_elements["euler_i_label"].text = f"Ei=({np.degrees(ei[0]):.0f},{np.degrees(ei[1]):.0f},{np.degrees(ei[2]):.0f})"
            self._ui_elements["euler_f_label"].text = f"Ef=({np.degrees(ef[0]):.0f},{np.degrees(ef[1]):.0f},{np.degrees(ef[2]):.0f})"
        else:
            angle, axis = self._scenario.tr2angvec(Ri.T @ Rf)
            self._ui_elements["aa_angle_label"].text = f"theta={np.degrees(angle):.1f} deg"
            self._ui_elements["aa_axis_label"].text = f"v=({axis[0]:.2f},{axis[1]:.2f},{axis[2]:.2f})"

    def _update_status(self, text: str):
        if self._status_label:
            self._status_label.text = text
