# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Orientation Trajectory UI using Euler Angles (MATLAB Style)

MATLAB 코드와 동일한 방식:
- Ri, Rf를 Rotation Matrix로 설정
- tr2eul로 Euler angle 변환
- jtraj (5차 다항식)로 궤적 생성
- eul2r로 R(t) 계산
"""

import numpy as np
import omni.ui as ui
import omni.timeline
from isaacsim.core.api.world import World

from .scenario import OrientationTrajectoryScenario


class OrientationTrajectoryUI:
    """UI for orientation trajectory using Euler angles (MATLAB style)."""

    def __init__(self):
        self._scenario = OrientationTrajectoryScenario()
        self._timeline = omni.timeline.get_timeline_interface()
        self._window = None
        self._physics_sub = None
        
        # UI elements
        self._status_label = None
        self._spawn_btn = None
        self._ri_type_combo = None
        self._ri_angle_field = None
        self._rf_angle_field = None
        self._tf_field = None
        self._steps_field = None
        self._execute_btn = None
        self._euler_i_label = None
        self._euler_f_label = None

    def setup(self, articulation):
        """Setup with articulation."""
        self._scenario.setup(articulation)

    def teardown(self):
        """Cleanup resources."""
        self._scenario.teardown()
        self._remove_physics_callback()
        if self._window:
            self._window.visible = False
            self._window = None

    def reset(self):
        """Reset UI state."""
        self._scenario.stop_execution()
        self._remove_physics_callback()
        if self._execute_btn:
            self._execute_btn.text = "Execute"
            self._execute_btn.set_style({"background_color": 0xFF006600})
        self._update_status("Ready")

    def on_stage_opened(self):
        """Handle stage opened event."""
        self.teardown()

    def _remove_physics_callback(self):
        """Remove physics callback safely."""
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
        """Build the UI button."""
        ui.Button("Orientation Trajectory", height=30, clicked_fn=self._open_window)

    def _open_window(self):
        """Open the orientation trajectory window."""
        if not self._scenario.is_ready:
            print("[OrientationTraj] Load robot first!")
            return
        if not self._window:
            self._create_window()
        self._window.visible = True

    def _create_window(self):
        """Create the orientation trajectory window."""
        self._window = ui.Window("Orientation Trajectory (Euler/jtraj)", width=480, height=450,
                                  flags=ui.WINDOW_FLAGS_NO_SCROLLBAR)
        
        btn_style = {
            "border_radius": 4,
            "margin": 2,
            ":hovered": {"background_color": 0xFF0088AA},
            ":pressed": {"background_color": 0xFF004455},
        }
        
        with self._window.frame:
            with ui.VStack(spacing=6, style={"margin": 10}):
                # Title
                ui.Label("Orientation Trajectory - Euler Angles (MATLAB Style)", height=24,
                         style={"font_size": 15, "color": 0xFFFFFFFF})
                
                # Status
                self._status_label = ui.Label("Ready", height=20,
                                               style={"font_size": 13, "color": 0xFFCCCCCC})
                
                ui.Spacer(height=3)
                
                # ─────────────────────────────────────────────────────────────
                # 1. Set Ri, Rf (Rotation Matrix)
                # ─────────────────────────────────────────────────────────────
                with ui.CollapsableFrame("1. Set Ri, Rf (Rotation Matrix)", height=0, collapsed=False):
                    with ui.VStack(spacing=4, style={"margin": 4}):
                        # Ri selection
                        with ui.HStack(height=24, spacing=4):
                            ui.Label("Ri =", width=30, style={"font_size": 13})
                            self._ri_type_combo = ui.ComboBox(0, "eye(3)", "rotx(angle)",
                                                               width=100, style={"font_size": 12})
                            self._ri_type_combo.model.add_item_changed_fn(self._on_ri_type_changed)
                            ui.Label("angle:", width=45, style={"font_size": 12})
                            self._ri_angle_field = ui.FloatField(width=60, style={"background_color": 0xFF333333})
                            self._ri_angle_field.model.set_value(0.0)
                            self._ri_angle_field.enabled = False
                            ui.Label("°", width=15, style={"font_size": 13})
                        
                        # Rf input
                        with ui.HStack(height=24, spacing=4):
                            ui.Label("Rf =", width=30, style={"font_size": 13})
                            ui.Label("rotx(", width=40, style={"font_size": 13})
                            self._rf_angle_field = ui.FloatField(width=60, style={"background_color": 0xFF333333})
                            self._rf_angle_field.model.set_value(-45.0)  # -π/4 in degrees
                            ui.Label("°)", width=20, style={"font_size": 13})
                            ui.Spacer()
                        
                        ui.Label("Default: Ri=eye(3), Rf=rotx(-π/4)", height=16,
                                 style={"font_size": 11, "color": 0xFF888888})
                
                # ─────────────────────────────────────────────────────────────
                # 2. Spawn XYZ.usd
                # ─────────────────────────────────────────────────────────────
                with ui.CollapsableFrame("2. Spawn XYZ.usd", height=0, collapsed=False):
                    with ui.VStack(spacing=4, style={"margin": 4}):
                        self._spawn_btn = ui.Button(
                            "Spawn XYZ at (1, 1, 1) with Ri", height=28,
                            clicked_fn=self._on_spawn_xyz,
                            style={**btn_style, "background_color": 0xFF006688, "font_size": 13}
                        )
                
                # ─────────────────────────────────────────────────────────────
                # 3. Generate Trajectory (jtraj)
                # ─────────────────────────────────────────────────────────────
                with ui.CollapsableFrame("3. Generate Trajectory (jtraj)", height=0, collapsed=False):
                    with ui.VStack(spacing=4, style={"margin": 4}):
                        # Euler angles display
                        self._euler_i_label = ui.Label("Euler_i = tr2eul(Ri) = (-, -, -)", height=18,
                                                        style={"font_size": 12, "color": 0xFF88FF88})
                        self._euler_f_label = ui.Label("Euler_f = tr2eul(Rf) = (-, -, -)", height=18,
                                                        style={"font_size": 12, "color": 0xFFFF8888})
                        
                        ui.Spacer(height=4)
                        
                        # Parameters
                        with ui.HStack(height=24, spacing=4):
                            ui.Label("tf:", width=40, style={"font_size": 13})
                            self._tf_field = ui.FloatField(width=60, style={"background_color": 0xFF333333})
                            self._tf_field.model.set_value(2.0)
                            ui.Label("sec", width=30, style={"font_size": 13})
                            
                            ui.Label("steps:", width=45, style={"font_size": 13})
                            self._steps_field = ui.IntField(width=50, style={"background_color": 0xFF333333})
                            self._steps_field.model.set_value(20)
                            ui.Spacer()
                        
                        ui.Button("Generate (jtraj - 5th order polynomial)", height=28,
                                  clicked_fn=self._on_generate,
                                  style={**btn_style, "background_color": 0xFF005588, "font_size": 13})
                
                # ─────────────────────────────────────────────────────────────
                # 4. Execute Trajectory
                # ─────────────────────────────────────────────────────────────
                with ui.CollapsableFrame("4. Execute Trajectory", height=0, collapsed=False):
                    with ui.VStack(spacing=4, style={"margin": 4}):
                        self._execute_btn = ui.Button("Execute", height=28,
                                                       clicked_fn=self._on_execute,
                                                       style={**btn_style, "background_color": 0xFF006600, "font_size": 13})
                        ui.Label("R(t) = eul2r(φ(t), θ(t), ψ(t))", height=16,
                                 style={"font_size": 11, "color": 0xFF888888})
                
                ui.Spacer(height=5)
                
                # Info
                ui.Label("ZYZ Euler convention: R = Rz(φ) * Ry(θ) * Rz(ψ)",
                         style={"font_size": 11, "color": 0xFF666666})

    def _on_ri_type_changed(self, model, item):
        """Handle Ri type combo box change."""
        idx = model.get_item_value_model().as_int
        # 0: eye(3), 1: rotx(angle)
        self._ri_angle_field.enabled = (idx == 1)

    def _on_spawn_xyz(self):
        """Handle spawn XYZ button click."""
        if self._spawn_btn:
            original_text = self._spawn_btn.text
            self._spawn_btn.text = "Spawning..."
            self._spawn_btn.enabled = False

        # Set Ri based on combo selection
        ri_type = self._ri_type_combo.model.get_item_value_model().as_int
        if ri_type == 0:
            # eye(3)
            self._scenario.set_Ri_eye()
        else:
            # rotx(angle)
            ri_angle = self._ri_angle_field.model.as_float
            self._scenario.set_Ri_rotx(ri_angle)
        
        # Set Rf = rotx(angle)
        rf_angle = self._rf_angle_field.model.as_float
        self._scenario.set_Rf_rotx(rf_angle)
        
        success = self._scenario.spawn_xyz()

        if self._spawn_btn:
            self._spawn_btn.text = original_text
            self._spawn_btn.enabled = True

        if success:
            self._update_status("XYZ spawned with Ri")
            self._update_euler_labels()
        else:
            self._update_status("Failed to spawn XYZ.usd")

    def _update_euler_labels(self):
        """Update Euler angle display labels."""
        # tr2eul을 사용하여 Euler angles 계산
        Ri = self._scenario.get_Ri()
        Rf = self._scenario.get_Rf()
        
        euler_i = self._scenario.tr2eul(Ri)
        euler_f = self._scenario.tr2eul(Rf)
        
        if self._euler_i_label:
            self._euler_i_label.text = (f"Euler_i = tr2eul(Ri) = "
                                        f"({np.degrees(euler_i[0]):.1f}°, "
                                        f"{np.degrees(euler_i[1]):.1f}°, "
                                        f"{np.degrees(euler_i[2]):.1f}°)")
        
        if self._euler_f_label:
            self._euler_f_label.text = (f"Euler_f = tr2eul(Rf) = "
                                        f"({np.degrees(euler_f[0]):.1f}°, "
                                        f"{np.degrees(euler_f[1]):.1f}°, "
                                        f"{np.degrees(euler_f[2]):.1f}°)")

    def _on_generate(self):
        """Generate trajectory."""
        if not self._scenario._xyz_prim:
            self._update_status("Spawn XYZ first!")
            return
        
        # Update Ri, Rf from fields
        ri_type = self._ri_type_combo.model.get_item_value_model().as_int
        if ri_type == 0:
            self._scenario.set_Ri_eye()
        else:
            ri_angle = self._ri_angle_field.model.as_float
            self._scenario.set_Ri_rotx(ri_angle)
        
        rf_angle = self._rf_angle_field.model.as_float
        self._scenario.set_Rf_rotx(rf_angle)
        
        # Get parameters
        tf = self._tf_field.model.as_float if self._tf_field else 2.0
        steps = self._steps_field.model.as_int if self._steps_field else 20
        
        result = self._scenario.generate_trajectory(tf=tf, num_steps=steps)
        
        if result["success"]:
            self._update_euler_labels()
            euler_i = self._scenario.get_euler_i_deg()
            euler_f = self._scenario.get_euler_f_deg()
            self._update_status(f"jtraj: ({euler_i[1]:.0f}°) → ({euler_f[1]:.0f}°), tf={tf:.1f}s")
        else:
            self._update_status(f"Failed: {result['message']}")

    def _on_execute(self):
        """Execute or stop trajectory."""
        if self._scenario.is_executing:
            self._scenario.stop_execution()
            self._remove_physics_callback()
            self._execute_btn.text = "Execute"
            self._execute_btn.set_style({"background_color": 0xFF006600})
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
            self._execute_btn.text = "Stop"
            self._execute_btn.set_style({"background_color": 0xFF660000})
            self._update_status("Executing...")

    def _on_physics_step(self, dt: float):
        """Physics step callback."""
        self._scenario.update(dt)
        state = self._scenario.get_state()
        
        if state:
            phi_deg = np.degrees(state['phi'])
            theta_deg = np.degrees(state['theta'])
            psi_deg = np.degrees(state['psi'])
            
            self._update_status(f"t={state['time']:.2f}s | Euler(ZYZ): ({phi_deg:.1f}°, {theta_deg:.1f}°, {psi_deg:.1f}°)")
            
            if not self._scenario.is_executing:
                self._remove_physics_callback()
                self._execute_btn.text = "Execute"
                self._execute_btn.set_style({"background_color": 0xFF006600})
                self._update_status(f"Done! Final: ({phi_deg:.1f}°, {theta_deg:.1f}°, {psi_deg:.1f}°)")

    def _update_status(self, text: str):
        """Update status label."""
        if self._status_label:
            self._status_label.text = text
