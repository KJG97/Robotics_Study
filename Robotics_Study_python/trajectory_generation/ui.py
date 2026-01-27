# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Trapezoidal Velocity Profile UI"""

import numpy as np
import omni.ui as ui
import omni.timeline
from isaacsim.core.api.world import World

from .scenario import TrajectoryGenerationScenario


class TrajectoryGenerationUI:
    """UI for single joint trajectory generation."""

    PLOT_SAMPLES = 100

    def __init__(self):
        self._scenario = TrajectoryGenerationScenario()
        self._timeline = omni.timeline.get_timeline_interface()
        self._window = None
        self._physics_sub = None
        
        # UI elements
        self._qi_field = None
        self._qf_field = None
        self._tf_field = None
        self._a_max_field = None
        self._status_label = None
        self._execute_btn = None
        self._position_plot = None
        self._velocity_plot = None
        self._accel_plot = None

    def setup(self, articulation):
        self._scenario.setup(articulation)

    def teardown(self):
        self._scenario.stop_execution()
        self._remove_physics_callback()
        self._scenario.teardown()
        if self._window:
            self._window.visible = False
            self._window = None

    def reset(self):
        self._scenario.stop_execution()
        self._remove_physics_callback()
        if self._execute_btn:
            self._execute_btn.text = "Execute"
            self._execute_btn.set_style({"background_color": 0xFF006600})
        self._update_status("Ready")

    def on_stage_opened(self):
        self.teardown()

    def _remove_physics_callback(self):
        """Remove physics callback safely."""
        if self._physics_sub:
            self._physics_sub.unsubscribe()
            self._physics_sub = None
        try:
            world = World.instance()
            if world and world.physics_callback_exists("trajgen_update"):
                world.remove_physics_callback("trajgen_update")
        except Exception:
            pass

    def build_ui(self):
        ui.Button("Trajectory Generation", height=30, clicked_fn=self._open_window)

    def _open_window(self):
        if not self._scenario.is_ready:
            print("[TrajGen] Load robot first!")
            return
        if not self._window:
            self._create_window()
        self._window.visible = True

    def _create_window(self):
        self._window = ui.Window("Trapezoidal Trajectory", width=500, height=650,
                                  flags=ui.WINDOW_FLAGS_NO_SCROLLBAR)
        
        with self._window.frame:
            with ui.VStack(spacing=4, style={"margin": 6}):
                # Status
                self._status_label = ui.Label("Ready", height=20,
                                               style={"font_size": 14, "color": 0xFFCCCCCC})
                
                # Input fields (2 columns)
                with ui.HStack(height=24, spacing=4):
                    ui.Label("qi:", width=25)
                    self._qi_field = ui.FloatField(style={"background_color": 0xFF333333})
                    self._qi_field.model.set_value(0.0)
                    ui.Label("qf:", width=25)
                    self._qf_field = ui.FloatField(style={"background_color": 0xFF333333})
                    self._qf_field.model.set_value(45.0)
                    ui.Label("deg", width=30)

                with ui.HStack(height=24, spacing=4):
                    ui.Label("tf:", width=25)
                    self._tf_field = ui.FloatField(style={"background_color": 0xFF333333})
                    self._tf_field.model.set_value(2.0)
                    ui.Label("a:", width=25)
                    self._a_max_field = ui.FloatField(style={"background_color": 0xFF333333})
                    self._a_max_field.model.set_value(114.6)
                    ui.Label("deg/s²", width=45)

                # Buttons
                with ui.HStack(height=26, spacing=4):
                    ui.Button("Generate", clicked_fn=self._on_generate,
                              style={"background_color": 0xFF005588})
                    self._execute_btn = ui.Button("Execute", clicked_fn=self._on_execute,
                                                   style={"background_color": 0xFF006600})
                    ui.Button("Current q", clicked_fn=self._on_set_current_qi,
                              style={"background_color": 0xFF444400})

                # Plots
                self._create_plot("Pos\n(deg)", 0xFFFF8800, "_position_plot")
                self._create_plot("Vel\n(deg/s)", 0xFF88FF88, "_velocity_plot")
                self._create_plot("Acc\n(deg/s²)", 0xFF8888FF, "_accel_plot")

    def _create_plot(self, label: str, color: int, attr_name: str):
        """Create a plot with grid background."""
        with ui.HStack(spacing=2):
            ui.Label(label, width=40, style={"font_size": 14, "color": color})
            with ui.ZStack():
                # Grid background
                ui.Rectangle(style={"background_color": 0xFF1A1A1A, "border_radius": 2})
                with ui.VStack(spacing=0):
                    for i in range(5):
                        if i > 0:
                            ui.Spacer()
                        ui.Line(style={"color": 0xFF333333, "border_width": 1})
                # Plot
                plot = ui.Plot(ui.Type.LINE, style={"color": color, "background_color": 0x00000000})
                setattr(self, attr_name, plot)

    def _on_set_current_qi(self):
        pos = self._scenario.get_current_position()
        if pos is not None:
            self._qi_field.model.set_value(np.degrees(pos))

    def _on_generate(self):
        if not self._scenario.is_ready:
            self._update_status("Error: Robot not ready!")
            return

        qi_deg = self._qi_field.model.as_float
        qf_deg = self._qf_field.model.as_float
        tf = self._tf_field.model.as_float
        a_max_deg = self._a_max_field.model.as_float
        
        if tf <= 0:
            self._update_status("Error: tf must be > 0!")
            return
        if a_max_deg <= 0:
            self._update_status("Error: a must be > 0!")
            return
        
        # Clamp to limits and update fields
        lo, hi = self._scenario.joint_limits
        qi_deg = np.clip(qi_deg, np.degrees(lo), np.degrees(hi))
        qf_deg = np.clip(qf_deg, np.degrees(lo), np.degrees(hi))
        self._qi_field.model.set_value(qi_deg)
        self._qf_field.model.set_value(qf_deg)
        
        result = self._scenario.generate(
            qi=np.radians(qi_deg),
            qf=np.radians(qf_deg),
            tf=tf,
            a_max=np.radians(a_max_deg)
        )
        
        if result["success"]:
            traj = self._scenario._trajectory
            profile = "Triangular" if traj.is_triangular else "Trapezoidal"
            self._update_status(f"{profile}: tf={traj.tf:.2f}s, tc={traj.tc:.2f}s, v_max={np.degrees(traj.v_max):.1f}°/s")
            self._update_plots()
        else:
            self._update_status(f"Failed: {result['message']}")

    def _update_plots(self):
        traj = self._scenario._trajectory
        if not traj or traj.duration < 1e-6:
            return
        
        times = np.linspace(0, traj.duration, self.PLOT_SAMPLES)
        pos, vel, acc = [], [], []
        
        for t in times:
            q, v, a = traj.get_state(t)
            pos.append(float(np.degrees(q)))
            vel.append(float(np.degrees(v)))
            acc.append(float(np.degrees(a)))
        
        def update_plot(plot, data):
            if plot:
                margin = max(abs(max(data) - min(data)) * 0.1, 1.0)
                plot.scale_min = min(data) - margin
                plot.scale_max = max(data) + margin
                plot.set_data(*data)
        
        update_plot(self._position_plot, pos)
        update_plot(self._velocity_plot, vel)
        update_plot(self._accel_plot, acc)

    def _on_execute(self):
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
                self._physics_sub = world.add_physics_callback("trajgen_update", self._on_physics_step)
            
            self._timeline.play()
            self._execute_btn.text = "Stop"
            self._execute_btn.set_style({"background_color": 0xFF660000})
            self._update_status("Executing...")

    def _on_physics_step(self, dt: float):
        self._scenario.update(dt)
        state = self._scenario.get_state()
        
        if state:
            self._update_status(f"t={state['time']:.2f}s, q={np.degrees(state['position']):.1f}°")
            
            if not self._scenario.is_executing:
                self._remove_physics_callback()
                self._execute_btn.text = "Execute"
                self._execute_btn.set_style({"background_color": 0xFF006600})
                self._update_status(f"Done! Final: {np.degrees(state['position']):.1f}°")

    def _update_status(self, text: str):
        if self._status_label:
            self._status_label.text = text
