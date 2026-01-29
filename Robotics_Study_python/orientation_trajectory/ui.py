# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""
Orientation Trajectory UI

Three trajectory methods:
1. Euler (ZYZ) - May have singularity
2. Angle-Axis - Shortest path, no singularity
3. Quaternion (SLERP) - Shortest path, no singularity
"""

import numpy as np
import omni.ui as ui
import omni.timeline
from isaacsim.core.api.world import World
from .scenario import OrientationTrajectoryScenario


class OrientationTrajectoryUI:
    """UI for orientation trajectory control."""

    # ─────────────────────────────────────────────────────────────────────────
    # Initialization
    # ─────────────────────────────────────────────────────────────────────────
    def __init__(self):
        self._scenario = OrientationTrajectoryScenario()
        self._timeline = omni.timeline.get_timeline_interface()
        self._window = None
        self._physics_sub = None
        self._status = None
        self._ui = {}

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
        for key in ["euler_btn", "aa_btn", "quat_btn"]:
            if btn := self._ui.get(key):
                btn.text, btn.set_style({"background_color": 0xFF006600})
        self._set_status("Ready")

    def on_stage_opened(self):
        self.teardown()

    def _remove_physics_callback(self):
        if self._physics_sub:
            self._physics_sub.unsubscribe()
            self._physics_sub = None
        try:
            world = World.instance()
            if world and world.physics_callback_exists("orientation_traj"):
                world.remove_physics_callback("orientation_traj")
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────────────────
    # UI Building
    # ─────────────────────────────────────────────────────────────────────────
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
        self._window = ui.Window("Orientation Trajectory", width=400, height=400)
        
        # Styles
        S = {
            "btn": {"border_radius": 3, "margin": 1, "font_size": 14,
                    ":hovered": {"background_color": 0xFF0088AA},
                    ":pressed": {"background_color": 0xFF004455}},
            "label": {"font_size": 14},
            "field": {"background_color": 0xFF222222, "font_size": 14},
            "info": {"font_size": 13, "color": 0xFF888888},
            "green": {"font_size": 14, "color": 0xFF88FF88},
            "red": {"font_size": 14, "color": 0xFFFF8888},
            "cyan": {"font_size": 14, "color": 0xFF88FFFF},
            "yellow": {"font_size": 14, "color": 0xFFFFFF88},
        }
        
        def btn(text, w, color, fn):
            return ui.Button(text, width=w, height=24, clicked_fn=fn,
                           style={**S["btn"], "background_color": color})
        
        def field(key, val, w=50):
            self._ui[key] = ui.FloatField(width=w, style=S["field"])
            self._ui[key].model.set_value(val)
        
        def label(key, text, w, style):
            self._ui[key] = ui.Label(text, width=w, style=style)
        
        with self._window.frame:
            with ui.VStack(spacing=4, style={"margin": 5}):
                # Status
                self._status = ui.Label("Ready", height=18, style={"font_size": 14, "color": 0xFFCCCCCC})
                
                # ═══════════════════════════════════════════════════════════════
                # Input Section
                # ═══════════════════════════════════════════════════════════════
                with ui.CollapsableFrame("Input (Ri, Rf)", collapsed=False):
                    with ui.VStack(spacing=3, style={"margin": 4}):
                        with ui.HStack(height=24, spacing=4):
                            ui.Label("Ri: rotx(", width=55, style=S["label"])
                            field("ri", 22.5)
                            ui.Label(") deg", width=35, style=S["label"])
                            ui.Spacer(width=10)
                            ui.Label("Rf: rotx(", width=55, style=S["label"])
                            field("rf", -22.5)
                            ui.Label(") deg", width=35, style=S["label"])
                            ui.Spacer()
                        
                        with ui.HStack(height=24, spacing=4):
                            ui.Label("tf:", width=20, style=S["label"])
                            field("tf", 5.0)
                            ui.Label("sec", width=25, style=S["label"])
                            ui.Spacer(width=10)
                            ui.Label("n:", width=12, style=S["label"])
                            field("n", 3.0, 35)
                            ui.Spacer(width=10)
                            btn("Spawn", 70, 0xFF006688, self._on_spawn)
                            btn("Clear", 60, 0xFF444444, self._scenario.clear_debug_draw)
                            ui.Spacer()
                
                # ═══════════════════════════════════════════════════════════════
                # Euler
                # ═══════════════════════════════════════════════════════════════
                with ui.CollapsableFrame("1. Euler (ZYZ)", collapsed=False):
                    with ui.VStack(spacing=2, style={"margin": 3}):
                        with ui.HStack(height=22, spacing=4):
                            label("ei", "Ei=(0,0,0)", 110, S["green"])
                            label("ef", "Ef=(0,45,0)", 110, S["red"])
                            ui.Spacer()
                        with ui.HStack(height=26, spacing=3):
                            btn("Generate", 70, 0xFF005588, lambda: self._on_generate("euler"))
                            self._ui["euler_btn"] = btn("Execute", 70, 0xFF006600, lambda: self._on_execute("euler"))
                            ui.Spacer()
                        ui.Label("R(t)=eul2r(phi,theta,psi) | May have singularity", height=14, style=S["info"])
                
                # ═══════════════════════════════════════════════════════════════
                # Angle-Axis
                # ═══════════════════════════════════════════════════════════════
                with ui.CollapsableFrame("2. Angle-Axis", collapsed=False):
                    with ui.VStack(spacing=2, style={"margin": 3}):
                        with ui.HStack(height=22, spacing=4):
                            label("aa_angle", "theta=- deg", 100, S["cyan"])
                            label("aa_axis", "v=(-,-,-)", 120, S["yellow"])
                            ui.Spacer()
                        with ui.HStack(height=26, spacing=3):
                            btn("Generate", 70, 0xFF885500, lambda: self._on_generate("angle_axis"))
                            self._ui["aa_btn"] = btn("Execute", 70, 0xFF006600, lambda: self._on_execute("angle_axis"))
                            ui.Spacer()
                        ui.Label("R(t)=Ri*angvec2r(theta(t),v) | No singularity", height=14, style=S["info"])
                
                # ═══════════════════════════════════════════════════════════════
                # Quaternion
                # ═══════════════════════════════════════════════════════════════
                with ui.CollapsableFrame("3. Quaternion (SLERP)", collapsed=False):
                    with ui.VStack(spacing=2, style={"margin": 3}):
                        with ui.HStack(height=22, spacing=4):
                            label("qi", "Qi=(1,0,0,0)", 130, S["green"])
                            label("qf", "Qf=(1,0,0,0)", 130, S["red"])
                            ui.Spacer()
                        with ui.HStack(height=26, spacing=3):
                            btn("linspace", 60, 0xFF555588, lambda: self._on_quat("linspace"))
                            btn("jtraj", 50, 0xFF885555, lambda: self._on_quat("jtraj"))
                            btn("ctraj", 50, 0xFF558855, lambda: self._on_quat("ctraj"))
                            self._ui["quat_btn"] = btn("Execute", 70, 0xFF006600, lambda: self._on_execute("quaternion"))
                            ui.Spacer()
                        ui.Label("linspace(const) | jtraj(S-curve,n) | ctraj(trap)", height=14, style=S["info"])

    # ─────────────────────────────────────────────────────────────────────────
    # Event Handlers
    # ─────────────────────────────────────────────────────────────────────────
    def _val(self, key: str) -> float:
        return self._ui[key].model.as_float

    def _set_status(self, text: str):
        if self._status:
            self._status.text = text

    def _on_spawn(self):
        self._scenario.set_Ri_rotx(self._val("ri"))
        self._scenario.set_Rf_rotx(self._val("rf"))
        if self._scenario.spawn_xyz():
            self._update_labels()
            self._set_status(f"Spawned: Ri=rotx({self._val('ri'):.1f}), Rf=rotx({self._val('rf'):.1f})")
        else:
            self._set_status("Failed to spawn XYZ.usd")

    def _on_generate(self, mode: str):
        if not self._scenario._xyz_prim:
            self._set_status("Spawn XYZ first!")
            return
        
        self._scenario.set_Ri_rotx(self._val("ri"))
        self._scenario.set_Rf_rotx(self._val("rf"))
        tf = self._val("tf")
        
        if mode == "euler":
            if self._scenario.generate_euler_trajectory(tf)["success"]:
                self._update_labels()
                ei, ef = self._scenario.get_euler_i_deg(), self._scenario.get_euler_f_deg()
                self._set_status(f"[Euler] theta: {ei[1]:.0f} -> {ef[1]:.0f} deg")
        else:  # angle_axis
            result = self._scenario.generate_angle_axis_trajectory(tf)
            if result["success"]:
                self._update_labels()
                self._set_status(f"[Angle-Axis] theta={result['angle_deg']:.1f} deg")

    def _on_quat(self, interp: str):
        if not self._scenario._xyz_prim:
            self._set_status("Spawn XYZ first!")
            return
        
        self._scenario.set_Ri_rotx(self._val("ri"))
        self._scenario.set_Rf_rotx(self._val("rf"))
        
        result = self._scenario.generate_quaternion_trajectory(self._val("tf"), interp, self._val("n"))
        if result["success"]:
            self._update_labels()
            info = {"linspace": "const vel", "jtraj": f"n={self._val('n'):.1f}", "ctraj": "trapezoid"}
            self._set_status(f"[Quaternion] {interp} ({info[interp]})")

    def _on_execute(self, mode: str):
        btn_key = {"euler": "euler_btn", "angle_axis": "aa_btn", "quaternion": "quat_btn"}[mode]
        btn = self._ui.get(btn_key)
        
        if self._scenario.is_executing:
            self._scenario.stop_execution()
            self._remove_physics_callback()
            if btn:
                btn.text = "Execute"
                btn.set_style({"background_color": 0xFF006600})
            self._set_status("Stopped")
        else:
            if self._scenario.trajectory_duration <= 0:
                self._set_status("Generate trajectory first!")
                return
            
            self._remove_physics_callback()
            self._scenario.start_execution()
            
            if world := World.instance():
                self._physics_sub = world.add_physics_callback("orientation_traj", self._on_step)
            
            self._timeline.play()
            if btn:
                btn.text = "Stop"
                btn.set_style({"background_color": 0xFF660000})
            self._set_status(f"Executing ({mode})...")

    def _on_step(self, dt: float):
        self._scenario.update(dt)
        state = self._scenario.get_state()
        if not state:
            return
        
        mode = state["mode"]
        btn_key = {"euler": "euler_btn", "angle_axis": "aa_btn", "quaternion": "quat_btn"}[mode]
        
        # Update status
        if mode == "euler":
            a = [np.degrees(state[k]) for k in ["phi", "theta", "psi"]]
            self._set_status(f"t={state['time']:.2f}s | ({a[0]:.1f}, {a[1]:.1f}, {a[2]:.1f})")
        elif mode == "angle_axis":
            self._set_status(f"t={state['time']:.2f}s | {np.degrees(state['angle']):.1f}/{np.degrees(state['angle_final']):.1f} deg")
        else:
            self._set_status(f"t={state['time']:.2f}s | s={state['s']:.3f} ({state['interp_type']})")
        
        # Execution complete
        if not self._scenario.is_executing:
            self._remove_physics_callback()
            if btn := self._ui.get(btn_key):
                btn.text = "Execute"
                btn.set_style({"background_color": 0xFF006600})
            self._set_status(f"[{mode.replace('_', '-').title()}] Done!")

    # ─────────────────────────────────────────────────────────────────────────
    # Label Updates
    # ─────────────────────────────────────────────────────────────────────────
    def _update_labels(self):
        Ri, Rf = self._scenario.get_Ri(), self._scenario.get_Rf()
        
        # Euler
        ei, ef = self._scenario.tr2eul(Ri), self._scenario.tr2eul(Rf)
        self._ui["ei"].text = f"Ei=({np.degrees(ei[0]):.0f},{np.degrees(ei[1]):.0f},{np.degrees(ei[2]):.0f})"
        self._ui["ef"].text = f"Ef=({np.degrees(ef[0]):.0f},{np.degrees(ef[1]):.0f},{np.degrees(ef[2]):.0f})"
        
        # Angle-Axis
        angle, axis = self._scenario.tr2angvec(Ri.T @ Rf)
        self._ui["aa_angle"].text = f"theta={np.degrees(angle):.1f} deg"
        self._ui["aa_axis"].text = f"v=({axis[0]:.2f},{axis[1]:.2f},{axis[2]:.2f})"
        
        # Quaternion
        qi, qf = self._scenario.rot2quat(Ri), self._scenario.rot2quat(Rf)
        self._ui["qi"].text = f"Qi=({qi[0]:.2f},{qi[1]:.2f},{qi[2]:.2f},{qi[3]:.2f})"
        self._ui["qf"].text = f"Qf=({qf[0]:.2f},{qf[1]:.2f},{qf[2]:.2f},{qf[3]:.2f})"
