# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Main UI Builder for Robotics Study Extension
"""

import os
import numpy as np
import omni.timeline
import omni.ui as ui
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation, XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage, get_current_stage
from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton
from isaacsim.gui.components.element_wrappers import CollapsableFrame, StateButton
from isaacsim.gui.components.ui_utils import get_style
from omni.usd import StageEventType
from pxr import Sdf, UsdLux

from .scenario import RoboticsStudyScenario
from .assignments import Assignment1UI

# Extension root directory (Robotics_Study/)
_EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class UIBuilder:
    """Main UI builder for Robotics Study extension."""
    
    # Asset paths (relative to extension root)
    ROBOT_USD = os.path.join(_EXT_ROOT, "asset", "ALLEX_Right_Arm.usd")
    PRISMATIC_2DOF_USD = os.path.join(_EXT_ROOT, "asset", "prismatic_2dof.usd")

    def __init__(self):
        self.frames = []
        self.wrapped_ui_elements = []
        
        self._timeline = omni.timeline.get_timeline_interface()
        self._scenario = RoboticsStudyScenario()
        self._assignment1_ui = Assignment1UI()
        
        # State
        self._articulation = None
        self._info_window = None
        self._info_label = None
        self._show_info = False
        self._prismatic_loaded = False
        
        # UI elements
        self._example3_btn = None
        self._example4_btn = None

    # ========================================
    # Extension Callbacks
    # ========================================

    def on_menu_callback(self):
        pass

    def on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = False

    def on_physics_step(self, step: float):
        self._assignment1_ui.update_animation(step)

    def on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED):
            # Reset all state
            self._scenario.teardown_scenario()
            self._show_info = False
            if self._info_window:
                self._info_window.visible = False
            self._articulation = None
            self._prismatic_loaded = False
            
            # Reset UI
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = False
            self._reset_btn.enabled = False
            if self._example3_btn:
                self._example3_btn.text = "Example3: 7DOF Manipulator [START]"
            if self._example4_btn:
                self._example4_btn.text = "Example4: 2DOF Prismatic [START]"

    def cleanup(self):
        self._scenario.teardown_scenario()
        self._assignment1_ui.cleanup()
        
        if self._info_window:
            self._info_window.visible = False
            self._info_window = None
        
        for elem in self.wrapped_ui_elements:
            elem.cleanup()

    # ========================================
    # Main UI Build
    # ========================================

    def build_ui(self):
        """Build the main extension UI."""
        # World Controls
        with CollapsableFrame("World Controls", collapsed=False):
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._load_btn = LoadButton(
                    "Load Button", "LOAD",
                    setup_scene_fn=self._setup_scene,
                    setup_post_load_fn=self._setup_post_load
                )
                self._load_btn.set_world_settings(physics_dt=1/60.0, rendering_dt=1/60.0)
                self.wrapped_ui_elements.append(self._load_btn)

                self._reset_btn = ResetButton(
                    "Reset Button", "RESET",
                    pre_reset_fn=None,
                    post_reset_fn=self._on_post_reset
                )
                self._reset_btn.enabled = False
                self.wrapped_ui_elements.append(self._reset_btn)

        # Run Scenario
        with CollapsableFrame("Run Scenario", collapsed=False):
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._scenario_state_btn = StateButton(
                    "Run Scenario", "RUN", "STOP",
                    on_a_click_fn=self._timeline.play,
                    on_b_click_fn=self._timeline.pause,
                    physics_callback_fn=self._update_scenario,
                )
                self._scenario_state_btn.enabled = False
                self.wrapped_ui_elements.append(self._scenario_state_btn)

        # Robot Information
        with CollapsableFrame("Robot Information", collapsed=False):
            with ui.VStack(style=get_style(), spacing=5, height=0):
                ui.Button("Robot Info", height=30, clicked_fn=self._on_show_info)
                ui.Button("Close Info Window", height=25, clicked_fn=self._on_close_info)

        # Chapter2 Examples
        with CollapsableFrame("Chapter2 Example", collapsed=False):
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._example3_btn = ui.Button(
                    "Example3: 7DOF Manipulator [START]", height=30,
                    clicked_fn=self._on_example3_toggle
                )
                self._example4_btn = ui.Button(
                    "Example4: 2DOF Prismatic [START]", height=30,
                    clicked_fn=self._on_example4_toggle
                )

    # ========================================
    # Scene Setup
    # ========================================

    def _setup_scene(self):
        """Setup scene with robot."""
        create_new_stage()
        
        # Add dome light
        UsdLux.DomeLight.Define(get_current_stage(), Sdf.Path("/World/DomeLight")).CreateIntensityAttr(1000)
        
        # Load robot
        add_reference_to_stage(self.ROBOT_USD, "/World/ALLEX_Right_Arm")
        self._articulation = SingleArticulation("/World/ALLEX_Right_Arm")
        World.instance().scene.add(self._articulation)

    def _setup_post_load(self):
        """Post-load setup callback."""
        self._scenario.setup_scenario(self._articulation)
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True
        self._reset_btn.enabled = True

    def _on_post_reset(self):
        """Post-reset callback."""
        if self._scenario.is_trajectory_active():
            self._scenario.stop_7dof_trajectory()
        if self._scenario.is_2dof_trajectory_active():
            self._scenario.stop_2dof_trajectory()
        
        if self._example3_btn:
            self._example3_btn.text = "Example3: 7DOF Manipulator [START]"
        if self._example4_btn:
            self._example4_btn.text = "Example4: 2DOF Prismatic [START]"
        self._prismatic_loaded = False
        
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True

    def _update_scenario(self, step: float):
        """Update scenario and info display."""
        self._scenario.update_scenario(step)
        
        if self._show_info and self._info_label:
            self._display_joint_states()

    # ========================================
    # Robot Info Display
    # ========================================
    
    def _on_show_info(self):
        """Show joint states info window."""
        self._show_info = True
        
        # Create window if needed
        if not self._info_window:
            self._info_window = ui.Window(
                "Robot Info", width=350, height=450,
                flags=ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_NO_SCROLLBAR,
                position_x=20, position_y=100
            )
            with self._info_window.frame:
                with ui.ZStack():
                    ui.Rectangle(style={
                        "background_color": 0xDD1A1A2E,
                        "border_radius": 10,
                        "border_width": 2,
                        "border_color": 0xFF00D4FF
                    })
                    with ui.VStack(spacing=5):
                        ui.Spacer(height=10)
                        self._info_label = ui.Label(
                            "Loading...",
                            alignment=ui.Alignment.LEFT_TOP,
                            word_wrap=True,
                            style={"color": 0xFFFFFFFF, "font_size": 14, "margin": 15}
                        )
                        ui.Spacer(height=10)
        
        self._display_joint_states()
        self._info_window.visible = True
    
    def _on_close_info(self):
        """Close info window."""
        self._show_info = False
        if self._info_window:
            self._info_window.visible = False
    
    def _display_joint_states(self):
        """Display all joint states."""
        if not self._info_label:
            return
        
        info = self._scenario.get_all_joint_states()
        
        if "error" in info:
            self._info_label.text = f"[ERROR] {info['error']}"
            return
        
        lines = [
            "[ALL JOINT STATES - Realtime]",
            "=" * 35, "",
            f"{'Joint':<12} {'Pos(rad)':>10} {'Vel(rad/s)':>10} {'Torque(Nm)':>10}",
            "-" * 45,
        ]
        
        for i, name in enumerate(info['dof_names']):
            pos = info['positions'][i] if i < len(info['positions']) else 0
            vel = info['velocities'][i] if i < len(info['velocities']) else 0
            torque = info['torques'][i] if i < len(info['torques']) else 0
            lines.append(f"{name[:10]:<12} {pos:>10.3f} {vel:>10.3f} {torque:>10.3f}")
        
        self._info_label.text = "\n".join(lines)

    # ========================================
    # Chapter2 Examples
    # ========================================
    
    def _on_example3_toggle(self):
        """Toggle 7DOF Manipulator trajectory."""
        if self._scenario.is_trajectory_active():
            self._scenario.stop_7dof_trajectory()
            self._example3_btn.text = "Example3: 7DOF Manipulator [START]"
        else:
            self._scenario.start_7dof_trajectory()
            self._example3_btn.text = "Example3: 7DOF Manipulator [RESET]"
            self._timeline.play()
    
    def _on_example4_toggle(self):
        """Toggle 2DOF Prismatic robot trajectory."""
        if self._scenario.is_2dof_trajectory_active():
            self._scenario.stop_2dof_trajectory()
            self._example4_btn.text = "Example4: 2DOF Prismatic [START]"
        else:
            # Load robot if not loaded
            if not self._prismatic_loaded:
                try:
                    if get_current_stage() is None:
                        print("[Example4] Error: No stage available.")
                        return
                    
                    add_reference_to_stage(self.PRISMATIC_2DOF_USD, "/prismatic_2dof")
                    XFormPrim("/prismatic_2dof").set_world_poses(positions=np.array([[0.0, 1.0, 0.0]]))
                    print("[Example4] Loaded: /prismatic_2dof at (0, 1, 0)")
                    self._prismatic_loaded = True
                except Exception as e:
                    print(f"[Example4] Failed to load robot: {e}")
                    return
            
            self._scenario.start_2dof_trajectory()
            self._example4_btn.text = "Example4: 2DOF Prismatic [RESET]"
            self._timeline.play()

    # ========================================
    # Assignment UI
    # ========================================
    
    def build_assignment1_ui(self, window):
        """Build Assignment1 UI in the given window."""
        self._assignment1_ui.build_ui(window)
        self.wrapped_ui_elements.extend(self._assignment1_ui.wrapped_ui_elements)
    
    def update_assignment1_animation(self, step: float):
        """Update Assignment1 animation."""
        self._assignment1_ui.update_animation(step)
