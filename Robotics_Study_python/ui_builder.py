# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Main UI Builder for Robotics Study Extension

Handles main UI components: World Controls, Run Scenario, Robot Information, Chapter2 Examples.
Assignment-specific UIs are located in the assignments/ module.
"""

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


class UIBuilder:
    """Main UI builder for Robotics Study extension."""
    
    # Asset paths
    _ROBOT_USD_PATH = "/home/jkkim/isaac-sim/extsUser/Robotics_Study/asset/ALLEX_Right_Arm.usd"
    _PRISMATIC_2DOF_USD_PATH = "/home/jkkim/isaac-sim/extsUser/Robotics_Study/asset/prismatic_2dof.usd"
    _ROBOT_PRIM_PATH = "/ALLEX_Right_Arm"
    _PRISMATIC_2DOF_PRIM_PATH = "/prismatic_2dof"
    
    # UI styles
    _INFO_WINDOW_BG_COLOR = 0xDD1A1A2E
    _INFO_WINDOW_BORDER_COLOR = 0xFF00D4FF

    def __init__(self):
        self.frames = []
        self.wrapped_ui_elements = []
        
        self._timeline = omni.timeline.get_timeline_interface()
        self._scenario = RoboticsStudyScenario()
        self._assignment1_ui = Assignment1UI()
        
        # UI state
        self._articulation = None
        self._info_overlay_window = None
        self._info_label = None
        self._current_display_mode = None
        self._prismatic_2dof_loaded = False
        
        # UI element references
        self._example3_btn = None
        self._example4_btn = None

    # ========================================
    # Extension Callbacks (called by extension.py)
    # ========================================

    def on_menu_callback(self):
        """Callback when UI is opened from toolbar."""
        pass

    def on_timeline_event(self, event):
        """Handle timeline events (Play, Pause, Stop)."""
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = False

    def on_physics_step(self, step: float):
        """Physics step callback."""
        self._assignment1_ui.update_animation(step)

    def on_stage_event(self, event):
        """Handle stage events."""
        if event.type == int(StageEventType.OPENED):
            self._reset_extension()

    def cleanup(self):
        """Clean up resources when stage closes or extension reloads."""
        self._scenario.teardown_scenario()
        self._assignment1_ui.cleanup()
        
        if self._info_overlay_window:
            self._info_overlay_window.visible = False
            self._info_overlay_window = None
            self._info_label = None
        
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    # ========================================
    # Main UI Build
    # ========================================

    def build_ui(self):
        """Build the main extension UI."""
        self._build_world_controls_frame()
        self._build_run_scenario_frame()
        self._build_robot_info_frame()
        self._build_chapter2_frame()

    def _build_world_controls_frame(self):
        """Build World Controls collapsable frame."""
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

    def _build_run_scenario_frame(self):
        """Build Run Scenario collapsable frame."""
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

    def _build_robot_info_frame(self):
        """Build Robot Information collapsable frame."""
        with CollapsableFrame("Robot Information", collapsed=False):
            with ui.VStack(style=get_style(), spacing=5, height=0):
                ui.Button("Robot Info", height=30, clicked_fn=self._on_show_all_states)
                ui.Button("Close Info Window", height=25, clicked_fn=self._on_close_info_window)

    def _build_chapter2_frame(self):
        """Build Chapter2 Example collapsable frame."""
        with CollapsableFrame("Chapter2 Example", collapsed=False):
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._example3_btn = ui.Button(
                    "Example3: 7DOF Manipulator [START]",
                    height=30,
                    clicked_fn=self._on_example3_toggle
                )
                self._example4_btn = ui.Button(
                    "Example4: 2DOF Prismatic [START]",
                    height=30,
                    clicked_fn=self._on_example4_toggle
                )

    # ========================================
    # Scene Setup
    # ========================================

    def _add_light_to_stage(self):
        """Add dome light to the stage."""
        dome_light = UsdLux.DomeLight.Define(get_current_stage(), Sdf.Path("/World/DomeLight"))
        dome_light.CreateIntensityAttr(1000)

    def _setup_scene(self):
        """Setup scene with robot."""
        create_new_stage()
        self._add_light_to_stage()
        add_reference_to_stage(self._ROBOT_USD_PATH, self._ROBOT_PRIM_PATH)
        
        self._articulation = SingleArticulation(self._ROBOT_PRIM_PATH)
        World.instance().scene.add(self._articulation)

    def _setup_post_load(self):
        """Post-load setup callback."""
        self._scenario.setup_scenario(self._articulation)
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True
        self._reset_btn.enabled = True

    def _on_post_reset(self):
        """Post-reset callback."""
        # Stop active trajectories
        if self._scenario.is_trajectory_active():
            self._scenario.stop_7dof_trajectory()
        if self._scenario.is_2dof_trajectory_active():
            self._scenario.stop_2dof_trajectory()
        
        # Reset button texts
        self._update_example_button_texts(reset=True)
        self._prismatic_2dof_loaded = False
        
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True

    def _update_scenario(self, step: float):
        """Update scenario and info display."""
        self._scenario.update_scenario(step)
        
        if self._current_display_mode == "all_states" and self._info_label:
            self._display_all_states()

    def _reset_extension(self):
        """Reset extension state when new stage opens."""
        self._scenario.teardown_scenario()
        self._on_close_info_window()
        self._articulation = None
        self._prismatic_2dof_loaded = False
        self._current_display_mode = None
        self._reset_ui()

    def _reset_ui(self):
        """Reset UI elements to default state."""
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = False
        self._reset_btn.enabled = False
        self._update_example_button_texts(reset=True)

    def _update_example_button_texts(self, reset=False):
        """Update example button texts."""
        if self._example3_btn:
            self._example3_btn.text = "Example3: 7DOF Manipulator [START]"
        if self._example4_btn:
            self._example4_btn.text = "Example4: 2DOF Prismatic [START]"
        if reset:
            self._prismatic_2dof_loaded = False

    # ========================================
    # Robot Info Display
    # ========================================
    
    def _create_info_overlay_window(self):
        """Create info overlay window if not exists."""
        if self._info_overlay_window:
            return
        
        self._info_overlay_window = ui.Window(
            "Robot Info", width=350, height=450,
            flags=ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_NO_SCROLLBAR,
            position_x=20, position_y=100
        )
        
        with self._info_overlay_window.frame:
            with ui.ZStack():
                ui.Rectangle(style={
                    "background_color": self._INFO_WINDOW_BG_COLOR,
                    "border_radius": 10,
                    "border_width": 2,
                    "border_color": self._INFO_WINDOW_BORDER_COLOR
                })
                with ui.VStack(spacing=5):
                    ui.Spacer(height=10)
                    self._info_label = ui.Label(
                        "Loading robot info...",
                        alignment=ui.Alignment.LEFT_TOP,
                        word_wrap=True,
                        style={"color": 0xFFFFFFFF, "font_size": 14, "margin": 15}
                    )
                    ui.Spacer(height=10)
    
    def _on_show_all_states(self):
        """Show all joint states in realtime."""
        self._current_display_mode = "all_states"
        self._create_info_overlay_window()
        self._display_all_states()
        self._info_overlay_window.visible = True
    
    def _on_close_info_window(self):
        """Close info window."""
        self._current_display_mode = None
        if self._info_overlay_window:
            self._info_overlay_window.visible = False
    
    def _display_all_states(self):
        """Display all joint states."""
        if not self._info_label:
            return
        
        info = self._scenario.get_all_joint_states()
        
        if "error" in info:
            self._info_label.text = f"[ERROR] {info['error']}"
            return
        
        # Build display text
        lines = [
            "[ALL JOINT STATES - Realtime]",
            "=" * 35,
            "",
            f"{'Joint':<12} {'Pos(rad)':>10} {'Vel(rad/s)':>10} {'Torque(Nm)':>10}",
            "-" * 45,
        ]
        
        names = info['dof_names']
        positions = info['positions']
        velocities = info['velocities']
        torques = info['torques']
        
        for i, name in enumerate(names):
            pos = positions[i] if i < len(positions) else 0
            vel = velocities[i] if i < len(velocities) else 0
            torque = torques[i] if i < len(torques) else 0
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
            if not self._prismatic_2dof_loaded:
                self._load_prismatic_2dof()
                self._prismatic_2dof_loaded = True
            
            self._scenario.start_2dof_trajectory()
            self._example4_btn.text = "Example4: 2DOF Prismatic [RESET]"
            self._timeline.play()
    
    def _load_prismatic_2dof(self):
        """Load 2DOF Prismatic robot to stage."""
        try:
            if get_current_stage() is None:
                print("[Example4] Error: No stage available.")
                return
            
            add_reference_to_stage(self._PRISMATIC_2DOF_USD_PATH, self._PRISMATIC_2DOF_PRIM_PATH)
            
            prim = XFormPrim(self._PRISMATIC_2DOF_PRIM_PATH)
            prim.set_world_poses(positions=np.array([[0.0, 1.0, 0.0]]))
            
            print(f"[Example4] Loaded: {self._PRISMATIC_2DOF_PRIM_PATH} at (0, 1, 0)")
        except Exception as e:
            print(f"[Example4] Failed to load robot: {e}")

    # ========================================
    # Assignment UI Delegation
    # ========================================
    
    def build_assignment1_ui(self, window):
        """Build Assignment1 UI in the given window."""
        self._assignment1_ui.build_ui(window)
        self.wrapped_ui_elements.extend(self._assignment1_ui.wrapped_ui_elements)
    
    def update_assignment1_animation(self, step: float):
        """Update Assignment1 animation."""
        self._assignment1_ui.update_animation(step)
