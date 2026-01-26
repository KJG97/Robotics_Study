# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Tutorial UI - Robot Info Display and Chapter2 Examples
"""

import os
import numpy as np
import omni.timeline
import omni.ui as ui
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style

from .scenario import TutorialScenario

# Extension root directory (Robotics_Study/)
_EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


class TutorialUI:
    """UI for Robot Info Display and Chapter2 Examples."""
    
    # Asset path
    PRISMATIC_2DOF_USD = os.path.join(_EXT_ROOT, "asset", "prismatic_2dof.usd")

    def __init__(self):
        self._timeline = omni.timeline.get_timeline_interface()
        self._scenario = TutorialScenario()
        
        # State
        self._articulation = None
        self._info_window = None
        self._info_label = None
        self._show_info = False
        self._prismatic_loaded = False
        
        # UI elements
        self._example3_btn = None
        self._example4_btn = None
        self._tutorial_window = None

    # ========================================
    # Setup / Teardown
    # ========================================

    def setup(self, articulation):
        """Setup with robot articulation."""
        self._articulation = articulation
        self._scenario.setup(articulation)

    def teardown(self):
        """Clean up resources."""
        self._scenario.teardown()
        self._show_info = False
        if self._info_window:
            self._info_window.visible = False
            self._info_window = None
        if self._tutorial_window:
            self._tutorial_window.visible = False
            self._tutorial_window = None
        self._articulation = None
        self._prismatic_loaded = False

    def reset(self):
        """Reset state after world reset."""
        if self._scenario.is_trajectory_active():
            self._scenario.stop_7dof_trajectory()
        if self._scenario.is_2dof_trajectory_active():
            self._scenario.stop_2dof_trajectory()
        
        if self._example3_btn:
            self._example3_btn.text = "Example3: 7DOF Manipulator [START]"
        if self._example4_btn:
            self._example4_btn.text = "Example4: 2DOF Prismatic [START]"
        self._prismatic_loaded = False

    def on_stage_opened(self):
        """Handle stage opened event."""
        self._scenario.teardown()
        self._show_info = False
        if self._info_window:
            self._info_window.visible = False
        self._articulation = None
        self._prismatic_loaded = False
        
        if self._example3_btn:
            self._example3_btn.text = "Example3: 7DOF Manipulator [START]"
        if self._example4_btn:
            self._example4_btn.text = "Example4: 2DOF Prismatic [START]"

    # ========================================
    # Update
    # ========================================

    def update(self, step: float):
        """Update scenario and info display."""
        self._scenario.update(step)
        
        if self._show_info and self._info_label:
            self._display_joint_states()

    # ========================================
    # Build UI
    # ========================================

    def build_ui(self):
        """Build Tutorials button that opens a new window."""
        ui.Button("Tutorials", height=30, clicked_fn=self._on_open_tutorial_window)

    def _on_open_tutorial_window(self):
        """Open tutorial window with Robot Info and Chapter2 Examples."""
        if self._tutorial_window is None:
            self._tutorial_window = ui.Window(
                "Tutorials", width=350, height=250,
                position_x=100, position_y=100
            )
            with self._tutorial_window.frame:
                with ui.VStack(style=get_style(), spacing=5):
                    # Robot Information
                    with CollapsableFrame("Robot Information", collapsed=False):
                        with ui.VStack(style=get_style(), spacing=5, height=0):
                            ui.Button("Robot Info", height=30, clicked_fn=self._on_show_info)

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
        
        self._tutorial_window.visible = True

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
    # Accessors for external use
    # ========================================

    def is_trajectory_active(self) -> bool:
        """Check if 7DOF trajectory is active."""
        return self._scenario.is_trajectory_active()

    def is_2dof_trajectory_active(self) -> bool:
        """Check if 2DOF trajectory is active."""
        return self._scenario.is_2dof_trajectory_active()

