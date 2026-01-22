# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Assignment1 UI: ALLEX Robot Forward Kinematics

UI components for robot loading, pose selection, animation control, and FK display.
"""

import omni.timeline
import omni.ui as ui
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage, get_current_stage
from isaacsim.examples.extension.core_connectors import LoadButton
from isaacsim.gui.components.ui_utils import get_style
from pxr import Sdf, UsdLux

from .scenario import Assignment1Scenario


class Assignment1UI:
    """Assignment1 UI Builder - Handles all UI components and callbacks"""
    
    # Configuration
    ROBOT_USD_PATH = "/home/jkkim/isaac-sim/extsUser/Robotics_Study/asset/ALLEX.usd"
    ROBOT_PRIM_PATH = "/ALLEX"
    
    # Pose button configurations
    POSES = [
        ("pose1", "Pose 1 (Zero)"),
        ("pose2", "Pose 2 (Symmetric)"),
        ("pose3", "Pose 3 (Asymmetric)")
    ]
    
    def __init__(self):
        self._timeline = omni.timeline.get_timeline_interface()
        self._scenario = Assignment1Scenario()
        self._articulation = None
        self._anim_btn = None
        self._fk_label = None
        self.wrapped_ui_elements = []
    
    def build_ui(self, window):
        """Build UI for Assignment1 window"""
        with window.frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                ui.Label("Assignment1: ALLEX Forward Kinematics", 
                        alignment=ui.Alignment.CENTER, height=30)
                ui.Spacer(height=5)
                
                # Load button
                load_btn = LoadButton(
                    "Assignment1 Load", "LOAD ALLEX",
                    setup_scene_fn=self._setup_scene,
                    setup_post_load_fn=self._post_load
                )
                load_btn.set_world_settings(physics_dt=1/60.0, rendering_dt=1/60.0)
                self.wrapped_ui_elements.append(load_btn)
                
                ui.Spacer(height=10)
                ui.Label("Apply Pose & Calculate FK:", height=20)
                
                # Pose buttons
                with ui.HStack(spacing=5, height=30):
                    for pose_id, label in self.POSES:
                        ui.Button(label, clicked_fn=lambda p=pose_id: self._on_pose_clicked(p))
                
                ui.Spacer(height=10)
                ui.Label("Animation (Pose 1 -> 2 -> 3 -> repeat):", height=20)
                
                self._anim_btn = ui.Button(
                    "Animation [START]", height=30,
                    clicked_fn=self._on_animation_toggle
                )
                
                ui.Spacer(height=10)
                ui.Label("Forward Kinematics Result:", height=20)
                
                with ui.ScrollingFrame(height=300):
                    self._fk_label = ui.Label(
                        "Load robot and click a pose button to see FK result.",
                        alignment=ui.Alignment.LEFT_TOP,
                        word_wrap=True,
                        style={"font_size": 12, "color": 0xFFCCCCCC}
                    )
    
    def _setup_scene(self):
        """Setup scene - load ALLEX robot"""
        create_new_stage()
        
        # Add light
        UsdLux.DomeLight.Define(get_current_stage(), Sdf.Path("/World/DomeLight")).CreateIntensityAttr(1000)
        
        # Load robot
        add_reference_to_stage(self.ROBOT_USD_PATH, self.ROBOT_PRIM_PATH)
        self._articulation = SingleArticulation(self.ROBOT_PRIM_PATH)
        World.instance().scene.add(self._articulation)
        
        print("[Assignment1] ALLEX robot loaded")
    
    def _post_load(self):
        """Post-load callback - setup gravity and scenario"""
        # Zero gravity for kinematic study
        World.instance().get_physics_context().set_gravity(value=0.0)
        print("[Assignment1] Gravity: 0")
        
        if self._articulation:
            self._scenario.setup(self._articulation)
            self._update_label("Robot loaded! Click a pose button.")
    
    def _on_pose_clicked(self, pose_name: str):
        """Handle pose button click"""
        if not self._articulation:
            self._update_label("[ERROR] Robot not loaded. Click LOAD ALLEX first.")
            return
        
        result = self._scenario.apply_pose(pose_name)
        result_str = self._scenario.get_fk_result_string(result)
        self._update_label(result_str)
        print(result_str)
    
    def _on_animation_toggle(self):
        """Toggle animation Start/Stop"""
        if not self._articulation:
            self._update_label("[ERROR] Robot not loaded. Click LOAD ALLEX first.")
            return
        
        if self._scenario.is_animation_active():
            self._scenario.stop_animation()
            self._anim_btn.text = "Animation [START]"
            self._update_label("Animation stopped. Returned to Pose 1.")
        else:
            self._scenario.start_animation()
            self._anim_btn.text = "Animation [STOP]"
            self._update_label(
                "Animation running...\n"
                "Pose 1 -> 2 -> 3 -> repeat\n\n"
                "Trajectory:\n  Blue = Left Hand\n  Red = Right Hand"
            )
            self._timeline.play()
    
    def _update_label(self, text: str):
        """Update FK result label"""
        if self._fk_label:
            self._fk_label.text = text
    
    def update_animation(self, step: float):
        """Update animation (called from physics step)"""
        if self._scenario.is_animation_active():
            self._scenario.update_animation(step)
    
    def is_animation_active(self) -> bool:
        return self._scenario.is_animation_active()
    
    def cleanup(self):
        """Cleanup UI elements"""
        self._scenario.teardown()
        self._articulation = None
        for elem in self.wrapped_ui_elements:
            elem.cleanup()
        self.wrapped_ui_elements.clear()
