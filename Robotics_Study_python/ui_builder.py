# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Main UI Builder for Robotics Study Extension"""

import os
import omni.timeline
import omni.ui as ui
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage, get_current_stage
from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton
from isaacsim.gui.components.element_wrappers import CollapsableFrame, StateButton
from isaacsim.gui.components.ui_utils import get_style
from omni.usd import StageEventType
from pxr import Sdf, UsdLux

from .assignments import Assignment1UI
from .tutorial import TutorialUI
from .forward import ForwardKinematicsUI
from .inverse import InverseKinematicsUI

_EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class UIBuilder:
    """Main UI builder for Robotics Study extension."""
    
    ROBOT_USD = os.path.join(_EXT_ROOT, "asset", "ALLEX_Right_Arm.usd")

    def __init__(self):
        self.frames = []
        self.wrapped_ui_elements = []
        
        self._timeline = omni.timeline.get_timeline_interface()
        self._assignment1_ui = Assignment1UI()
        self._tutorial_ui = TutorialUI()
        self._forward_ui = ForwardKinematicsUI()
        self._inverse_ui = InverseKinematicsUI()
        
        self._articulation = None

    # ─────────────────────────────────────────────────────────────────────────
    # Extension Callbacks
    # ─────────────────────────────────────────────────────────────────────────
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
            self._tutorial_ui.on_stage_opened()
            self._forward_ui.on_stage_opened()
            self._inverse_ui.on_stage_opened()
            self._articulation = None
            
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = False
            self._reset_btn.enabled = False

    def cleanup(self):
        self._assignment1_ui.cleanup()
        self._tutorial_ui.teardown()
        self._forward_ui.teardown()
        self._inverse_ui.teardown()
        
        for elem in self.wrapped_ui_elements:
            elem.cleanup()

    # ─────────────────────────────────────────────────────────────────────────
    # Main UI Build
    # ─────────────────────────────────────────────────────────────────────────
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

        # Tutorial UI
        self._tutorial_ui.build_ui()
        
        # Forward Kinematics UI
        self._forward_ui.build_ui()
        
        # Inverse Kinematics UI
        self._inverse_ui.build_ui()

    # ─────────────────────────────────────────────────────────────────────────
    # Scene Setup
    # ─────────────────────────────────────────────────────────────────────────
    def _setup_scene(self):
        """Setup scene with robot."""
        create_new_stage()
        UsdLux.DomeLight.Define(get_current_stage(), Sdf.Path("/World/DomeLight")).CreateIntensityAttr(500)
        
        add_reference_to_stage(self.ROBOT_USD, "/World/ALLEX_Right_Arm")
        self._articulation = SingleArticulation("/World/ALLEX_Right_Arm")
        World.instance().scene.add(self._articulation)

    def _setup_post_load(self):
        """Post-load setup callback."""
        self._tutorial_ui.setup(self._articulation)
        self._forward_ui.setup(self._articulation)
        self._inverse_ui.setup(self._articulation)
        
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True
        self._reset_btn.enabled = True
        
        print(f"[OK] Robot loaded - DOF: {self._articulation.num_dof}")

    def _on_post_reset(self):
        """Post-reset callback."""
        self._tutorial_ui.reset()
        self._forward_ui.reset()
        self._inverse_ui.reset()
        
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True

    def _update_scenario(self, step: float):
        """Update scenario each physics step."""
        self._tutorial_ui.update(step)

    # ─────────────────────────────────────────────────────────────────────────
    # Assignment UI
    # ─────────────────────────────────────────────────────────────────────────
    def build_assignment1_ui(self, window):
        """Build Assignment1 UI in the given window."""
        self._assignment1_ui.build_ui(window)
        self.wrapped_ui_elements.extend(self._assignment1_ui.wrapped_ui_elements)
    
    def update_assignment1_animation(self, step: float):
        """Update Assignment1 animation."""
        self._assignment1_ui.update_animation(step)
