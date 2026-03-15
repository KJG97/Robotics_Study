# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Assignment1 Scenario: ALLEX Robot Forward Kinematics
"""

import json
import os
from typing import List

import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdGeom

# Extension root directory (Robotics_Study/)
_EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


class Assignment1Scenario:
    """ALLEX Robot Forward Kinematics Scenario"""
    
    CONFIG_PATH = os.path.join(_EXT_ROOT, "config", "joint_config.json")
    LEFT_EE_PATH = "/ALLEX/Left_Hand_base"
    RIGHT_EE_PATH = "/ALLEX/Right_Hand_base"
    POSE_DURATION = 2.0  # seconds per pose
    
    def __init__(self):
        self._articulation = None
        self._config = None
        self._left_arm_indices: List[int] = []
        self._right_arm_indices: List[int] = []
        
        # Predefined poses
        self._poses = {
            "pose1": {
                "description": "Zero pose",
                "left_arm": np.zeros(7),
                "right_arm": np.zeros(7)
            },
            "pose2": {
                "description": "Symmetric arm pose",
                "left_arm": np.array([0, np.pi/4, 0, -np.pi/2, 0, 0, -np.pi/4]),
                "right_arm": np.array([0, -np.pi/4, 0, -np.pi/2, 0, 0, -np.pi/4])
            },
            "pose3": {
                "description": "Asymmetric arm pose",
                "left_arm": np.array([-np.pi/3, np.pi/6, -np.pi/4, -np.pi/2, 0, -np.pi/8, 0]),
                "right_arm": np.array([np.pi/3, -np.pi/6, np.pi/4, -np.pi/2, 0, np.pi/8, 0])
            }
        }
        
        # Animation state
        self._animation_active = False
        self._animation_time = 0.0
        self._current_pose_idx = 0
        
        # Load config
        if os.path.exists(self.CONFIG_PATH):
            try:
                with open(self.CONFIG_PATH, 'r') as f:
                    self._config = json.load(f)
                print(f"[Assignment1] Config loaded: {self.CONFIG_PATH}")
            except Exception as e:
                print(f"[Assignment1] Config load failed: {e}")
    
    def setup(self, articulation):
        """Setup scenario with robot articulation"""
        self._articulation = articulation
        if not self._articulation or not self._config:
            return
        
        # Find joint indices
        dof_names = self._articulation.dof_names
        allex_cfg = self._config.get("ALLEX", {})
        
        left_names = allex_cfg.get("left_arm", {}).get("joint_names", [])
        right_names = allex_cfg.get("right_arm", {}).get("joint_names", [])
        
        self._left_arm_indices = [dof_names.index(n) for n in left_names if n in dof_names]
        self._right_arm_indices = [dof_names.index(n) for n in right_names if n in dof_names]
        
        print(f"[Assignment1] Setup complete | DOF: {self._articulation.num_dof}")
    
    def apply_pose_and_get_result(self, pose_name: str) -> str:
        """Apply predefined pose and return FK result string"""
        if not self._articulation:
            return "[ERROR] Articulation not initialized"
        
        pose = self._poses.get(pose_name)
        if not pose:
            return f"[ERROR] Pose '{pose_name}' not found"
        
        positions = self._articulation.get_joint_positions()
        if positions is None:
            return "[ERROR] Failed to get joint positions"
        
        # Apply arm values
        new_pos = positions.copy()
        for i, idx in enumerate(self._left_arm_indices):
            if i < len(pose["left_arm"]):
                new_pos[idx] = pose["left_arm"][i]
        for i, idx in enumerate(self._right_arm_indices):
            if i < len(pose["right_arm"]):
                new_pos[idx] = pose["right_arm"][i]
        
        self._articulation.apply_action(ArticulationAction(joint_positions=new_pos))
        
        # Format result string
        fmt = lambda v: [f'{x:.4f}' for x in v]
        return (
            f"{'=' * 50}\n"
            f"FK Result: {pose_name}\n"
            f"Description: {pose['description']}\n"
            f"{'=' * 50}\n"
            f"\n[Left Arm]\n  Joint Values (rad): {fmt(pose['left_arm'])}\n"
            f"\n[Right Arm]\n  Joint Values (rad): {fmt(pose['right_arm'])}\n"
            f"{'=' * 50}"
        )
    
    def teardown(self):
        """Cleanup scenario resources"""
        self._animation_active = False
        self._articulation = None
        self._left_arm_indices.clear()
        self._right_arm_indices.clear()

    # ========================================
    # Animation
    # ========================================
    
    def start_animation(self):
        """Start pose animation cycle"""
        if not self._articulation:
            print("[Assignment1] Error: Articulation not initialized")
            return
        
        self._animation_active = True
        self._animation_time = 0.0
        self._current_pose_idx = 0
        
        # Clear debug draw
        try:
            from isaacsim.util.debug_draw import _debug_draw
            _debug_draw.acquire_debug_draw_interface().clear_points()
        except Exception:
            pass
        
        print(f"[Assignment1] Animation started | Duration: {self.POSE_DURATION}s per pose")
    
    def stop_animation(self):
        """Stop animation and return to pose1"""
        self._animation_active = False
        if self._articulation:
            self.apply_pose_and_get_result("pose1")
        
        # Clear debug draw
        try:
            from isaacsim.util.debug_draw import _debug_draw
            _debug_draw.acquire_debug_draw_interface().clear_points()
        except Exception:
            pass
        
        print("[Assignment1] Animation stopped")
    
    def is_animation_active(self) -> bool:
        return self._animation_active
    
    def update_animation(self, dt: float):
        """Update animation each physics step"""
        if not self._animation_active or not self._articulation:
            return
        
        self._animation_time += dt
        pose_sequence = ["pose1", "pose2", "pose3"]
        
        # Switch to next pose when duration exceeded
        if self._animation_time >= self.POSE_DURATION:
            self._animation_time = 0.0
            self._current_pose_idx = (self._current_pose_idx + 1) % 3
        
        # Interpolate between current and next pose
        curr_pose = self._poses[pose_sequence[self._current_pose_idx]]
        next_pose = self._poses[pose_sequence[(self._current_pose_idx + 1) % 3]]
        
        # Smooth cosine interpolation
        t = (1 - np.cos(self._animation_time / self.POSE_DURATION * np.pi)) / 2
        
        left_interp = (1 - t) * curr_pose["left_arm"] + t * next_pose["left_arm"]
        right_interp = (1 - t) * curr_pose["right_arm"] + t * next_pose["right_arm"]
        
        # Apply interpolated values
        positions = self._articulation.get_joint_positions()
        if positions is not None:
            new_pos = positions.copy()
            for i, idx in enumerate(self._left_arm_indices):
                if i < len(left_interp):
                    new_pos[idx] = left_interp[i]
            for i, idx in enumerate(self._right_arm_indices):
                if i < len(right_interp):
                    new_pos[idx] = right_interp[i]
            self._articulation.apply_action(ArticulationAction(joint_positions=new_pos))
        
        # Draw end-effector trajectory
        try:
            from isaacsim.core.utils.stage import get_current_stage
            from isaacsim.util.debug_draw import _debug_draw
            
            stage = get_current_stage()
            if not stage:
                return
            
            draw = _debug_draw.acquire_debug_draw_interface()
            
            # Draw both hands
            for ee_path, color in [(self.LEFT_EE_PATH, (0.0, 0.5, 1.0, 1.0)), 
                                   (self.RIGHT_EE_PATH, (1.0, 0.2, 0.2, 1.0))]:
                prim = stage.GetPrimAtPath(ee_path)
                if prim.IsValid():
                    xform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
                    pos = tuple(xform.GetRow(3)[:3])
                    draw.draw_points([pos], [color], [5])
        except Exception:
            pass
