# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Assignment1 Scenario: ALLEX Robot Forward Kinematics

Core logic for predefined pose management, FK calculation, and pose animation.
"""

import json
import os
from typing import Dict, List, Optional

import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdGeom


class Assignment1Scenario:
    """ALLEX Robot Forward Kinematics Scenario"""
    
    # Configuration
    CONFIG_PATH = "/home/jkkim/isaac-sim/extsUser/Robotics_Study/config/joint_config.json"
    
    # End-effector paths
    LEFT_EE_PATH = "/ALLEX/Left_Hand_base"
    RIGHT_EE_PATH = "/ALLEX/Right_Hand_base"
    
    # Animation settings
    POSE_DURATION = 2.0  # seconds per pose
    POSE_SEQUENCE = ["pose1", "pose2", "pose3"]
    
    # Debug draw colors
    BLUE = (0.0, 0.5, 1.0, 1.0)
    RED = (1.0, 0.2, 0.2, 1.0)
    POINT_SIZE = 5
    
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
        self._draw_enabled = False
        
        self._load_config()
    
    def _load_config(self):
        """Load joint configuration from JSON file"""
        if not os.path.exists(self.CONFIG_PATH):
            print(f"[Assignment1] Config not found: {self.CONFIG_PATH}")
            return
        
        try:
            with open(self.CONFIG_PATH, 'r') as f:
                self._config = json.load(f)
            print(f"[Assignment1] Config loaded: {self.CONFIG_PATH}")
        except Exception as e:
            print(f"[Assignment1] Config load failed: {e}")
    
    def setup(self, articulation):
        """Setup scenario with robot articulation"""
        self._articulation = articulation
        if not self._articulation:
            print("[Assignment1] Error: Articulation is None")
            return
        
        self._find_joint_indices()
        print(f"[Assignment1] Setup complete | DOF: {self._articulation.num_dof}")
    
    def _find_joint_indices(self):
        """Map joint names to indices for both arms"""
        if not self._articulation or not self._config:
            return
        
        dof_names = self._articulation.dof_names
        allex_cfg = self._config.get("ALLEX", {})
        
        for arm, indices_attr in [("left_arm", "_left_arm_indices"), ("right_arm", "_right_arm_indices")]:
            joint_names = allex_cfg.get(arm, {}).get("joint_names", [])
            setattr(self, indices_attr, [dof_names.index(n) for n in joint_names if n in dof_names])
    
    def apply_pose(self, pose_name: str) -> Dict:
        """Apply predefined pose and return FK result"""
        if not self._articulation:
            return {"error": "Articulation not initialized"}
        
        pose = self._poses.get(pose_name)
        if not pose:
            return {"error": f"Pose '{pose_name}' not found"}
        
        positions = self._articulation.get_joint_positions()
        if positions is None:
            return {"error": "Failed to get joint positions"}
        
        new_pos = positions.copy()
        self._apply_arm_values(new_pos, pose["left_arm"], self._left_arm_indices)
        self._apply_arm_values(new_pos, pose["right_arm"], self._right_arm_indices)
        
        self._articulation.apply_action(ArticulationAction(joint_positions=new_pos))
        return self._build_fk_result(pose_name, pose)
    
    def _apply_arm_values(self, positions: np.ndarray, values: np.ndarray, indices: List[int]):
        """Apply arm joint values to position array"""
        for i, idx in enumerate(indices):
            if i < len(values):
                positions[idx] = values[i]
    
    def _build_fk_result(self, pose_name: str, pose: Dict) -> Dict:
        """Build FK result dictionary"""
        return {
            "pose_name": pose_name,
            "description": pose.get("description", ""),
            "left_arm": {"joint_values": pose["left_arm"].tolist()},
            "right_arm": {"joint_values": pose["right_arm"].tolist()}
        }
    
    def get_fk_result_string(self, result: Dict) -> str:
        """Format FK result as displayable string"""
        if "error" in result:
            return f"[ERROR] {result['error']}"
        
        fmt_vals = lambda v: [f'{x:.4f}' for x in v]
        return (
            f"{'=' * 50}\n"
            f"FK Result: {result['pose_name']}\n"
            f"Description: {result['description']}\n"
            f"{'=' * 50}\n"
            f"\n[Left Arm]\n  Joint Values (rad): {fmt_vals(result['left_arm']['joint_values'])}\n"
            f"\n[Right Arm]\n  Joint Values (rad): {fmt_vals(result['right_arm']['joint_values'])}\n"
            f"{'=' * 50}"
        )
    
    def teardown(self):
        """Cleanup scenario resources"""
        self._animation_active = False
        self._draw_enabled = False
        self._articulation = None
        self._left_arm_indices.clear()
        self._right_arm_indices.clear()
    
    # ========================================
    # Animation
    # ========================================
    
    def _get_debug_draw(self):
        """Get debug draw interface"""
        try:
            from isaacsim.util.debug_draw import _debug_draw
            return _debug_draw.acquire_debug_draw_interface()
        except Exception:
            return None
    
    def _clear_debug_draw(self):
        """Clear debug draw points"""
        try:
            draw = self._get_debug_draw()
            if draw:
                draw.clear_points()
        except Exception:
            pass
    
    def start_animation(self):
        """Start pose animation cycle"""
        if not self._articulation:
            print("[Assignment1] Error: Articulation not initialized")
            return
        
        self._animation_active = True
        self._animation_time = 0.0
        self._current_pose_idx = 0
        self._draw_enabled = True
        self._clear_debug_draw()
        print(f"[Assignment1] Animation started | Duration: {self.POSE_DURATION}s per pose")
    
    def stop_animation(self):
        """Stop animation and return to pose1"""
        self._animation_active = False
        self._draw_enabled = False
        if self._articulation:
            self.apply_pose("pose1")
        self._clear_debug_draw()
        print("[Assignment1] Animation stopped")
    
    def is_animation_active(self) -> bool:
        return self._animation_active
    
    def update_animation(self, dt: float):
        """Update animation each physics step"""
        if not self._animation_active or not self._articulation:
            return
        
        self._animation_time += dt
        
        # Switch to next pose when duration exceeded
        if self._animation_time >= self.POSE_DURATION:
            self._animation_time = 0.0
            self._current_pose_idx = (self._current_pose_idx + 1) % len(self.POSE_SEQUENCE)
        
        # Interpolate between current and next pose
        curr_pose = self._poses[self.POSE_SEQUENCE[self._current_pose_idx]]
        next_pose = self._poses[self.POSE_SEQUENCE[(self._current_pose_idx + 1) % len(self.POSE_SEQUENCE)]]
        
        # Smooth cosine interpolation
        t = (1 - np.cos(self._animation_time / self.POSE_DURATION * np.pi)) / 2
        
        left_interp = (1 - t) * curr_pose["left_arm"] + t * next_pose["left_arm"]
        right_interp = (1 - t) * curr_pose["right_arm"] + t * next_pose["right_arm"]
        
        # Apply interpolated values
        positions = self._articulation.get_joint_positions()
        if positions is not None:
            new_pos = positions.copy()
            self._apply_arm_values(new_pos, left_interp, self._left_arm_indices)
            self._apply_arm_values(new_pos, right_interp, self._right_arm_indices)
            self._articulation.apply_action(ArticulationAction(joint_positions=new_pos))
        
        self._draw_ee_trajectory()
    
    def _draw_ee_trajectory(self):
        """Draw end-effector trajectory points"""
        if not self._draw_enabled:
            return
        
        try:
            from isaacsim.core.utils.stage import get_current_stage
            stage = get_current_stage()
            if not stage:
                return
            
            draw = self._get_debug_draw()
            if not draw:
                return
            
            # Draw both hands
            for ee_path, color in [(self.LEFT_EE_PATH, self.BLUE), (self.RIGHT_EE_PATH, self.RED)]:
                prim = stage.GetPrimAtPath(ee_path)
                if prim.IsValid():
                    xform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
                    pos = tuple(xform.GetRow(3)[:3])
                    draw.draw_points([pos], [color], [self.POINT_SIZE])
        except Exception:
            pass
