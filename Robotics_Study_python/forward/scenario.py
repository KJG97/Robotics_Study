# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Forward Kinematics Scenario - LulaKinematicsSolver for FK computation"""

import os
import numpy as np
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

_EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


class ForwardKinematicsScenario:
    """Forward kinematics scenario with LulaKinematicsSolver."""

    ROBOT_DESC_PATH = os.path.join(_EXT_ROOT, "asset", "URDF_ALLEX_RightArm", "config", "robot_descriptor.yaml")
    URDF_PATH = os.path.join(_EXT_ROOT, "asset", "URDF_ALLEX_RightArm", "urdf", "URDF_ALLEX_RightArm.urdf")
    EE_FRAME = "tcp"

    def __init__(self):
        self.articulation = None  # Public - UI에서 직접 접근
        self._solver = None
        self._num_dof = 0
        self._dof_names = []
        self._joint_limits = []

    # ─────────────────────────────────────────────────────────────────────────
    # Setup / Teardown
    # ─────────────────────────────────────────────────────────────────────────
    def setup(self, articulation):
        self.articulation = articulation
        if not articulation:
            return

        self._num_dof = articulation.num_dof
        self._dof_names = list(articulation.dof_names)
        
        # Joint limits (clamp infinite values)
        lower, upper = articulation.dof_properties["lower"], articulation.dof_properties["upper"]
        self._joint_limits = [
            (max(lower[i], -np.pi), min(upper[i], np.pi)) for i in range(self._num_dof)
        ]

        # Initialize solver
        try:
            self._solver = LulaKinematicsSolver(
                robot_description_path=self.ROBOT_DESC_PATH, urdf_path=self.URDF_PATH
            )
            print(f"[FK] Ready - DOF: {self._num_dof}, EE: {self.EE_FRAME}")
        except Exception as e:
            print(f"[FK] Solver init failed: {e}")
            self._solver = None

    def teardown(self):
        self.articulation = self._solver = None
        self._num_dof = 0
        self._dof_names, self._joint_limits = [], []

    # ─────────────────────────────────────────────────────────────────────────
    # Properties
    # ─────────────────────────────────────────────────────────────────────────
    @property
    def num_dof(self): return self._num_dof
    
    @property
    def dof_names(self): return self._dof_names
    
    @property
    def joint_limits(self): return self._joint_limits
    
    @property
    def is_ready(self): return self._solver is not None

    # ─────────────────────────────────────────────────────────────────────────
    # Forward Kinematics
    # ─────────────────────────────────────────────────────────────────────────
    def compute_fk(self) -> dict | None:
        if not self._solver or not self.articulation:
            return None
        
        joint_pos = self.articulation.get_joint_positions()
        if joint_pos is None:
            return None

        try:
            pos, ori = self.articulation.get_world_pose()
            self._solver.set_robot_base_pose(pos, ori)

            position, rotation = self._solver.compute_forward_kinematics(
                frame_name=self.EE_FRAME, joint_positions=joint_pos
            )
            return {"position": position, "rpy": self._rotation_to_rpy(rotation)}
        except Exception as e:
            print(f"[FK] Error: {e}")
            return None

    @staticmethod
    def _rotation_to_rpy(r: np.ndarray) -> tuple:
        """Convert 3x3 rotation matrix to (roll, pitch, yaw) using atan2."""
        r13, r23, r33 = r[0, 2], r[1, 2], r[2, 2]
        r31, r32 = r[2, 0], r[2, 1]

        if abs(r13) > 1e-10 or abs(r23) > 1e-10:
            pitch = np.arctan2(np.sqrt(r13**2 + r23**2), r33)
            if pitch > 0:
                return np.arctan2(r23, r13), pitch, np.arctan2(r32, -r31)
            return np.arctan2(-r23, -r13), pitch, np.arctan2(-r32, r31)
        
        # Gimbal lock
        return 0.0, (0.0 if r33 > 0 else np.pi), np.arctan2(r[1, 0], r[0, 0])
