# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Inverse Kinematics Scenario - Follow Target with LulaKinematicsSolver"""

import os
import numpy as np
import carb
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver

_EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


class InverseKinematicsScenario:
    """Inverse kinematics scenario - robot follows a target cube."""

    ROBOT_DESC_PATH = os.path.join(_EXT_ROOT, "asset", "URDF_ALLEX_RightArm", "config", "robot_descriptor.yaml")
    URDF_PATH = os.path.join(_EXT_ROOT, "asset", "URDF_ALLEX_RightArm", "urdf", "URDF_ALLEX_RightArm.urdf")
    EE_FRAME = "tcp"
    
    # Workspace bounds
    WS_X = (-0.6, 1.0)
    WS_Y = (-1.0, 0.5)
    WS_Z = (0.5, 2.0)

    def __init__(self):
        self.articulation = None
        self.target = None
        self._solver = None
        self._art_solver = None
        self._active = False
        self._position_only = False

    # ─────────────────────────────────────────────────────────────────────────
    # Setup / Teardown
    # ─────────────────────────────────────────────────────────────────────────
    def setup(self, articulation, target):
        self.articulation = articulation
        self.target = target
        if not articulation:
            return

        try:
            self._solver = LulaKinematicsSolver(
                robot_description_path=self.ROBOT_DESC_PATH, urdf_path=self.URDF_PATH
            )
            self._art_solver = ArticulationKinematicsSolver(
                articulation, self._solver, self.EE_FRAME
            )
            print(f"[IK] Ready - EE: {self.EE_FRAME}")
        except Exception as e:
            print(f"[IK] Solver init failed: {e}")
            self._solver = None

    def teardown(self):
        self._active = self._position_only = False
        self.articulation = self.target = self._solver = self._art_solver = None

    # ─────────────────────────────────────────────────────────────────────────
    # Properties
    # ─────────────────────────────────────────────────────────────────────────
    @property
    def is_ready(self):
        return self._solver is not None and self.target is not None

    @property
    def is_active(self):
        return self._active

    @property
    def position_only(self):
        return self._position_only

    def start(self):
        if self.is_ready:
            self._active = True
            print("[IK] Started")

    def stop(self):
        self._active = False
        print("[IK] Stopped")

    def set_position_only(self, value: bool):
        self._position_only = value
        print(f"[IK] Mode: {'Position Only' if value else 'Position + Orientation'}")

    # ─────────────────────────────────────────────────────────────────────────
    # Update (called each physics step)
    # ─────────────────────────────────────────────────────────────────────────
    def update(self):
        if not self._active or not self.is_ready:
            return

        try:
            target_pos, target_ori = self.target.get_world_pose()
            robot_pos, robot_ori = self.articulation.get_world_pose()
            self._solver.set_robot_base_pose(robot_pos, robot_ori)

            ori = None if self._position_only else target_ori
            action, success = self._art_solver.compute_inverse_kinematics(target_pos, ori)

            if success:
                self.articulation.apply_action(action)
            else:
                carb.log_warn("[IK] No solution")
        except Exception as e:
            carb.log_error(f"[IK] Error: {e}")

    # ─────────────────────────────────────────────────────────────────────────
    # End-Effector Pose
    # ─────────────────────────────────────────────────────────────────────────
    def get_ee_pose(self):
        if not self._art_solver:
            return None, None
        try:
            return self._art_solver.compute_end_effector_pose()
        except:
            return None, None

    # ─────────────────────────────────────────────────────────────────────────
    # Workspace Visualization
    # ─────────────────────────────────────────────────────────────────────────
    def compute_workspace(self, position_only: bool, resolution: int = 15) -> dict:
        """Compute reachable workspace by testing IK at grid points."""
        if not self._solver or not self.articulation:
            return {"reachable": [], "unreachable": []}

        # Generate grid points
        grid = np.mgrid[
            self.WS_X[0]:self.WS_X[1]:complex(resolution),
            self.WS_Y[0]:self.WS_Y[1]:complex(resolution),
            self.WS_Z[0]:self.WS_Z[1]:complex(resolution)
        ].reshape(3, -1).T  # Shape: (N, 3)

        fixed_ori = np.array([1.0, 0.0, 0.0, 0.0])
        reachable, unreachable = [], []

        # Set robot base pose once
        robot_pos, robot_ori = self.articulation.get_world_pose()
        self._solver.set_robot_base_pose(robot_pos, robot_ori)

        total = len(grid)
        for i, pos in enumerate(grid):
            try:
                ori = None if position_only else fixed_ori
                _, success = self._art_solver.compute_inverse_kinematics(pos, ori)
                (reachable if success else unreachable).append(tuple(pos))
            except:
                unreachable.append(tuple(pos))

            if (i + 1) % 500 == 0:
                print(f"[Workspace] {i+1}/{total} ({100*(i+1)/total:.0f}%)")

        print(f"[Workspace] Done! Reachable: {len(reachable)}, Unreachable: {len(unreachable)}")
        return {"reachable": reachable, "unreachable": unreachable}
