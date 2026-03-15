# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Assignment2 Scenario: ALLEX Jacobian — link-to-link T, J_SerialLink, manipulability ellipsoid.
"""

import json
import os
from typing import List, Tuple, Any, Optional

import numpy as np
from pxr import Usd, UsdGeom

# Fixed base (bT0) and tool (7Tt) from mdl_ALLEX_Simple.m (row-major 4x4)
_D2R = np.pi / 180.0
_LB = 0.5
BASE_LEFT = np.array([
    [0, 1, 0, 0],
    [-np.sin(15 * _D2R), 0, np.cos(15 * _D2R), 0],
    [np.cos(15 * _D2R), 0, np.sin(15 * _D2R), _LB],
    [0, 0, 0, 1],
], dtype=float)
BASE_RIGHT = np.array([
    [0, 1, 0, 0],
    [np.sin(15 * _D2R), 0, np.cos(15 * _D2R), 0],
    [np.cos(15 * _D2R), 0, -np.sin(15 * _D2R), _LB],
    [0, 0, 0, 1],
], dtype=float)

# Link-to-link T shape and numerical stability
T_SHAPE = (4, 4, 9)
_EPS_EIG = 1e-10
_EPS_SQRT = 1e-12

# Extension root directory (Robotics_Study/)
_EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))

# Prim paths for Jacobian: base + left 7 links + left tool, right 7 links + right tool (world 4x4 T).
# Correspondence with mdl_ALLEX_Simple:
#   - base: AllexLeft.base / AllexRight.base are FIXED (bT0): from robot base to arm frame 0.
#     USD has Origin_Body only; arm "frame 0" is not a prim — use DH base matrix in code if needed.
#   - links: L1..L7, R1..R7 are world_T_link (DH uses link-to-link T_i-1,i).
#   - tool: AllexLeft.tool / AllexRight.tool are FIXED (7Te). USD has Left/Right_Hand_base → use as EE.
TRANSFORM_PRIM_PATHS: List[Tuple[str, str]] = [
    ("Base (Origin_Body)", "/ALLEX/Origin_Body"),
    ("L1 Shoulder_Pitch", "/ALLEX/L_Shoulder_Pitch_link"),
    ("L2 Shoulder_Roll", "/ALLEX/L_Shoulder_Roll_link"),
    ("L3 Upperarm", "/ALLEX/L_Upperarm_link"),
    ("L4 Elbow", "/ALLEX/L_Elbow_link"),
    ("L5 Forearm", "/ALLEX/L_Forearm_link"),
    ("L6 Wrist_Roll", "/ALLEX/L_Wrist_Roll_link"),
    ("L7 Wrist_Pitch", "/ALLEX/L_Wrist_Pitch_link"),
    ("L_tool (Left_Hand_base)", "/ALLEX/Left_Hand_base"),
    ("R1 Shoulder_Pitch", "/ALLEX/R_Shoulder_Pitch_link"),
    ("R2 Shoulder_Roll", "/ALLEX/R_Shoulder_Roll_link"),
    ("R3 Upperarm", "/ALLEX/R_Upperarm_link"),
    ("R4 Elbow", "/ALLEX/R_Elbow_link"),
    ("R5 Forearm", "/ALLEX/R_Forearm_link"),
    ("R6 Wrist_Roll", "/ALLEX/R_Wrist_Roll_link"),
    ("R7 Wrist_Pitch", "/ALLEX/R_Wrist_Pitch_link"),
    ("R_tool (Right_Hand_base)", "/ALLEX/Right_Hand_base"),
]

# mdl_ALLEX vs USD output:
# - mdl_ALLEX: base (bT0), T01..T67 (link-to-link), tool (7Te). All in DH frames.
# - USD output: world_T for each prim. Frames are link/body frames, not necessarily DH.
# - Base: No prim for "arm frame 0". Use AllexLeft.base / AllexRight.base in code (fixed matrix).
# - Tool: 7Te = inv(world_T_L7) @ world_T_Left_Hand_base (same for right). No USD change needed.
#
# Stored T convention: row-major, translation in row 3 (T[3,:3]). So p_i = bT_i[3,:3], z_i = bT_i[:3,2].

# Pose definitions (same as assignment 1 / assignment 2 problem)
POSES = {
    "pose1": {
        "description": "Zero pose",
        "left_arm": np.zeros(7),
        "right_arm": np.zeros(7),
    },
    "pose2": {
        "description": "Symmetric arm pose",
        "left_arm": np.array([0, np.pi / 4, 0, -np.pi / 2, 0, 0, -np.pi / 4]),
        "right_arm": np.array([0, -np.pi / 4, 0, -np.pi / 2, 0, 0, -np.pi / 4]),
    },
    "pose3": {
        "description": "Asymmetric arm pose",
        "left_arm": np.array([-np.pi / 3, np.pi / 6, -np.pi / 4, -np.pi / 2, 0, -np.pi / 8, 0]),
        "right_arm": np.array([np.pi / 3, -np.pi / 6, np.pi / 4, -np.pi / 2, 0, np.pi / 8, 0]),
    },
}


class Assignment2Scenario:
    """ALLEX Robot Jacobian Scenario - load and hold articulation reference."""

    CONFIG_PATH = os.path.join(_EXT_ROOT, "config", "joint_config.json")

    def __init__(self):
        self._articulation = None
        self._config = None
        self._left_arm_indices: List[int] = []
        self._right_arm_indices: List[int] = []

        if os.path.exists(self.CONFIG_PATH):
            try:
                with open(self.CONFIG_PATH, "r") as f:
                    self._config = json.load(f)
                print(f"[Assignment2] Config loaded: {self.CONFIG_PATH}")
            except Exception as e:
                print(f"[Assignment2] Config load failed: {e}")

    def setup(self, articulation):
        """Setup scenario with robot articulation."""
        self._articulation = articulation
        if not self._articulation or not self._config:
            return

        dof_names = self._articulation.dof_names
        allex_cfg = self._config.get("ALLEX", {})
        left_names = allex_cfg.get("left_arm", {}).get("joint_names", [])
        right_names = allex_cfg.get("right_arm", {}).get("joint_names", [])

        self._left_arm_indices = [dof_names.index(n) for n in left_names if n in dof_names]
        self._right_arm_indices = [dof_names.index(n) for n in right_names if n in dof_names]

        print(f"[Assignment2] Setup complete | DOF: {self._articulation.num_dof}")

    def apply_pose(self, articulation: Any, pose_name: str) -> bool:
        """Apply predefined pose (pose1, pose2, pose3) to articulation. Returns True on success."""
        from isaacsim.core.utils.types import ArticulationAction

        pose = POSES.get(pose_name)
        if not pose or not articulation:
            return False
        positions = articulation.get_joint_positions()
        if positions is None:
            return False
        new_pos = positions.copy()
        for i, idx in enumerate(self._left_arm_indices):
            if i < len(pose["left_arm"]):
                new_pos[idx] = pose["left_arm"][i]
        for i, idx in enumerate(self._right_arm_indices):
            if i < len(pose["right_arm"]):
                new_pos[idx] = pose["right_arm"][i]
        articulation.apply_action(ArticulationAction(joint_positions=new_pos))
        return True

    def get_link_transforms(
        self, stage: Any, time_code: Optional[Any] = None
    ) -> List[Tuple[str, List[List[float]]]]:
        """
        Get 4x4 world transform matrix for each Jacobian-relevant prim at the same time.
        Use current stage time so that inv(W_origin)@W_hand is invariant under rigid body move.
        Returns list of (display_name, 4x4 row-major matrix as list of 4 lists).
        """
        result: List[Tuple[str, List[List[float]]]] = []
        if not stage:
            return result
        t = time_code if time_code is not None else Usd.TimeCode.Default()
        for label, path in TRANSFORM_PRIM_PATHS:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                result.append((label, []))
                continue
            xform = UsdGeom.Xformable(prim)
            mat = xform.ComputeLocalToWorldTransform(t)
            rows = [
                [mat.GetRow(i)[j] for j in range(4)]
                for i in range(4)
            ]
            result.append((label, rows))
        return result

    def world_to_link_to_link(
        self, world_transforms: List[Tuple[str, List[List[float]]]]
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Convert world-to-link transforms to link-to-link T 4x4x9 for J_SerialLink.
        world_transforms: from get_link_transforms (order: Base, L1..L7, L_tool, R1..R7, R_tool).
        Returns (T_left, T_right), each shape (4,4,9): T(:,:,1)=bTo, 2=oT1, ..., 8=6T7, 9=7Tt.
        """
        if len(world_transforms) < 17:
            return None, None

        def to_mat(item: Tuple[str, List[List[float]]]) -> Optional[np.ndarray]:
            _, rows = item
            if not rows or len(rows) != 4:
                return None
            return np.array(rows, dtype=float)

        W_base = to_mat(world_transforms[0])
        W_L = [to_mat(world_transforms[i]) for i in range(1, 9)]   # L1..L7, L_tool
        W_R = [to_mat(world_transforms[i]) for i in range(9, 17)]  # R1..R7, R_tool
        if W_base is None or any(w is None for w in W_L) or any(w is None for w in W_R):
            return None, None

        # T(:,:,1)=bTo (Origin_Body→arm frame0), 2=oT1, ..., 9=7Tt. Chain bT = Origin_Body → tool.
        T_left = np.zeros(T_SHAPE)
        T_left[:, :, 0] = BASE_LEFT
        W_f0_left = W_base @ BASE_LEFT
        T_left[:, :, 1] = np.linalg.inv(W_f0_left) @ W_L[0]
        for i in range(1, 7):
            T_left[:, :, i + 1] = np.linalg.inv(W_L[i - 1]) @ W_L[i]
        T_left[:, :, 8] = np.linalg.inv(W_L[6]) @ W_L[7]

        T_right = np.zeros(T_SHAPE)
        T_right[:, :, 0] = BASE_RIGHT
        W_f0_right = W_base @ BASE_RIGHT
        T_right[:, :, 1] = np.linalg.inv(W_f0_right) @ W_R[0]
        for i in range(1, 7):
            T_right[:, :, i + 1] = np.linalg.inv(W_R[i - 1]) @ W_R[i]
        T_right[:, :, 8] = np.linalg.inv(W_R[6]) @ W_R[7]

        return T_left, T_right

    @staticmethod
    def jacobian_serial_link(T: np.ndarray) -> np.ndarray:
        """
        J_SerialLink(T): compute 6x7 geometric Jacobian from link-to-link T 4x4x9.
        T: (4,4,9), T(:,:,1)=bTo, ..., T(:,:,9)=7Tt. Convention: translation in row 3.
        Returns J: (6,7), [v; w] = J * qd (linear and angular velocity of EE).
        """
        if T.shape != T_SHAPE:
            return np.zeros((6, 7))
        # Cumulative: bT_list[k] = T(:,:,1)*...*T(:,:,k) => frame 0 at k=1, frame i at k=i+1, tool at k=9
        bT = np.eye(4)
        bT_list = [bT.copy()]
        for k in range(9):
            bT = bT @ T[:, :, k]
            bT_list.append(bT.copy())
        p_ee = bT_list[9][3, :3]
        J = np.zeros((6, 7))
        for i in range(7):
            # Joint i+1: use frame i+1 (after joint i+1); bT_list[i+2] = frame 1..7
            bT_i = bT_list[i + 2]
            p_i = bT_i[3, :3]
            z_i = bT_i[:3, 2]
            J[:3, i] = np.cross(z_i, p_ee - p_i)
            J[3:, i] = z_i
        return J

    @staticmethod
    def _ellipsoid_core(T: np.ndarray) -> Optional[Tuple]:
        """Compute p_ee, E_vel, E_force and eigens. Returns None if T shape invalid."""
        if T.shape != T_SHAPE:
            return None
        bT = np.eye(4)
        for k in range(9):
            bT = bT @ T[:, :, k]
        p_ee = bT[3, :3].astype(float)
        J = Assignment2Scenario.jacobian_serial_link(T)
        Jv = J[:3, :]
        E_vel = Jv @ Jv.T
        E_force = np.linalg.inv(E_vel + _EPS_EIG * np.eye(3))
        ev_vel, evec_vel = np.linalg.eigh(E_vel)
        ev_frc, evec_frc = np.linalg.eigh(E_force)
        return (p_ee, ev_vel, evec_vel, ev_frc, evec_frc)

    @staticmethod
    def ellipsoid_principal_axes_from_T(
        T: np.ndarray,
        scale_vel: float = 0.15,
        scale_force: float = 0.08,
        which: str = "both",
    ) -> List[Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float, float]]]:
        """
        Velocity and/or force manipulability ellipsoid principal axes at Hand Base.
        which: "both" | "velocity" | "force". Returns (start_pt, end_pt, color_rgba) for draw_lines.
        """
        core = Assignment2Scenario._ellipsoid_core(T)
        if core is None:
            return []
        p_ee, ev_vel, evec_vel, ev_frc, evec_frc = core
        segments = []
        if which in ("both", "velocity"):
            lengths = scale_vel * np.sqrt(np.maximum(ev_vel, _EPS_SQRT))
            for i in range(3):
                end = (p_ee + lengths[i] * evec_vel[:, i]).astype(float)
                segments.append((tuple(p_ee), tuple(end), (0.0, 0.8, 0.2, 1.0)))
        if which in ("both", "force"):
            lengths = scale_force * np.sqrt(np.maximum(ev_frc, _EPS_SQRT))
            for i in range(3):
                end = (p_ee + lengths[i] * evec_frc[:, i]).astype(float)
                segments.append((tuple(p_ee), tuple(end), (0.9, 0.2, 0.2, 1.0)))
        return segments

    @staticmethod
    def ellipsoid_surface_points_from_T(
        T: np.ndarray,
        which: str,
        scale_vel: float = 0.15,
        scale_force: float = 0.08,
        n_theta: int = 64,
        n_phi: int = 32,
    ) -> List[Tuple[float, float, float]]:
        """
        Sample points on ellipsoid surface in Origin_Body frame for draw_points fill.
        which: "velocity" | "force". Uses same semi-axes as ellipsoid_principal_axes_from_T.
        """
        core = Assignment2Scenario._ellipsoid_core(T)
        if core is None or which not in ("velocity", "force"):
            return []
        p_ee, ev_vel, evec_vel, ev_frc, evec_frc = core
        if which == "velocity":
            a, b, c = scale_vel * np.sqrt(np.maximum(ev_vel, _EPS_SQRT))
            R_axes = evec_vel
        else:
            a, b, c = scale_force * np.sqrt(np.maximum(ev_frc, _EPS_SQRT))
            R_axes = evec_frc
        points = []
        for i in range(n_phi + 1):
            phi = -0.5 * np.pi + (np.pi * i) / n_phi
            for j in range(n_theta):
                theta = 2.0 * np.pi * j / n_theta
                # Principal frame: ellipsoid (a*cos(phi)*cos(theta), b*cos(phi)*sin(theta), c*sin(phi))
                xp = a * np.cos(phi) * np.cos(theta)
                yp = b * np.cos(phi) * np.sin(theta)
                zp = c * np.sin(phi)
                p_body = p_ee + R_axes @ np.array([xp, yp, zp])
                points.append(tuple(p_body.astype(float)))
        return points

    @staticmethod
    def ellipsoid_axes_data(T: np.ndarray) -> Optional[dict]:
        """
        Ellipsoid data in Origin_Body frame for terminal output.
        Keys: p_ee, vel_axes, vel_lengths, force_axes, force_lengths.
        """
        core = Assignment2Scenario._ellipsoid_core(T)
        if core is None:
            return None
        p_ee, ev_vel, evec_vel, ev_frc, evec_frc = core
        return {
            "p_ee": p_ee,
            "vel_axes": evec_vel,
            "vel_lengths": np.sqrt(np.maximum(ev_vel, _EPS_SQRT)),
            "force_axes": evec_frc,
            "force_lengths": np.sqrt(np.maximum(ev_frc, _EPS_SQRT)),
        }

    def teardown(self):
        """Cleanup scenario resources."""
        self._articulation = None
        self._left_arm_indices.clear()
        self._right_arm_indices.clear()
