# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Orientation Trajectory Scenario - Euler Angles & Angle-Axis Methods

Two trajectory generation methods:
1. Euler (ZYZ): tr2eul -> jtraj -> eul2r
2. Angle-Axis: tr2angvec -> jtraj -> angvec2r (shortest path, no singularity)
"""

import os
import numpy as np
from typing import Optional, Tuple

from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from pxr import Gf, UsdGeom

_EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))


class QuinticPolynomialTrajectory:
    """5th order polynomial trajectory (jtraj style) with zero velocity/acceleration at boundaries."""

    def __init__(self, qi: float, qf: float, tf: float):
        self.qi, self.qf, self.tf = qi, qf, tf
        self.h = qf - qi

    def get_position(self, t: float) -> float:
        """Get position at time t using normalized quintic polynomial."""
        if self.tf < 1e-6:
            return self.qf
        s = np.clip(t, 0.0, self.tf) / self.tf
        return self.qi + self.h * (10*s**3 - 15*s**4 + 6*s**5)


class OrientationTrajectoryScenario:
    """Orientation trajectory generation using Euler angles and Angle-Axis methods."""
    
    XYZ_USD = os.path.join(_EXT_ROOT, "asset", "XYZ.usd")
    XYZ_PRIM_PATH = "/World/XYZ"
    XYZ_POSITION = (1.0, 1.0, 1.5)
    
    # Tip paths for debug draw (X: Red, Y: Green, Z: Blue)
    TIP_PATHS = [
        ("/World/XYZ/Frame/X_tip/x_tip_point", (1.0, 0.0, 0.0, 1.0)),
        ("/World/XYZ/Frame/Y_tip/y_tip_point", (0.0, 1.0, 0.0, 1.0)),
        ("/World/XYZ/Frame/Z_tip/z_tip_point", (0.0, 0.5, 1.0, 1.0)),
    ]

    def __init__(self):
        self._xyz_prim = None
        self._is_ready = False
        self._is_executing = False
        self._elapsed_time = 0.0
        self._tf = 5.0
        
        # Rotation matrices
        self._Ri = np.eye(3)
        self._Rf = self.rotx(-np.pi/4)
        
        # Euler trajectories
        self._euler_traj: Optional[Tuple[QuinticPolynomialTrajectory, ...]] = None
        self._euler_i = (0.0, 0.0, 0.0)
        self._euler_f = (0.0, 0.0, 0.0)
        
        # Angle-Axis trajectory
        self._angle_traj: Optional[QuinticPolynomialTrajectory] = None
        self._axis = np.array([0.0, 0.0, 1.0])
        self._angle_final = 0.0
        
        # Mode: "euler" or "angle_axis"
        self._mode = "euler"

    # ─────────────────────────────────────────────────────────────────────────
    # Properties
    # ─────────────────────────────────────────────────────────────────────────
    @property
    def is_ready(self) -> bool:
        return self._is_ready

    @property
    def is_executing(self) -> bool:
        return self._is_executing

    @property
    def trajectory_duration(self) -> float:
        has_traj = self._euler_traj if self._mode == "euler" else self._angle_traj
        return self._tf if has_traj else 0.0

    # ─────────────────────────────────────────────────────────────────────────
    # Rotation Matrix Utilities
    # ─────────────────────────────────────────────────────────────────────────
    @staticmethod
    def rotx(angle: float) -> np.ndarray:
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    @staticmethod
    def roty(angle: float) -> np.ndarray:
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    @staticmethod
    def rotz(angle: float) -> np.ndarray:
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    @staticmethod
    def tr2eul(R: np.ndarray) -> Tuple[float, float, float]:
        """Convert rotation matrix to ZYZ Euler angles (phi, theta, psi)."""
        if abs(R[2, 2]) < 1 - 1e-10:
            theta = np.arctan2(np.sqrt(R[0, 2]**2 + R[1, 2]**2), R[2, 2])
            phi = np.arctan2(R[1, 2], R[0, 2])
            psi = np.arctan2(R[2, 1], -R[2, 0])
        elif R[2, 2] > 0:  # theta ~ 0
            theta, phi = 0.0, 0.0
            psi = np.arctan2(-R[0, 1], R[0, 0])
        else:  # theta ~ pi
            theta, phi = np.pi, 0.0
            psi = np.arctan2(R[0, 1], -R[0, 0])
        return (phi, theta, psi)

    @staticmethod
    def eul2r(phi: float, theta: float, psi: float) -> np.ndarray:
        """Convert ZYZ Euler angles to rotation matrix."""
        return OrientationTrajectoryScenario.rotz(phi) @ \
               OrientationTrajectoryScenario.roty(theta) @ \
               OrientationTrajectoryScenario.rotz(psi)

    @staticmethod
    def tr2angvec(R: np.ndarray) -> Tuple[float, np.ndarray]:
        """Convert rotation matrix to angle-axis (theta, v)."""
        theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
        
        if abs(theta) < 1e-10:
            return 0.0, np.array([0.0, 0.0, 1.0])
        
        if abs(theta - np.pi) < 1e-10:
            vvT = (R + np.eye(3)) / 2
            v = np.sqrt(np.maximum(np.diag(vvT), 0))
            if vvT[0, 1] < 0: v[1] = -v[1]
            if vvT[0, 2] < 0: v[2] = -v[2]
            return theta, v / np.linalg.norm(v)
        
        v = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]) / (2*np.sin(theta))
        return float(theta), v / np.linalg.norm(v)

    @staticmethod
    def angvec2r(theta: float, v: np.ndarray) -> np.ndarray:
        """Convert angle-axis to rotation matrix using Rodrigues' formula."""
        if abs(theta) < 1e-10:
            return np.eye(3)
        v = v / np.linalg.norm(v)
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K@K)

    @staticmethod
    def _rot_to_euler_xyz(R: np.ndarray) -> Tuple[float, float, float]:
        """Convert rotation matrix to XYZ Euler angles for USD."""
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        if sy > 1e-6:
            return np.arctan2(R[2,1], R[2,2]), np.arctan2(-R[2,0], sy), np.arctan2(R[1,0], R[0,0])
        return np.arctan2(-R[1,2], R[1,1]), np.arctan2(-R[2,0], sy), 0.0

    # ─────────────────────────────────────────────────────────────────────────
    # Setup / Teardown
    # ─────────────────────────────────────────────────────────────────────────
    def setup(self, articulation=None):
        self._is_ready = True

    def teardown(self):
        self._xyz_prim = None
        self._is_ready = False
        self._is_executing = False
        self._euler_traj = None
        self._angle_traj = None
        self._mode = "euler"

    # ─────────────────────────────────────────────────────────────────────────
    # Rotation Matrix Setters/Getters
    # ─────────────────────────────────────────────────────────────────────────
    def set_Ri_rotx(self, angle_deg: float):
        """Set Ri = rotx(angle)"""
        self._Ri = self.rotx(np.radians(angle_deg))

    def set_Rf_rotx(self, angle_deg: float):
        self._Rf = self.rotx(np.radians(angle_deg))

    def get_Ri(self) -> np.ndarray:
        return self._Ri.copy()

    def get_Rf(self) -> np.ndarray:
        return self._Rf.copy()

    def get_euler_i_deg(self) -> Tuple[float, float, float]:
        return tuple(np.degrees(x) for x in self._euler_i)

    def get_euler_f_deg(self) -> Tuple[float, float, float]:
        return tuple(np.degrees(x) for x in self._euler_f)

    # ─────────────────────────────────────────────────────────────────────────
    # XYZ USD Spawning
    # ─────────────────────────────────────────────────────────────────────────
    def spawn_xyz(self) -> bool:
        stage = get_current_stage()
        if not stage or not os.path.exists(self.XYZ_USD):
            return False

        existing = stage.GetPrimAtPath(self.XYZ_PRIM_PATH)
        if existing.IsValid():
            stage.RemovePrim(self.XYZ_PRIM_PATH)

        self._xyz_prim = add_reference_to_stage(self.XYZ_USD, self.XYZ_PRIM_PATH)
        if not self._xyz_prim:
            return False

        self._apply_rotation(self._Ri)
        return True

    def _apply_rotation(self, R: np.ndarray):
        """Apply rotation matrix to XYZ prim."""
        stage = get_current_stage()
        if not stage or not self._xyz_prim:
            return
        
        prim = stage.GetPrimAtPath(self.XYZ_PRIM_PATH)
        if not prim.IsValid():
            return
        
        xformable = UsdGeom.Xformable(prim)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(Gf.Vec3d(*self.XYZ_POSITION))
        
        roll, pitch, yaw = self._rot_to_euler_xyz(R)
        xformable.AddRotateXYZOp().Set(Gf.Vec3f(np.degrees(roll), np.degrees(pitch), np.degrees(yaw)))

    # ─────────────────────────────────────────────────────────────────────────
    # Trajectory Generation
    # ─────────────────────────────────────────────────────────────────────────
    def generate_euler_trajectory(self, tf: float = 5.0) -> dict:
        """Generate Euler angle trajectory using quintic polynomial."""
        if tf <= 0:
            return {"success": False, "message": "tf must be > 0"}

        self._tf = tf
        self._mode = "euler"
        
        self._euler_i = self.tr2eul(self._Ri)
        self._euler_f = self.tr2eul(self._Rf)
        
        self._euler_traj = tuple(
            QuinticPolynomialTrajectory(ei, ef, tf) 
            for ei, ef in zip(self._euler_i, self._euler_f)
        )
        self._elapsed_time = 0.0
        
        return {"success": True, "message": "OK"}

    def generate_angle_axis_trajectory(self, tf: float = 5.0) -> dict:
        """Generate Angle-Axis trajectory (shortest path, no singularity)."""
        if tf <= 0:
            return {"success": False, "message": "tf must be > 0"}

        self._tf = tf
        self._mode = "angle_axis"
        
        Rif = self._Ri.T @ self._Rf
        self._angle_final, self._axis = self.tr2angvec(Rif)
        self._angle_traj = QuinticPolynomialTrajectory(0.0, self._angle_final, tf)
        self._elapsed_time = 0.0
        
        return {
            "success": True,
            "message": "OK",
            "angle_deg": np.degrees(self._angle_final),
            "axis": self._axis.tolist()
        }

    # ─────────────────────────────────────────────────────────────────────────
    # Trajectory Execution
    # ─────────────────────────────────────────────────────────────────────────
    def start_execution(self):
        has_traj = self._euler_traj if self._mode == "euler" else self._angle_traj
        if not has_traj:
            return
        
        self._apply_rotation(self._Ri)
        self.clear_debug_draw()
        self._is_executing = True
        self._elapsed_time = 0.0

    def stop_execution(self):
        self._is_executing = False

    def clear_debug_draw(self):
        try:
            from isaacsim.util.debug_draw import _debug_draw
            _debug_draw.acquire_debug_draw_interface().clear_points()
        except Exception:
            pass

    def update(self, dt: float):
        if not self._is_executing:
            return
        
        self._elapsed_time += dt
        if self._elapsed_time >= self._tf:
            self._elapsed_time = self._tf
            self._is_executing = False
        
        # Compute current rotation
        if self._mode == "euler" and self._euler_traj:
            angles = [traj.get_position(self._elapsed_time) for traj in self._euler_traj]
            R = self.eul2r(*angles)
        elif self._mode == "angle_axis" and self._angle_traj:
            theta = self._angle_traj.get_position(self._elapsed_time)
            R = self._Ri @ self.angvec2r(theta, self._axis)
        else:
            return
        
        self._apply_rotation(R)
        self._draw_tip_trajectories()

    def _draw_tip_trajectories(self):
        """Draw debug points for X, Y, Z tip trajectories."""
        try:
            from isaacsim.util.debug_draw import _debug_draw
            stage = get_current_stage()
            if not stage:
                return
            
            draw = _debug_draw.acquire_debug_draw_interface()
            for path, color in self.TIP_PATHS:
                prim = stage.GetPrimAtPath(path)
                if prim.IsValid():
                    pos = tuple(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation())
                    draw.draw_points([pos], [color], [20])
        except Exception:
            pass

    def get_state(self) -> Optional[dict]:
        """Get current trajectory state."""
        if self._mode == "euler" and self._euler_traj:
            angles = [traj.get_position(self._elapsed_time) for traj in self._euler_traj]
            return {
                "mode": "euler",
                "time": self._elapsed_time,
                "phi": angles[0],
                "theta": angles[1],
                "psi": angles[2],
            }
        elif self._mode == "angle_axis" and self._angle_traj:
            return {
                "mode": "angle_axis",
                "time": self._elapsed_time,
                "angle": self._angle_traj.get_position(self._elapsed_time),
                "angle_final": self._angle_final,
                "axis": self._axis.copy(),
            }
        return None
