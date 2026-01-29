# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""
Orientation Trajectory Scenario

Three trajectory generation methods:
1. Euler (ZYZ): tr2eul -> jtraj -> eul2r
2. Angle-Axis: tr2angvec -> jtraj -> angvec2r
3. Quaternion: SLERP interpolation

References:
- jtraj: https://www.petercorke.com/RTB/r9/html/jtraj.html
- ctraj: https://www.petercorke.com/RTB/r9/html/ctraj.html
"""

import os
import numpy as np
from typing import Optional, Tuple

from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from pxr import Gf, UsdGeom

_EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))


# ═══════════════════════════════════════════════════════════════════════════════
# Trajectory Classes
# ═══════════════════════════════════════════════════════════════════════════════
class SmoothTrajectory:
    """
    S-curve trajectory with adjustable smoothness (jtraj style).
    Formula: s_out = s^n / (s^n + (1-s)^n)
    
    n=1: linear | n=3: default | n=10: aggressive
    """
    def __init__(self, q0: float, qf: float, tf: float, n: float = 3.0):
        self.q0, self.qf, self.tf, self.n = q0, qf, tf, n
        self.dq = qf - q0

    def __call__(self, t: float) -> float:
        if self.tf < 1e-6:
            return self.qf
        s = np.clip(t / self.tf, 0.0, 1.0)
        if 0 < s < 1:
            s = s**self.n / (s**self.n + (1-s)**self.n)
        return self.q0 + self.dq * s


class LinearTrajectory:
    """Linear trajectory: s = t/tf (no acceleration)."""
    def __init__(self, q0: float, qf: float, tf: float):
        self.q0, self.qf, self.tf = q0, qf, tf
        self.dq = qf - q0

    def __call__(self, t: float) -> float:
        if self.tf < 1e-6:
            return self.qf
        return self.q0 + self.dq * np.clip(t / self.tf, 0.0, 1.0)


class TrapezoidalTrajectory:
    """
    LSPB trajectory (ctraj style).
    Trapezoidal velocity: accelerate -> cruise -> decelerate.
    """
    def __init__(self, q0: float, qf: float, tf: float):
        self.q0, self.qf, self.tf = q0, qf, tf
        self.dq = qf - q0
        
        if tf < 1e-6:
            self.tb = self.v = self.a = 0.0
            return
        
        # Symmetric trapezoid: blend time = tf/3
        self.tb = tf / 3.0
        self.v = self.dq / (tf - self.tb)
        self.a = self.v / self.tb if self.tb > 1e-6 else 0.0

    def __call__(self, t: float) -> float:
        if self.tf < 1e-6:
            return self.qf
        t = np.clip(t, 0.0, self.tf)
        
        if t <= self.tb:                          # Acceleration
            return self.q0 + 0.5 * self.a * t**2
        elif t <= self.tf - self.tb:              # Cruise
            return self.q0 + 0.5 * self.a * self.tb**2 + self.v * (t - self.tb)
        else:                                     # Deceleration
            return self.qf - 0.5 * self.a * (self.tf - t)**2


# ═══════════════════════════════════════════════════════════════════════════════
# Rotation Utilities (Static Functions)
# ═══════════════════════════════════════════════════════════════════════════════
def rotx(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0], [0,c,-s], [0,s,c]])

def roty(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,0,s], [0,1,0], [-s,0,c]])

def rotz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0], [s,c,0], [0,0,1]])

def tr2eul(R: np.ndarray) -> Tuple[float, float, float]:
    """Rotation matrix -> ZYZ Euler angles (phi, theta, psi)."""
    if abs(R[2,2]) < 1 - 1e-10:
        return (np.arctan2(R[1,2], R[0,2]),
                np.arctan2(np.sqrt(R[0,2]**2 + R[1,2]**2), R[2,2]),
                np.arctan2(R[2,1], -R[2,0]))
    elif R[2,2] > 0:
        return (0.0, 0.0, np.arctan2(-R[0,1], R[0,0]))
    else:
        return (0.0, np.pi, np.arctan2(R[0,1], -R[0,0]))

def eul2r(phi: float, theta: float, psi: float) -> np.ndarray:
    """ZYZ Euler angles -> Rotation matrix."""
    return rotz(phi) @ roty(theta) @ rotz(psi)

def tr2angvec(R: np.ndarray) -> Tuple[float, np.ndarray]:
    """Rotation matrix -> Angle-axis (theta, v)."""
    theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
    
    if abs(theta) < 1e-10:
        return 0.0, np.array([0.0, 0.0, 1.0])
    if abs(theta - np.pi) < 1e-10:
        vvT = (R + np.eye(3)) / 2
        v = np.sqrt(np.maximum(np.diag(vvT), 0))
        if vvT[0,1] < 0: v[1] = -v[1]
        if vvT[0,2] < 0: v[2] = -v[2]
        return theta, v / np.linalg.norm(v)
    
    v = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]) / (2*np.sin(theta))
    return float(theta), v / np.linalg.norm(v)

def angvec2r(theta: float, v: np.ndarray) -> np.ndarray:
    """Angle-axis -> Rotation matrix (Rodrigues' formula)."""
    if abs(theta) < 1e-10:
        return np.eye(3)
    v = v / np.linalg.norm(v)
    K = np.array([[0,-v[2],v[1]], [v[2],0,-v[0]], [-v[1],v[0],0]])
    return np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K@K)

def rot2euler_xyz(R: np.ndarray) -> Tuple[float, float, float]:
    """Rotation matrix -> XYZ Euler angles (for USD)."""
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    if sy > 1e-6:
        return (np.arctan2(R[2,1], R[2,2]), np.arctan2(-R[2,0], sy), np.arctan2(R[1,0], R[0,0]))
    return (np.arctan2(-R[1,2], R[1,1]), np.arctan2(-R[2,0], sy), 0.0)

def rot2quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> Quaternion [w, x, y, z]."""
    tr = np.trace(R)
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        q = np.array([0.25/s, (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s])
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        q = np.array([(R[2,1]-R[1,2])/s, 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s])
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        q = np.array([(R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s])
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        q = np.array([(R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s])
    return q / np.linalg.norm(q)

def quat2rot(q: np.ndarray) -> np.ndarray:
    """Quaternion [w, x, y, z] -> Rotation matrix."""
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
    ])

def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between quaternions."""
    q0, q1 = q0/np.linalg.norm(q0), q1/np.linalg.norm(q1)
    dot = np.dot(q0, q1)
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        return (q0 + t*(q1-q0)) / np.linalg.norm(q0 + t*(q1-q0))
    theta0 = np.arccos(dot)
    q2 = (q1 - q0*dot) / np.linalg.norm(q1 - q0*dot)
    return q0*np.cos(theta0*t) + q2*np.sin(theta0*t)


# ═══════════════════════════════════════════════════════════════════════════════
# Main Scenario Class
# ═══════════════════════════════════════════════════════════════════════════════
class OrientationTrajectoryScenario:
    """Orientation trajectory using Euler, Angle-Axis, and Quaternion methods."""
    
    XYZ_USD = os.path.join(_EXT_ROOT, "asset", "XYZ.usd")
    XYZ_PRIM_PATH = "/World/XYZ"
    XYZ_POSITION = (1.0, 1.0, 1.5)
    TIP_PATHS = [
        ("/World/XYZ/Frame/X_tip/x_tip_point", (1.0, 0.0, 0.0, 1.0)),  # Red
        ("/World/XYZ/Frame/Y_tip/y_tip_point", (0.0, 1.0, 0.0, 1.0)),  # Green
        ("/World/XYZ/Frame/Z_tip/z_tip_point", (0.0, 0.5, 1.0, 1.0)),  # Blue
    ]

    def __init__(self):
        self._reset_state()

    def _reset_state(self):
        self._xyz_prim = None
        self._is_ready = False
        self._is_executing = False
        self._elapsed = 0.0
        self._tf = 5.0
        self._mode = "euler"
        
        self._Ri, self._Rf = np.eye(3), rotx(-np.pi/4)
        self._euler_i, self._euler_f = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        self._euler_traj = None
        
        self._axis, self._angle_final = np.array([0.0, 0.0, 1.0]), 0.0
        self._angle_traj = None
        
        self._quat_i, self._quat_f = np.array([1.0, 0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0])
        self._quat_interp = "linspace"
        self._s_traj = None

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
        traj = {"euler": self._euler_traj, "angle_axis": self._angle_traj, "quaternion": self._s_traj}
        return self._tf if traj.get(self._mode) else 0.0

    # ─────────────────────────────────────────────────────────────────────────
    # Setup / Teardown
    # ─────────────────────────────────────────────────────────────────────────
    def setup(self, articulation=None):
        self._is_ready = True

    def teardown(self):
        self._reset_state()

    # ─────────────────────────────────────────────────────────────────────────
    # Rotation Setters/Getters
    # ─────────────────────────────────────────────────────────────────────────
    def set_Ri_rotx(self, deg: float):
        self._Ri = rotx(np.radians(deg))

    def set_Rf_rotx(self, deg: float):
        self._Rf = rotx(np.radians(deg))

    def get_Ri(self) -> np.ndarray:
        return self._Ri.copy()

    def get_Rf(self) -> np.ndarray:
        return self._Rf.copy()

    def get_euler_i_deg(self) -> Tuple[float, float, float]:
        return tuple(np.degrees(x) for x in self._euler_i)

    def get_euler_f_deg(self) -> Tuple[float, float, float]:
        return tuple(np.degrees(x) for x in self._euler_f)

    # ─────────────────────────────────────────────────────────────────────────
    # USD Operations
    # ─────────────────────────────────────────────────────────────────────────
    def spawn_xyz(self) -> bool:
        stage = get_current_stage()
        if not stage or not os.path.exists(self.XYZ_USD):
            return False
        
        prim = stage.GetPrimAtPath(self.XYZ_PRIM_PATH)
        if prim.IsValid():
            stage.RemovePrim(self.XYZ_PRIM_PATH)
        
        self._xyz_prim = add_reference_to_stage(self.XYZ_USD, self.XYZ_PRIM_PATH)
        if self._xyz_prim:
            self._apply_rotation(self._Ri)
        return bool(self._xyz_prim)

    def _apply_rotation(self, R: np.ndarray):
        stage = get_current_stage()
        if not stage or not self._xyz_prim:
            return
        prim = stage.GetPrimAtPath(self.XYZ_PRIM_PATH)
        if not prim.IsValid():
            return
        
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*self.XYZ_POSITION))
        r, p, y = rot2euler_xyz(R)
        xform.AddRotateXYZOp().Set(Gf.Vec3f(np.degrees(r), np.degrees(p), np.degrees(y)))

    # ─────────────────────────────────────────────────────────────────────────
    # Trajectory Generation
    # ─────────────────────────────────────────────────────────────────────────
    def generate_euler_trajectory(self, tf: float = 5.0) -> dict:
        if tf <= 0:
            return {"success": False, "message": "tf must be > 0"}
        
        self._tf, self._mode = tf, "euler"
        self._euler_i, self._euler_f = tr2eul(self._Ri), tr2eul(self._Rf)
        self._euler_traj = tuple(SmoothTrajectory(i, f, tf) for i, f in zip(self._euler_i, self._euler_f))
        self._elapsed = 0.0
        return {"success": True}

    def generate_angle_axis_trajectory(self, tf: float = 5.0) -> dict:
        if tf <= 0:
            return {"success": False, "message": "tf must be > 0"}
        
        self._tf, self._mode = tf, "angle_axis"
        self._angle_final, self._axis = tr2angvec(self._Ri.T @ self._Rf)
        self._angle_traj = SmoothTrajectory(0.0, self._angle_final, tf)
        self._elapsed = 0.0
        return {"success": True, "angle_deg": np.degrees(self._angle_final), "axis": self._axis.tolist()}

    def generate_quaternion_trajectory(self, tf: float = 5.0, interp: str = "linspace", n: float = 3.0) -> dict:
        if tf <= 0:
            return {"success": False, "message": "tf must be > 0"}
        
        self._tf, self._mode, self._quat_interp = tf, "quaternion", interp
        self._quat_i, self._quat_f = rot2quat(self._Ri), rot2quat(self._Rf)
        
        traj_map = {
            "linspace": LinearTrajectory,
            "jtraj": lambda q0, qf, tf: SmoothTrajectory(q0, qf, tf, n),
            "ctraj": TrapezoidalTrajectory,
        }
        self._s_traj = traj_map.get(interp, LinearTrajectory)(0.0, 1.0, tf)
        self._elapsed = 0.0
        return {"success": True, "interp_type": interp, "n": n if interp == "jtraj" else None}

    # ─────────────────────────────────────────────────────────────────────────
    # Execution
    # ─────────────────────────────────────────────────────────────────────────
    def start_execution(self):
        traj = {"euler": self._euler_traj, "angle_axis": self._angle_traj, "quaternion": self._s_traj}
        if not traj.get(self._mode):
            return
        self._apply_rotation(self._Ri)
        self.clear_debug_draw()
        self._is_executing, self._elapsed = True, 0.0

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
        
        self._elapsed = min(self._elapsed + dt, self._tf)
        if self._elapsed >= self._tf:
            self._is_executing = False
        
        # Compute rotation
        if self._mode == "euler" and self._euler_traj:
            R = eul2r(*[t(self._elapsed) for t in self._euler_traj])
        elif self._mode == "angle_axis" and self._angle_traj:
            R = self._Ri @ angvec2r(self._angle_traj(self._elapsed), self._axis)
        elif self._mode == "quaternion" and self._s_traj:
            R = quat2rot(slerp(self._quat_i, self._quat_f, self._s_traj(self._elapsed)))
        else:
            return
        
        self._apply_rotation(R)
        self._draw_tips()

    def _draw_tips(self):
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
        if self._mode == "euler" and self._euler_traj:
            angles = [t(self._elapsed) for t in self._euler_traj]
            return {"mode": "euler", "time": self._elapsed, "phi": angles[0], "theta": angles[1], "psi": angles[2]}
        elif self._mode == "angle_axis" and self._angle_traj:
            return {"mode": "angle_axis", "time": self._elapsed, 
                    "angle": self._angle_traj(self._elapsed), "angle_final": self._angle_final}
        elif self._mode == "quaternion" and self._s_traj:
            return {"mode": "quaternion", "time": self._elapsed, 
                    "s": self._s_traj(self._elapsed), "interp_type": self._quat_interp}
        return None

    # Static method aliases for external access
    @staticmethod
    def tr2eul(R): return tr2eul(R)
    @staticmethod
    def tr2angvec(R): return tr2angvec(R)
    @staticmethod
    def rot2quat(R): return rot2quat(R)
