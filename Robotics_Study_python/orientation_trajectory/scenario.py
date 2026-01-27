# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Orientation Trajectory Scenario using Roll-Pitch-Yaw (Euler Angles)

Orientation 궤적 생성 - Roll-Pitch-Yaw표현법 이용 (MATLAB 스타일)

%시작 및 도착 rotation matrix
Ri = eye(3);
Rf = rotx(-pi/4);

%Ri와 Rf를 Euler angle로 변환
Euler_i = tr2eul(Ri);
Euler_f = tr2eul(Rf);

%Euler angle의 궤적 생성
Euler_traj = jtraj(Euler_i, Euler_f, 20);

%Euler angle의 궤적을 Rotation matrix의 궤적으로 변환
R = eul2r(Euler_traj(:,1), Euler_traj(:,2), Euler_traj(:,3));

STEP 1: Ri, Rf의 Euler angle인 (φi, θi, ψi)와 (φf, θf, ψf) 구함
STEP 2: (φi, θi, ψi) → (φf, θf, ψf)의 궤적인 (φ(t), θ(t), ψ(t)) 생성 (5차 다항식)
STEP 3: 임의 t에 대한 Euler angle인 φ(t), θ(t), ψ(t)를 rotation matrix로 변환하여 R(t) 구함
"""

import os
import numpy as np
from typing import Optional, Tuple

from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from pxr import Gf, UsdGeom

_EXT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))


class QuinticPolynomialTrajectory:
    """
    5차 다항식 궤적 (jtraj 스타일)
    
    q(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    
    경계 조건:
    - q(0) = qi, q(tf) = qf
    - q'(0) = 0, q'(tf) = 0  (초기/최종 속도 = 0)
    - q''(0) = 0, q''(tf) = 0  (초기/최종 가속도 = 0)
    """

    def __init__(self, qi: float, qf: float, tf: float):
        """
        Args:
            qi: Initial position [rad]
            qf: Final position [rad]
            tf: Final time [s]
        """
        self.qi = qi
        self.qf = qf
        self.tf = tf
        
        # 5차 다항식 계수 계산
        # 정규화된 시간 s = t/tf (0 ≤ s ≤ 1) 사용
        # q(s) = qi + (qf - qi) * (10s³ - 15s⁴ + 6s⁵)
        # 이는 s=0에서 q=qi, s=1에서 q=qf
        # 속도와 가속도가 양 끝에서 0
        self.h = qf - qi

    def get_state(self, t: float) -> Tuple[float, float, float]:
        """
        Get (position, velocity, acceleration) at time t.
        
        5차 다항식: q(s) = qi + h * (10s³ - 15s⁴ + 6s⁵)
        여기서 s = t/tf, h = qf - qi
        """
        t = np.clip(t, 0.0, self.tf)
        
        if self.tf < 1e-6:
            return float(self.qf), 0.0, 0.0
        
        s = t / self.tf  # 정규화된 시간
        s2 = s * s
        s3 = s2 * s
        s4 = s3 * s
        s5 = s4 * s
        
        # Position: q = qi + h * (10s³ - 15s⁴ + 6s⁵)
        q = self.qi + self.h * (10*s3 - 15*s4 + 6*s5)
        
        # Velocity: dq/dt = (dq/ds) * (ds/dt) = (dq/ds) / tf
        # dq/ds = h * (30s² - 60s³ + 30s⁴)
        dq_ds = self.h * (30*s2 - 60*s3 + 30*s4)
        v = dq_ds / self.tf
        
        # Acceleration: d²q/dt² = (d²q/ds²) / tf²
        # d²q/ds² = h * (60s - 180s² + 120s³)
        d2q_ds2 = self.h * (60*s - 180*s2 + 120*s3)
        a = d2q_ds2 / (self.tf * self.tf)
        
        return float(q), float(v), float(a)

    @property
    def duration(self) -> float:
        return self.tf


class OrientationTrajectoryScenario:
    """
    Orientation trajectory generation using Roll-Pitch-Yaw (Euler angles).
    
    MATLAB 스타일 구현:
    - Ri, Rf를 Rotation Matrix로 입력
    - tr2eul로 Euler angle 변환
    - jtraj (5차 다항식)로 궤적 생성
    - eul2r로 R(t) 계산
    """
    
    XYZ_USD = os.path.join(_EXT_ROOT, "asset", "XYZ.usd")
    XYZ_PRIM_PATH = "/World/XYZ"
    XYZ_POSITION = (1.0, 1.0, 1.0)

    def __init__(self):
        self._xyz_prim = None
        self._is_ready = False
        
        # Rotation matrices (Ri, Rf)
        self._Ri: np.ndarray = np.eye(3)  # 기본값: 단위행렬
        self._Rf: np.ndarray = self.rotx(-np.pi/4)  # 기본값: rotx(-π/4)
        
        # Euler angle trajectories (roll, pitch, yaw)
        self._roll_traj: Optional[QuinticPolynomialTrajectory] = None
        self._pitch_traj: Optional[QuinticPolynomialTrajectory] = None
        self._yaw_traj: Optional[QuinticPolynomialTrajectory] = None
        
        # Euler angles (extracted from Ri, Rf)
        self._euler_i: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._euler_f: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        
        # Execution state
        self._is_executing = False
        self._elapsed_time = 0.0
        self._tf = 2.0  # Default duration
        self._num_steps = 20  # Default number of steps (like MATLAB)

    def setup(self, articulation=None):
        """Initialize scenario."""
        self._is_ready = True

    def teardown(self):
        """Cleanup resources."""
        self._xyz_prim = None
        self._is_ready = False
        self._is_executing = False
        self._roll_traj = None
        self._pitch_traj = None
        self._yaw_traj = None

    @property
    def is_ready(self) -> bool:
        return self._is_ready

    @property
    def is_executing(self) -> bool:
        return self._is_executing

    @property
    def trajectory_duration(self) -> float:
        return self._tf if self._roll_traj else 0.0

    # ─────────────────────────────────────────────────────────────────────────
    # Rotation Matrix Utilities (MATLAB 스타일)
    # ─────────────────────────────────────────────────────────────────────────
    @staticmethod
    def rotx(angle: float) -> np.ndarray:
        """
        Rotation matrix around X-axis.
        MATLAB: rotx(angle)
        """
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ])

    @staticmethod
    def roty(angle: float) -> np.ndarray:
        """
        Rotation matrix around Y-axis.
        MATLAB: roty(angle)
        """
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, 0, s],
            [0, 1, 0],
            [-s, 0, c]
        ])

    @staticmethod
    def rotz(angle: float) -> np.ndarray:
        """
        Rotation matrix around Z-axis.
        MATLAB: rotz(angle)
        """
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])

    @staticmethod
    def tr2eul(R: np.ndarray) -> Tuple[float, float, float]:
        """
        Convert rotation matrix to ZYZ Euler angles.
        MATLAB: tr2eul(R) - ZYZ convention
        
        Returns (phi, theta, psi) in radians.
        R = Rz(phi) * Ry(theta) * Rz(psi)
        """
        # ZYZ Euler angles extraction
        # theta = atan2(sqrt(R[0,2]² + R[1,2]²), R[2,2])
        
        eps = 1e-10
        
        if abs(R[2, 2]) < 1 - eps:
            # Normal case
            theta = np.arctan2(np.sqrt(R[0, 2]**2 + R[1, 2]**2), R[2, 2])
            phi = np.arctan2(R[1, 2], R[0, 2])
            psi = np.arctan2(R[2, 1], -R[2, 0])
        else:
            # Gimbal lock (theta ≈ 0 or π)
            if R[2, 2] > 0:
                # theta ≈ 0
                theta = 0.0
                phi = 0.0
                psi = np.arctan2(-R[0, 1], R[0, 0])
            else:
                # theta ≈ π
                theta = np.pi
                phi = 0.0
                psi = np.arctan2(R[0, 1], -R[0, 0])
        
        return (phi, theta, psi)

    @staticmethod
    def eul2r(phi: float, theta: float, psi: float) -> np.ndarray:
        """
        Convert ZYZ Euler angles to rotation matrix.
        MATLAB: eul2r(phi, theta, psi) - ZYZ convention
        
        R = Rz(phi) * Ry(theta) * Rz(psi)
        """
        Rz_phi = OrientationTrajectoryScenario.rotz(phi)
        Ry_theta = OrientationTrajectoryScenario.roty(theta)
        Rz_psi = OrientationTrajectoryScenario.rotz(psi)
        
        return Rz_phi @ Ry_theta @ Rz_psi

    @staticmethod
    def rotation_matrix_to_euler_xyz(R: np.ndarray) -> Tuple[float, float, float]:
        """
        Convert rotation matrix to XYZ Euler angles (for USD).
        Returns (roll, pitch, yaw) in radians.
        """
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        
        if sy > 1e-6:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0.0
        
        return (roll, pitch, yaw)

    # ─────────────────────────────────────────────────────────────────────────
    # XYZ USD Spawning
    # ─────────────────────────────────────────────────────────────────────────
    def spawn_xyz(self) -> bool:
        """Spawn XYZ.usd at position (1, 1, 1) with initial orientation Ri."""
        stage = get_current_stage()
        if not stage:
            print("[OrientationTraj] ERROR: No stage available!")
            return False

        if not os.path.exists(self.XYZ_USD):
            print(f"[OrientationTraj] ERROR: XYZ.usd not found at {self.XYZ_USD}")
            return False

        # Remove existing prim if exists
        existing_prim = stage.GetPrimAtPath(self.XYZ_PRIM_PATH)
        if existing_prim.IsValid():
            stage.RemovePrim(self.XYZ_PRIM_PATH)

        # Add XYZ.usd to stage
        self._xyz_prim = add_reference_to_stage(self.XYZ_USD, self.XYZ_PRIM_PATH)

        if not self._xyz_prim:
            print("[OrientationTraj] ERROR: Failed to add XYZ.usd to stage!")
            return False

        # Set position and initial orientation (Ri)
        self._apply_rotation_matrix(self._Ri)
        
        euler_i = self.tr2eul(self._Ri)
        print(f"[OrientationTraj] XYZ.usd spawned at {self.XYZ_POSITION}")
        print(f"[OrientationTraj] Ri Euler (ZYZ): ({np.degrees(euler_i[0]):.1f}°, "
              f"{np.degrees(euler_i[1]):.1f}°, {np.degrees(euler_i[2]):.1f}°)")
        return True

    def _apply_rotation_matrix(self, R: np.ndarray):
        """Apply position and rotation matrix to XYZ prim."""
        if not self._xyz_prim:
            return
        
        stage = get_current_stage()
        if not stage:
            return
        
        prim = stage.GetPrimAtPath(self.XYZ_PRIM_PATH)
        if not prim.IsValid():
            return
        
        xformable = UsdGeom.Xformable(prim)
        xformable.ClearXformOpOrder()
        
        # Translation
        translate_op = xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*self.XYZ_POSITION))
        
        # Convert rotation matrix to XYZ Euler angles for USD
        roll, pitch, yaw = self.rotation_matrix_to_euler_xyz(R)
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)
        
        rotate_op = xformable.AddRotateXYZOp()
        rotate_op.Set(Gf.Vec3f(roll_deg, pitch_deg, yaw_deg))

    # ─────────────────────────────────────────────────────────────────────────
    # Rotation Matrix Input (MATLAB 스타일)
    # ─────────────────────────────────────────────────────────────────────────
    def set_Ri_eye(self):
        """Set Ri = eye(3) (identity matrix)."""
        self._Ri = np.eye(3)
        print("[OrientationTraj] Ri = eye(3)")

    def set_Ri_rotx(self, angle_deg: float):
        """Set Ri = rotx(angle)."""
        self._Ri = self.rotx(np.radians(angle_deg))
        print(f"[OrientationTraj] Ri = rotx({angle_deg:.1f}°)")

    def set_Rf_rotx(self, angle_deg: float):
        """Set Rf = rotx(angle)."""
        self._Rf = self.rotx(np.radians(angle_deg))
        print(f"[OrientationTraj] Rf = rotx({angle_deg:.1f}°)")

    def set_Ri(self, R: np.ndarray):
        """Set Ri directly."""
        self._Ri = R.copy()

    def set_Rf(self, R: np.ndarray):
        """Set Rf directly."""
        self._Rf = R.copy()

    def get_Ri(self) -> np.ndarray:
        """Get current Ri."""
        return self._Ri.copy()

    def get_Rf(self) -> np.ndarray:
        """Get current Rf."""
        return self._Rf.copy()

    def get_euler_i_deg(self) -> Tuple[float, float, float]:
        """Get Euler_i in degrees (ZYZ)."""
        return tuple(np.degrees(x) for x in self._euler_i)

    def get_euler_f_deg(self) -> Tuple[float, float, float]:
        """Get Euler_f in degrees (ZYZ)."""
        return tuple(np.degrees(x) for x in self._euler_f)

    # ─────────────────────────────────────────────────────────────────────────
    # Trajectory Generation (jtraj - 5차 다항식)
    # ─────────────────────────────────────────────────────────────────────────
    def generate_trajectory(self, tf: float = 2.0, num_steps: int = 20) -> dict:
        """
        Generate Euler angle trajectory using quintic polynomial (jtraj).
        
        MATLAB 스타일:
        Euler_i = tr2eul(Ri);
        Euler_f = tr2eul(Rf);
        Euler_traj = jtraj(Euler_i, Euler_f, num_steps);
        
        Args:
            tf: Final time [s]
            num_steps: Number of trajectory steps (like MATLAB's jtraj)
        
        Returns:
            {"success": bool, "message": str}
        """
        if tf <= 0:
            return {"success": False, "message": "tf must be > 0"}

        self._tf = tf
        self._num_steps = num_steps
        
        # STEP 1: Ri와 Rf를 Euler angle로 변환 (tr2eul)
        print(f"\n[OrientationTraj] ===== Trajectory Generation (jtraj) =====")
        print(f"[OrientationTraj] STEP 1: tr2eul - Convert Ri, Rf to Euler angles (ZYZ)")
        
        self._euler_i = self.tr2eul(self._Ri)
        self._euler_f = self.tr2eul(self._Rf)
        
        phi_i, theta_i, psi_i = self._euler_i
        phi_f, theta_f, psi_f = self._euler_f
        
        print(f"  Ri:")
        print(f"    {self._Ri[0]}")
        print(f"    {self._Ri[1]}")
        print(f"    {self._Ri[2]}")
        print(f"  Euler_i = tr2eul(Ri) = ({np.degrees(phi_i):.1f}°, {np.degrees(theta_i):.1f}°, {np.degrees(psi_i):.1f}°)")
        
        print(f"  Rf:")
        print(f"    {self._Rf[0]}")
        print(f"    {self._Rf[1]}")
        print(f"    {self._Rf[2]}")
        print(f"  Euler_f = tr2eul(Rf) = ({np.degrees(phi_f):.1f}°, {np.degrees(theta_f):.1f}°, {np.degrees(psi_f):.1f}°)")
        
        # STEP 2: Euler angle의 궤적 생성 (jtraj - 5차 다항식)
        print(f"[OrientationTraj] STEP 2: jtraj - Generate quintic polynomial trajectory")
        print(f"  Euler_traj = jtraj(Euler_i, Euler_f, {num_steps})")
        
        # Create quintic polynomial trajectories for each Euler angle
        self._roll_traj = QuinticPolynomialTrajectory(phi_i, phi_f, tf)  # phi
        self._pitch_traj = QuinticPolynomialTrajectory(theta_i, theta_f, tf)  # theta
        self._yaw_traj = QuinticPolynomialTrajectory(psi_i, psi_f, tf)  # psi
        
        print(f"  φ (phi):   {np.degrees(phi_i):.1f}° → {np.degrees(phi_f):.1f}°")
        print(f"  θ (theta): {np.degrees(theta_i):.1f}° → {np.degrees(theta_f):.1f}°")
        print(f"  ψ (psi):   {np.degrees(psi_i):.1f}° → {np.degrees(psi_f):.1f}°")
        
        self._elapsed_time = 0.0
        
        print(f"[OrientationTraj] Trajectory generated: tf={tf:.2f}s, steps={num_steps}")
        print(f"[OrientationTraj] STEP 3: eul2r will convert Euler(t) → R(t) during execution")
        
        return {"success": True, "message": "OK"}

    # ─────────────────────────────────────────────────────────────────────────
    # Trajectory Execution
    # ─────────────────────────────────────────────────────────────────────────
    def start_execution(self):
        """Start trajectory execution."""
        if not self._roll_traj:
            print("[OrientationTraj] Generate trajectory first!")
            return
        
        # Apply initial orientation (Ri)
        self._apply_rotation_matrix(self._Ri)
        
        self._is_executing = True
        self._elapsed_time = 0.0
        print("[OrientationTraj] Execution started")

    def stop_execution(self):
        """Stop trajectory execution."""
        self._is_executing = False
        print("[OrientationTraj] Execution stopped")

    def update(self, dt: float):
        """
        Update trajectory (call every physics step).
        
        STEP 3: eul2r - Euler angle의 궤적을 Rotation matrix의 궤적으로 변환
        R = eul2r(Euler_traj(:,1), Euler_traj(:,2), Euler_traj(:,3))
        """
        if not self._is_executing or not self._roll_traj:
            return
        
        self._elapsed_time += dt
        
        if self._elapsed_time >= self._tf:
            self._elapsed_time = self._tf
            self._is_executing = False
        
        # Get current Euler angles from trajectories
        phi, _, _ = self._roll_traj.get_state(self._elapsed_time)
        theta, _, _ = self._pitch_traj.get_state(self._elapsed_time)
        psi, _, _ = self._yaw_traj.get_state(self._elapsed_time)
        
        # STEP 3: eul2r - Convert Euler angles to rotation matrix
        R = self.eul2r(phi, theta, psi)
        
        # Apply to XYZ prim
        self._apply_rotation_matrix(R)

    def get_state(self) -> Optional[dict]:
        """Get current trajectory state."""
        if not self._roll_traj:
            return None
        
        phi, phi_v, phi_a = self._roll_traj.get_state(self._elapsed_time)
        theta, theta_v, theta_a = self._pitch_traj.get_state(self._elapsed_time)
        psi, psi_v, psi_a = self._yaw_traj.get_state(self._elapsed_time)
        
        # STEP 3: eul2r - Convert to rotation matrix
        R = self.eul2r(phi, theta, psi)
        
        return {
            "time": self._elapsed_time,
            "duration": self._tf,
            "phi": phi,      # ZYZ Euler angle 1
            "theta": theta,  # ZYZ Euler angle 2
            "psi": psi,      # ZYZ Euler angle 3
            "phi_velocity": phi_v,
            "theta_velocity": theta_v,
            "psi_velocity": psi_v,
            "rotation_matrix": R,
        }

    def get_xyz_position(self) -> Optional[tuple]:
        """Get current XYZ prim position."""
        if not self._xyz_prim:
            return None

        stage = get_current_stage()
        if not stage:
            return None

        prim = stage.GetPrimAtPath(self.XYZ_PRIM_PATH)
        if not prim.IsValid():
            return None

        xformable = UsdGeom.Xformable(prim)
        world_transform = xformable.ComputeLocalToWorldTransform(0)
        translation = world_transform.ExtractTranslation()

        return (translation[0], translation[1], translation[2])
