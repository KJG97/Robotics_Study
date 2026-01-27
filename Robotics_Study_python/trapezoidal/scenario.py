# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Trapezoidal Velocity Profile Trajectory Generation
"""

import numpy as np
from typing import Tuple, Optional


class TrapezoidalProfile:
    """
    Trapezoidal velocity profile for single joint motion.
    
    사다리꼴 속도 함수를 이용한 궤적계획:
    
    q(t) = | qi + (1/2)*a*t²                    0 ≤ t ≤ tc
           | qi + a*tc*(t - tc/2)               tc < t ≤ tf - tc
           | qf - (1/2)*a*(tf - t)²             tf - tc < t ≤ tf
    
    단, tc = tf/2 - (1/2)*sqrt((tf²*a - 4*(qf-qi))/a)
        |a| ≥ 4*|qf-qi|/tf² 를 만족해야 함
    """

    def __init__(self, qi: float, qf: float, tf: float, tc: float, a: float):
        """
        Args:
            qi: Initial position [rad]
            qf: Final position [rad]  
            tf: Final time [s]
            tc: Acceleration/deceleration time [s]
            a: Acceleration (signed) [rad/s²]
        """
        self.qi = qi
        self.qf = qf
        self.tf = tf
        self.tc = tc
        self.a = a  # Signed acceleration
        
        # v_max = a * tc
        self.v_max = abs(a * tc)
        self.is_triangular = (tc >= tf / 2 - 1e-6)  # tc == tf/2 이면 삼각형

    def get_state(self, t: float) -> Tuple[float, float, float]:
        """
        Get (position, velocity, acceleration) at time t.
        
        수식:
        q(t) = | qi + (1/2)*a*t²                    0 ≤ t ≤ tc
               | qi + a*tc*(t - tc/2)               tc < t ≤ tf - tc  
               | qf - (1/2)*a*(tf - t)²             tf - tc < t ≤ tf
        """
        t = np.clip(t, 0.0, self.tf)
        
        qi, qf, tf, tc, a = self.qi, self.qf, self.tf, self.tc, self.a
        
        if t <= tc:
            # Phase 1: Acceleration (0 ≤ t ≤ tc)
            q = qi + 0.5 * a * t**2
            v = a * t
            acc = a
        elif t <= tf - tc:
            # Phase 2: Cruise (tc < t ≤ tf - tc)
            q = qi + a * tc * (t - tc / 2)
            v = a * tc
            acc = 0.0
        else:
            # Phase 3: Deceleration (tf - tc < t ≤ tf)
            q = qf - 0.5 * a * (tf - t)**2
            v = a * (tf - t)
            acc = -a
        
        return float(q), float(v), float(acc)

    @property
    def duration(self) -> float:
        return self.tf

class TrajectoryGenerationScenario:
    """Single joint trajectory generation using trapezoidal profile."""
    
    TARGET_JOINT = "SP"

    def __init__(self):
        self.articulation = None
        self._joint_index: Optional[int] = None
        self._joint_limits: Tuple[float, float] = (-np.pi, np.pi)
        self._trajectory: Optional[TrapezoidalProfile] = None
        self._is_executing = False
        self._elapsed_time = 0.0

    def setup(self, articulation):
        """Initialize with robot articulation."""
        self.articulation = articulation
        if not articulation:
            return
        
        dof_names = list(articulation.dof_names)
        if self.TARGET_JOINT in dof_names:
            self._joint_index = dof_names.index(self.TARGET_JOINT)
            lower = articulation.dof_properties["lower"][self._joint_index]
            upper = articulation.dof_properties["upper"][self._joint_index]
            self._joint_limits = (lower, upper)
            print(f"[TrajGen] Joint: {self.TARGET_JOINT}, Limits: [{np.degrees(lower):.0f}, {np.degrees(upper):.0f}]°")
        else:
            print(f"[TrajGen] ERROR: {self.TARGET_JOINT} not found!")
            self._joint_index = None

    def teardown(self):
        """Cleanup resources."""
        self.articulation = None
        self._joint_index = None
        self._trajectory = None
        self._is_executing = False

    @property
    def is_ready(self) -> bool:
        return self.articulation is not None and self._joint_index is not None

    @property
    def is_executing(self) -> bool:
        return self._is_executing

    @property
    def joint_limits(self) -> Tuple[float, float]:
        return self._joint_limits

    @property
    def trajectory_duration(self) -> float:
        return self._trajectory.duration if self._trajectory else 0.0

    def get_current_position(self) -> Optional[float]:
        """Get current joint position [rad]."""
        if not self.is_ready:
            return None
        positions = self.articulation.get_joint_positions()
        return float(positions[self._joint_index]) if positions is not None else None

    def generate(self, qi: float, qf: float, tf: float, a_max: float) -> dict:
        """
        Generate trapezoidal trajectory.
        
        사다리꼴 속도 함수를 이용한 궤적계획:
        
        tc = tf/2 - (1/2)*sqrt((tf²*a - 4*(qf-qi))/a)
        
        조건: |a| ≥ 4*|qf-qi|/tf²
        
        Args:
            qi: Initial position [rad]
            qf: Final position [rad]
            tf: Final time [s]
            a_max: Max acceleration magnitude [rad/s²]
        
        Returns:
            {"success": bool, "message": str}
        """
        if not self.is_ready:
            return {"success": False, "message": "Robot not ready"}
        
        # Clamp to limits
        qi = np.clip(qi, *self._joint_limits)
        qf = np.clip(qf, *self._joint_limits)
        
        # Validation
        h = qf - qi  # Signed displacement
        if abs(h) < 1e-6:
            return {"success": False, "message": "qi == qf"}
        if tf <= 0:
            return {"success": False, "message": "tf <= 0"}
        if a_max <= 0:
            return {"success": False, "message": "a_max <= 0"}
        
        # 조건: |a| ≥ 4*|qf-qi|/tf²
        a_min_required = 4 * abs(h) / (tf**2)
        if a_max < a_min_required:
            return {"success": False, "message": f"|a| must be ≥ {np.degrees(a_min_required):.1f}°/s²"}
        
        # Signed acceleration (방향에 따라 부호 결정)
        a = a_max if h > 0 else -a_max
        
        # tc 계산: tc = tf/2 - (1/2)*sqrt((tf²*a - 4*h)/a)
        # h와 a가 같은 부호이므로: tf²*|a| - 4*|h| ≥ 0
        discriminant = (tf**2 * abs(a) - 4 * abs(h)) / abs(a)
        tc = tf / 2 - np.sqrt(discriminant) / 2
        
        # tc는 0 < tc ≤ tf/2 범위여야 함
        tc = np.clip(tc, 1e-6, tf / 2)
        
        # v_max = |a| * tc
        v_max = abs(a) * tc
        
        # Create trajectory
        self._trajectory = TrapezoidalProfile(qi, qf, tf, tc, a)
        self._elapsed_time = 0.0
        
        profile = "Triangular" if self._trajectory.is_triangular else "Trapezoidal"
        print(f"[TrajGen] {profile}: qi={np.degrees(qi):.1f}° → qf={np.degrees(qf):.1f}°")
        print(f"         tc={tc:.3f}s, tf={tf:.2f}s, v_max={np.degrees(v_max):.1f}°/s, a={np.degrees(a):.1f}°/s²")
        
        return {"success": True, "message": "OK"}

    def start_execution(self):
        """Start trajectory execution from qi."""
        if not self._trajectory:
            return
        
        # Set initial position
        if self.articulation:
            positions = self.articulation.get_joint_positions()
            if positions is not None:
                positions[self._joint_index] = self._trajectory.qi
                self.articulation.set_joint_positions(positions)
        
        self._is_executing = True
        self._elapsed_time = 0.0

    def stop_execution(self):
        """Stop trajectory execution."""
        self._is_executing = False

    def update(self, dt: float):
        """Update trajectory (call every physics step)."""
        if not self._is_executing or not self._trajectory:
            return
        
        self._elapsed_time += dt
        
        if self._elapsed_time >= self._trajectory.duration:
            self._elapsed_time = self._trajectory.duration
            self._is_executing = False
        
        q, _, _ = self._trajectory.get_state(self._elapsed_time)
        
        if self.articulation:
            positions = self.articulation.get_joint_positions()
            if positions is not None:
                positions[self._joint_index] = q
                self.articulation.set_joint_positions(positions)

    def get_state(self) -> Optional[dict]:
        """Get current trajectory state."""
        if not self._trajectory:
            return None
        
        q, v, a = self._trajectory.get_state(self._elapsed_time)
        return {
            "time": self._elapsed_time,
            "duration": self._trajectory.duration,
            "position": q,
            "velocity": v,
            "acceleration": a,
        }
