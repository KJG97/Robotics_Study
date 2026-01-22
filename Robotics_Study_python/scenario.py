# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Robotics Study Scenario

Chapter2 Examples: 7DOF Manipulator, 2DOF Prismatic robot trajectory control.
Assignment-specific scenarios are located in the assignments/ module.
"""

from typing import Optional
import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdGeom


class RoboticsStudyScenario:
    """Main scenario class for robotics study examples."""
    
    # Configuration constants
    _7DOF_PERIOD = 2.0  # seconds
    _2DOF_PERIOD = 3.0  # seconds
    _EE_PRIM_PATH = "/ALLEX_Right_Arm/tcp"
    _2DOF_PRIM_PATH = "/prismatic_2dof"
    _POINT_SIZE = 5
    _RED_COLOR = (1.0, 0.0, 0.0, 1.0)

    def __init__(self):
        self._articulation = None
        self._time = 0.0
        self._draw_enabled = True
        
        # 7DOF trajectory state
        self._trajectory_active = False
        self._trajectory_time = 0.0
        
        # 2DOF trajectory state
        self._2dof_trajectory_active = False
        self._2dof_trajectory_time = 0.0
        self._2dof_articulation = None

    # ========================================
    # Debug Draw Utilities
    # ========================================
    
    def _get_debug_draw(self):
        """Acquire debug draw interface (fresh each time to avoid stale reference)."""
        try:
            from isaacsim.util.debug_draw import _debug_draw
            return _debug_draw.acquire_debug_draw_interface()
        except Exception:
            return None
    
    def _clear_debug_draw(self):
        """Safely clear debug draw points."""
        if not self._draw_enabled:
            return
        try:
            from isaacsim.core.utils.stage import get_current_stage
            if get_current_stage() is None:
                return
            draw = self._get_debug_draw()
            if draw:
                draw.clear_points()
        except Exception:
            pass

    # ========================================
    # Scenario Lifecycle
    # ========================================

    def setup_scenario(self, articulation):
        """Initialize scenario with robot articulation."""
        self._articulation = articulation
        self._time = 0.0
        self._draw_enabled = True
        
        if self._articulation:
            print("=" * 50)
            print("[OK] Robotics Study Scenario Setup Complete!")
            print(f"   DOF Count: {self._articulation.num_dof}")
            print(f"   Body Count: {self._articulation.num_bodies}")
            print("=" * 50)

    def teardown_scenario(self):
        """Clean up scenario resources."""
        self._draw_enabled = False
        self._articulation = None
        self._time = 0.0
        
        # Reset trajectory states
        self._trajectory_active = False
        self._trajectory_time = 0.0
        self._2dof_trajectory_active = False
        self._2dof_trajectory_time = 0.0
        self._2dof_articulation = None

    def update_scenario(self, step: float):
        """Update scenario each physics step."""
        self._time += step
        
        if self._trajectory_active:
            self._update_7dof_trajectory(step)
        
        if self._2dof_trajectory_active:
            self._update_2dof_trajectory(step)

    # ========================================
    # Robot Info Query
    # ========================================
    
    def get_all_joint_states(self) -> dict:
        """Get all joint states (position, velocity, torque)."""
        if self._articulation is None:
            return {"error": "Articulation not initialized."}
        
        try:
            return {
                "dof_names": self._articulation.dof_names,
                "positions": self._articulation.get_joint_positions().tolist(),
                "velocities": self._articulation.get_joint_velocities().tolist(),
                "torques": self._articulation.get_measured_joint_efforts().tolist(),
            }
        except Exception as e:
            return {"error": f"Failed to get joint states: {e}"}

    # ========================================
    # 7DOF Manipulator Trajectory (Example3)
    # ========================================
    
    def start_7dof_trajectory(self):
        """Start 7DOF sinusoidal trajectory."""
        if self._articulation is None:
            print("[ERROR] Articulation not initialized. Load robot first.")
            return
        
        self._trajectory_active = True
        self._trajectory_time = 0.0
        self._draw_enabled = True
        self._clear_debug_draw()
        
        print(f"[Example3] 7DOF trajectory started! Period: {self._7DOF_PERIOD}s")
    
    def stop_7dof_trajectory(self):
        """Stop 7DOF trajectory and return to zero pose."""
        self._trajectory_active = False
        
        if self._articulation:
            action = ArticulationAction(joint_positions=np.zeros(7))
            self._articulation.apply_action(action)
        
        self._clear_debug_draw()
        print("[Example3] 7DOF trajectory stopped. Debug draw cleared.")
    
    def is_trajectory_active(self) -> bool:
        """Check if 7DOF trajectory is active."""
        return self._trajectory_active
    
    def _update_7dof_trajectory(self, step: float):
        """Update 7DOF trajectory (sinusoidal joint motion)."""
        if self._articulation is None:
            return
        
        self._trajectory_time += step
        t = self._trajectory_time
        omega = 2 * np.pi / self._7DOF_PERIOD
        
        # Sinusoidal trajectory for 7 joints
        q = np.array([
            np.pi/4 * np.sin(omega * t),   # Joint 0
            0,                              # Joint 1
            -np.pi/4 * np.sin(omega * t),  # Joint 2
            np.pi/2 * np.sin(omega * t),   # Joint 3
            0,                              # Joint 4
            -np.pi * np.sin(omega * t),    # Joint 5
            np.pi/4 * np.sin(omega * t),   # Joint 6
        ])
        
        self._articulation.apply_action(ArticulationAction(joint_positions=q))
        self._draw_ee_point()
    
    def _draw_ee_point(self):
        """Draw end-effector position as a point."""
        if not self._draw_enabled:
            return
            
        try:
            from isaacsim.core.utils.stage import get_current_stage
            stage = get_current_stage()
            if stage is None:
                return
            
            ee_prim = stage.GetPrimAtPath(self._EE_PRIM_PATH)
            if not ee_prim.IsValid():
                return
            
            # Get world position
            xform = UsdGeom.Xformable(ee_prim)
            transform = xform.ComputeLocalToWorldTransform(0)
            pos = tuple(transform.GetRow(3)[:3])
            
            draw = self._get_debug_draw()
            if draw:
                draw.draw_points([pos], [self._RED_COLOR], [self._POINT_SIZE])
        except Exception:
            pass

    # ========================================
    # 2DOF Prismatic Trajectory (Example4)
    # ========================================
    
    def start_2dof_trajectory(self):
        """Start 2DOF Prismatic robot trajectory."""
        try:
            from isaacsim.core.prims import SingleArticulation
            from isaacsim.core.utils.stage import get_current_stage
            
            stage = get_current_stage()
            if stage is None:
                print("[Example4] Error: No stage available.")
                return
            
            prim = stage.GetPrimAtPath(self._2DOF_PRIM_PATH)
            if not prim.IsValid():
                print(f"[Example4] Error: Prim not found at {self._2DOF_PRIM_PATH}")
                return
            
            self._2dof_articulation = SingleArticulation(self._2DOF_PRIM_PATH)
            self._2dof_articulation.initialize()
            self._2dof_trajectory_active = True
            self._2dof_trajectory_time = 0.0
            
            print(f"[Example4] 2DOF trajectory started! DOF: {self._2dof_articulation.num_dof}")
        except Exception as e:
            print(f"[Example4] Failed to start trajectory: {e}")
    
    def stop_2dof_trajectory(self):
        """Stop 2DOF trajectory and reset to initial pose."""
        self._2dof_trajectory_active = False
        
        if self._2dof_articulation:
            try:
                q_init = np.array([0.0, 0.5])
                self._2dof_articulation.apply_action(ArticulationAction(joint_positions=q_init))
            except Exception:
                pass
        
        self._2dof_articulation = None
        print("[Example4] 2DOF trajectory stopped.")
    
    def is_2dof_trajectory_active(self) -> bool:
        """Check if 2DOF trajectory is active."""
        return self._2dof_trajectory_active
    
    def _update_2dof_trajectory(self, step: float):
        """Update 2DOF trajectory (sinusoidal joint motion)."""
        if self._2dof_articulation is None:
            return
        
        self._2dof_trajectory_time += step
        t = self._2dof_trajectory_time
        omega = 2 * np.pi / self._2DOF_PERIOD
        
        # q(1) = pi/4 * sin(omega*t)       - revolute
        # q(2) = -0.5 * sin(omega*t)       - prismatic
        q = np.array([
            np.pi/4 * np.sin(omega * t),
            -0.5 * np.sin(omega * t),
        ])
        
        try:
            self._2dof_articulation.apply_action(ArticulationAction(joint_positions=q))
        except Exception:
            pass
