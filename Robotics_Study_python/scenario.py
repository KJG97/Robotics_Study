# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Robotics Study Scenario - Chapter2 Examples
7DOF Manipulator, 2DOF Prismatic robot trajectory control.
"""

import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdGeom


class RoboticsStudyScenario:
    """Main scenario class for robotics study examples."""

    def __init__(self):
        self._articulation = None
        self._time = 0.0
        
        # 7DOF trajectory state
        self._trajectory_active = False
        self._trajectory_time = 0.0
        
        # 2DOF trajectory state
        self._2dof_trajectory_active = False
        self._2dof_trajectory_time = 0.0
        self._2dof_articulation = None

    def setup_scenario(self, articulation):
        """Initialize scenario with robot articulation."""
        self._articulation = articulation
        self._time = 0.0
        
        if self._articulation:
            print("=" * 50)
            print("[OK] Robotics Study Scenario Setup Complete!")
            print(f"   DOF Count: {self._articulation.num_dof}")
            print(f"   Body Count: {self._articulation.num_bodies}")
            print("=" * 50)

    def teardown_scenario(self):
        """Clean up scenario resources."""
        self._articulation = None
        self._time = 0.0
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
        
        # Clear debug draw
        try:
            from isaacsim.util.debug_draw import _debug_draw
            _debug_draw.acquire_debug_draw_interface().clear_points()
        except Exception:
            pass
        
        print("[Example3] 7DOF trajectory started! Period: 2.0s")
    
    def stop_7dof_trajectory(self):
        """Stop 7DOF trajectory and return to zero pose."""
        self._trajectory_active = False
        
        if self._articulation:
            self._articulation.apply_action(ArticulationAction(joint_positions=np.zeros(7)))
        
        # Clear debug draw
        try:
            from isaacsim.util.debug_draw import _debug_draw
            _debug_draw.acquire_debug_draw_interface().clear_points()
        except Exception:
            pass
        
        print("[Example3] 7DOF trajectory stopped.")
    
    def is_trajectory_active(self) -> bool:
        return self._trajectory_active
    
    def _update_7dof_trajectory(self, step: float):
        """Update 7DOF trajectory (sinusoidal joint motion)."""
        if self._articulation is None:
            return
        
        self._trajectory_time += step
        t = self._trajectory_time
        omega = 2 * np.pi / 2.0  # Period = 2.0s
        
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
        
        # Draw end-effector point
        try:
            from isaacsim.core.utils.stage import get_current_stage
            from isaacsim.util.debug_draw import _debug_draw
            
            stage = get_current_stage()
            if stage is None:
                return
            
            ee_prim = stage.GetPrimAtPath("/World/ALLEX_Right_Arm/tcp")
            if not ee_prim.IsValid():
                return
            
            xform = UsdGeom.Xformable(ee_prim)
            transform = xform.ComputeLocalToWorldTransform(0)
            pos = tuple(transform.GetRow(3)[:3])
            
            draw = _debug_draw.acquire_debug_draw_interface()
            draw.draw_points([pos], [(1.0, 0.0, 0.0, 1.0)], [5])
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
            
            prim = stage.GetPrimAtPath("/prismatic_2dof")
            if not prim.IsValid():
                print("[Example4] Error: Prim not found at /prismatic_2dof")
                return
            
            self._2dof_articulation = SingleArticulation("/prismatic_2dof")
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
                self._2dof_articulation.apply_action(ArticulationAction(joint_positions=np.array([0.0, 0.5])))
            except Exception:
                pass
        
        self._2dof_articulation = None
        print("[Example4] 2DOF trajectory stopped.")
    
    def is_2dof_trajectory_active(self) -> bool:
        return self._2dof_trajectory_active
    
    def _update_2dof_trajectory(self, step: float):
        """Update 2DOF trajectory (sinusoidal joint motion)."""
        if self._2dof_articulation is None:
            return
        
        self._2dof_trajectory_time += step
        t = self._2dof_trajectory_time
        omega = 2 * np.pi / 3.0  # Period = 3.0s
        
        q = np.array([
            np.pi/4 * np.sin(omega * t),   # revolute
            -0.5 * np.sin(omega * t),      # prismatic
        ])
        
        try:
            self._2dof_articulation.apply_action(ArticulationAction(joint_positions=q))
        except Exception:
            pass
