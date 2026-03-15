# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Assignment1: ALLEX Robot Forward Kinematics

This module implements forward kinematics study for the ALLEX robot.
Features:
- Predefined pose application (pose1, pose2, pose3)
- FK result display
- Pose animation with trajectory visualization
"""

from .scenario import Assignment1Scenario
from .ui import Assignment1UI

__all__ = [
    "Assignment1Scenario",
    "Assignment1UI",
]

