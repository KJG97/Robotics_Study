# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Trajectory Generation module for Robotics Study Extension
Trapezoidal velocity profile based trajectory planning
"""

from .ui import TrajectoryGenerationUI
from .scenario import TrajectoryGenerationScenario

__all__ = ["TrajectoryGenerationUI", "TrajectoryGenerationScenario"]
