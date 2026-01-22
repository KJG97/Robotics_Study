# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Robotics Study Assignments Module

This module contains all assignment implementations for robotics study.
Each assignment is organized in its own submodule with:
- scenario.py: Core logic and robot control
- ui.py: UI components and callbacks
"""

from .assignment1 import Assignment1Scenario, Assignment1UI

__all__ = [
    "Assignment1Scenario",
    "Assignment1UI",
]

