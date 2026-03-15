# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Robotics Study Assignments Module

This module contains all assignment implementations for robotics study.
Each assignment is organized in its own submodule with:
- scenario.py: Core logic and robot control
- ui.py: UI components and callbacks
"""

import importlib.util
import os
import sys

# Load "assignment 1" package (folder name contains space, not a valid Python identifier)
_ASSIGNMENT1_DIR = os.path.join(os.path.dirname(__file__), "assignment 1")
_spec = importlib.util.spec_from_file_location(
    "assignment1_module",
    os.path.join(_ASSIGNMENT1_DIR, "__init__.py"),
    submodule_search_locations=[_ASSIGNMENT1_DIR],
)
_assignment1 = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _assignment1
_spec.loader.exec_module(_assignment1)

Assignment1Scenario = _assignment1.Assignment1Scenario
Assignment1UI = _assignment1.Assignment1UI

# Load "assignment 2" package (folder name contains space)
_ASSIGNMENT2_DIR = os.path.join(os.path.dirname(__file__), "assignment 2")
_spec2 = importlib.util.spec_from_file_location(
    "assignment2_module",
    os.path.join(_ASSIGNMENT2_DIR, "__init__.py"),
    submodule_search_locations=[_ASSIGNMENT2_DIR],
)
_assignment2 = importlib.util.module_from_spec(_spec2)
sys.modules[_spec2.name] = _assignment2
_spec2.loader.exec_module(_assignment2)

Assignment2Scenario = _assignment2.Assignment2Scenario
Assignment2UI = _assignment2.Assignment2UI

__all__ = [
    "Assignment1Scenario",
    "Assignment1UI",
    "Assignment2Scenario",
    "Assignment2UI",
]

