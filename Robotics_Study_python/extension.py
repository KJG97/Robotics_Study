# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import asyncio
import gc

import omni
import omni.kit.commands
import omni.physx as _physx
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import ScrollingWindow
from isaacsim.gui.components.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.usd import StageEventType

from .global_variables import EXTENSION_DESCRIPTION, EXTENSION_TITLE, ASSIGNMENT1_TITLE, ASSIGNMENT2_TITLE, ASSIGNMENT3_TITLE, ASSIGNMENT4_TITLE, ASSIGNMENT5_TITLE
from .ui_builder import UIBuilder

"""
This file serves as a basic template for the standard boilerplate operations
that make a UI-based extension appear on the toolbar.

This implementation is meant to cover most use-cases without modification.
Various callbacks are hooked up to a seperate class UIBuilder in .ui_builder.py
Most users will be able to make their desired UI extension by interacting solely with
UIBuilder.

This class sets up standard useful callback functions in UIBuilder:
    on_menu_callback: Called when extension is opened
    on_timeline_event: Called when timeline is stopped, paused, or played
    on_physics_step: Called on every physics step
    on_stage_event: Called when stage is opened or closed
    cleanup: Called when resources such as physics subscriptions should be cleaned up
    build_ui: User function that creates the UI they want.
"""


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""

        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()

        # Build Main Window
        self._window = ScrollingWindow(
            title=EXTENSION_TITLE, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        # Build Assignment Windows
        self._assignment_windows = {}
        self._assignment1_ui_built = False  # Flag to track if UI has been built
        
        # Assignment1 window with LOAD button
        assignment1_window = ScrollingWindow(
            title=ASSIGNMENT1_TITLE, width=400, height=300, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._assignment_windows[ASSIGNMENT1_TITLE] = assignment1_window
        
        # Other assignment windows (empty)
        empty_assignment_titles = [
            ASSIGNMENT2_TITLE, ASSIGNMENT3_TITLE, ASSIGNMENT4_TITLE, ASSIGNMENT5_TITLE
        ]
        for title in empty_assignment_titles:
            window = ScrollingWindow(
                title=title, width=400, height=300, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
            )
            # Build empty UI for assignment window
            with window.frame:
                with ui.VStack(spacing=5, height=0):
                    ui.Label(f"{title} - Empty Window", alignment=ui.Alignment.CENTER, height=50)
            self._assignment_windows[title] = window

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Add {EXTENSION_TITLE} Extension to UI toolbar",
        )
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{ASSIGNMENT1_TITLE}",
            self._assignment1_callback,
            description=f"Add {ASSIGNMENT1_TITLE} Extension to UI toolbar",
        )
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{ASSIGNMENT2_TITLE}",
            self._assignment2_callback,
            description=f"Add {ASSIGNMENT2_TITLE} Extension to UI toolbar",
        )
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{ASSIGNMENT3_TITLE}",
            self._assignment3_callback,
            description=f"Add {ASSIGNMENT3_TITLE} Extension to UI toolbar",
        )
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{ASSIGNMENT4_TITLE}",
            self._assignment4_callback,
            description=f"Add {ASSIGNMENT4_TITLE} Extension to UI toolbar",
        )
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{ASSIGNMENT5_TITLE}",
            self._assignment5_callback,
            description=f"Add {ASSIGNMENT5_TITLE} Extension to UI toolbar",
        )
        self._menu_items = [
            MenuItemDescription(name=EXTENSION_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")),
            MenuItemDescription(name=ASSIGNMENT1_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{ASSIGNMENT1_TITLE}")),
            MenuItemDescription(name=ASSIGNMENT2_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{ASSIGNMENT2_TITLE}")),
            MenuItemDescription(name=ASSIGNMENT3_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{ASSIGNMENT3_TITLE}")),
            MenuItemDescription(name=ASSIGNMENT4_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{ASSIGNMENT4_TITLE}")),
            MenuItemDescription(name=ASSIGNMENT5_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{ASSIGNMENT5_TITLE}")),
        ]

        add_menu_items(self._menu_items, EXTENSION_TITLE)

        # Filled in with User Functions
        self.ui_builder = UIBuilder()

        # Events
        self._usd_context = omni.usd.get_context()
        self._physxIFace = _physx.get_physx_interface()
        self._physx_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

    def on_shutdown(self):
        self._models = {}
        remove_menu_items(self._menu_items, EXTENSION_TITLE)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{ASSIGNMENT1_TITLE}")
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{ASSIGNMENT2_TITLE}")
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{ASSIGNMENT3_TITLE}")
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{ASSIGNMENT4_TITLE}")
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{ASSIGNMENT5_TITLE}")

        if self._window:
            self._window = None
        
        # Cleanup assignment windows
        if hasattr(self, '_assignment_windows'):
            for title, window in self._assignment_windows.items():
                if window:
                    window.visible = False
            self._assignment_windows = {}
        self._assignment1_ui_built = False
        
        self.ui_builder.cleanup()
        gc.collect()

    def _on_window(self, visible):
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)

            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub = None
            self._timeline_event_sub = None
            self.ui_builder.cleanup()

    def _build_ui(self):
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()

        async def dock_window():
            await omni.kit.app.get_app().next_update_async()

            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    #################################################################
    # Functions below this point call user functions
    #################################################################

    def _menu_callback(self):
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _assignment1_callback(self):
        """Toggle Assignment1 window visibility"""
        window = self._assignment_windows.get(ASSIGNMENT1_TITLE)
        if window:
            # Build UI only once (first time opening)
            if not self._assignment1_ui_built:
                self.ui_builder.build_assignment1_ui(window)
                self._assignment1_ui_built = True
            window.visible = not window.visible
            
            # Subscribe to timeline/stage events when window becomes visible
            if window.visible:
                self._setup_event_subscriptions()

    def _assignment2_callback(self):
        """Toggle Assignment2 window visibility"""
        window = self._assignment_windows.get(ASSIGNMENT2_TITLE)
        if window:
            window.visible = not window.visible

    def _assignment3_callback(self):
        """Toggle Assignment3 window visibility"""
        window = self._assignment_windows.get(ASSIGNMENT3_TITLE)
        if window:
            window.visible = not window.visible

    def _assignment4_callback(self):
        """Toggle Assignment4 window visibility"""
        window = self._assignment_windows.get(ASSIGNMENT4_TITLE)
        if window:
            window.visible = not window.visible

    def _assignment5_callback(self):
        """Toggle Assignment5 window visibility"""
        window = self._assignment_windows.get(ASSIGNMENT5_TITLE)
        if window:
            window.visible = not window.visible

    def _setup_event_subscriptions(self):
        """Setup timeline and stage event subscriptions"""
        # Only setup if not already subscribed
        if self._stage_event_sub is None:
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
        
        if not hasattr(self, '_timeline_event_sub') or self._timeline_event_sub is None:
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)

    def _on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if not self._physx_subscription:
                self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._on_physics_step)
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physx_subscription = None

        self.ui_builder.on_timeline_event(event)

    def _on_physics_step(self, step):
        self.ui_builder.on_physics_step(step)

    def _on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED) or event.type == int(StageEventType.CLOSED):
            # stage was opened or closed, cleanup
            self._physx_subscription = None
            self.ui_builder.cleanup()

        self.ui_builder.on_stage_event(event)

    def _build_extension_ui(self):
        # Call user function for building UI
        self.ui_builder.build_ui()
