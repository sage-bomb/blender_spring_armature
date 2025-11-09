"""Spring Bones add-on package."""

from __future__ import annotations

import importlib

from . import draw, handlers, operators, panel, properties, simulation, utils

bl_info = {
    "name": "Spring Bones",
    "author": "sage-bomb",
    "version": (0, 9),
    "blender": (4, 4, 0),
    "location": "3D Viewport > Sidebar (N) > Spring Bones",
    "description": "Add a spring dynamic effect to a single/multiple bones",
    "category": "Animation",
}

_MODULES = (utils, simulation, draw, properties, operators, panel, handlers)


def register():
    for module in _MODULES:
        importlib.reload(module)

    properties.register()
    operators.register()
    panel.register()
    handlers.register()


def unregister():
    handlers.unregister()
    panel.unregister()
    operators.unregister()
    properties.unregister()
    draw.remove_force_viz_handler()


__all__ = ["register", "unregister", "bl_info"]
