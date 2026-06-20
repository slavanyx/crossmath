"""BladeCAM: 5-axis flank-milling tool-positioning toolkit for impeller blades.

A Fortran numeric core (C ABI, via ctypes) under a Python application layer
(pipeline, CAD I/O, presets, certified posts) and a PySide6 + PyVista GUI.
"""
from . import blade  # noqa: F401

__all__ = ["blade", "core"]
__version__ = "0.5.0"
