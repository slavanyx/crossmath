"""Back-compat entry point. The GUI now lives in the architected `gui` package.

Run either:
    PYTHONPATH=. python -m bladecam.viewer
    PYTHONPATH=. python -m bladecam.gui.main
"""
from __future__ import annotations


def main():
    from .gui.main import main as gui_main
    gui_main()


if __name__ == "__main__":
    main()
