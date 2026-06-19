# PyInstaller spec for the BladeCAM desktop GUI.
#
# Build the Fortran core first (cmake --build build), then from bladecam/python:
#     pip install -e ".[gui,cad]" pyinstaller
#     pyinstaller bladecam_gui.spec
# The shared library is bundled next to the app; BLADECAM_LIB can override it.
import os
from PyInstaller.utils.hooks import collect_submodules

block_cipher = None

_here = os.path.abspath(os.getcwd())
_lib_candidates = [
    os.path.join(_here, "..", "build", "core", "libbladecam.so"),
    os.path.join(_here, "..", "build", "core", "libbladecam.dylib"),
    os.path.join(_here, "..", "build", "core", "bladecam.dll"),
]
_binaries = [(p, "bladecam") for p in _lib_candidates if os.path.exists(p)]

a = Analysis(
    ["-m", "bladecam.viewer"],
    pathex=["."],
    binaries=_binaries,
    datas=[],
    hiddenimports=collect_submodules("pyvistaqt") + collect_submodules("vtkmodules"),
    hookspath=[],
    runtime_hooks=[],
    excludes=[],
    cipher=block_cipher,
)
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)
exe = EXE(pyz, a.scripts, [], exclude_binaries=True, name="bladecam",
          console=False)
coll = COLLECT(exe, a.binaries, a.zipfiles, a.datas, name="bladecam")
