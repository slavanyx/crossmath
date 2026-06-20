# Installing & running BladeCAM on Windows

BladeCAM has two layers:

1. a **Fortran numeric core** compiled to a shared library (`libbladecam.dll`), and
2. a **Python** layer (`numpy` for headless; `PySide6 + PyVista` for the 3D GUI;
   optional `cadquery-ocp` for STEP/IGES import).

So a Windows install is three things: a **Fortran toolchain** to build the DLL,
**Python** for the app, and wiring the two together. This guide uses the
free **MSYS2 / MinGW-w64** toolchain (the simplest reliable option) and a normal
Python from python.org. Commands are for **PowerShell** unless noted.

> Estimated time: ~30–45 min (most of it is installing the toolchains).

---

## 0. What you need

| Component | Recommended | Why |
|---|---|---|
| Git | [git-scm.com](https://git-scm.com/download/win) | clone the repo |
| Fortran + CMake + Make | **MSYS2** (gfortran, cmake, ninja) | build `libbladecam.dll` |
| Python | **3.11 or 3.12** (64-bit) from [python.org](https://www.python.org/downloads/windows/) | run the app |

Use **64-bit** everywhere (don't mix 32/64-bit Python and compiler).

---

## 1. Install Git

Download and run the installer from <https://git-scm.com/download/win> (defaults
are fine). Open a new PowerShell and check:

```powershell
git --version
```

## 2. Install the Fortran toolchain (MSYS2 / MinGW-w64)

1. Download and run the installer from <https://www.msys2.org> (default location
   `C:\msys64`).
2. Open **"MSYS2 MINGW64"** from the Start menu (the blue icon — *not* the plain
   "MSYS2 MSYS" one), then install the toolchain:

   ```bash
   pacman -Syu                 # update; if it closes the window, reopen and repeat
   pacman -S --needed mingw-w64-x86_64-gcc-fortran \
                      mingw-w64-x86_64-cmake \
                      mingw-w64-x86_64-ninja
   ```

3. Add MinGW's `bin` to your **Windows PATH** so the compiled DLL can find its
   runtime libraries (`libgfortran`, `libquadmath`, …) at run time. In
   PowerShell (one-time, per-user):

   ```powershell
   [Environment]::SetEnvironmentVariable(
     "Path", $env:Path + ";C:\msys64\mingw64\bin", "User")
   ```

   Close and reopen PowerShell, then verify:

   ```powershell
   gfortran --version
   cmake --version
   ```

   > If `gfortran` isn't found, your PATH didn't pick up `C:\msys64\mingw64\bin`
   > — re-check step 3.

## 3. Install Python

Install **Python 3.11/3.12 (64-bit)** from python.org. On the first installer
screen tick **"Add python.exe to PATH"**. Verify:

```powershell
python --version
python -m pip --version
```

---

## 4. Get the code

The project currently lives inside the `crossmath` repo under `bladecam/`:

```powershell
cd $HOME
git clone https://github.com/slavanyx/crossmath.git
cd crossmath\bladecam
```

(If you were given a feature branch, e.g. the development branch, check it out:
`git checkout claude/fortran-android-windows-1fzxs7`.)

---

## 5. Build the Fortran core (the DLL)

From `crossmath\bladecam` in PowerShell:

```powershell
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

This produces **`build\core\libbladecam.dll`**.

> **If CMake can't find a compiler**, run the two commands above from the
> **"MSYS2 MINGW64"** shell instead (it has the toolchain on its PATH). In that
> shell the source path is e.g. `/c/Users/<you>/crossmath/bladecam`.

Tell Python where the DLL is (the loader also auto-discovers it under `build\`,
but setting this is the robust way):

```powershell
$env:BLADECAM_LIB = "$PWD\build\core\libbladecam.dll"
```

To make it permanent:

```powershell
[Environment]::SetEnvironmentVariable(
  "BLADECAM_LIB", "$PWD\build\core\libbladecam.dll", "User")
```

---

## 6. Create a Python environment and install packages

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1        # prompt now shows (.venv)
python -m pip install --upgrade pip
```

> If activation is blocked by execution policy, run once:
> `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`

Pick one of these install levels:

```powershell
# a) headless only (numbers + CSV, no 3D window) — smallest
python -m pip install numpy

# b) full GUI (3D viewer) — recommended
python -m pip install -e ".[gui]"

# c) GUI + CAD import (load STEP/IGES blades) — heaviest (downloads OCP)
python -m pip install -e ".[gui,cad]"
```

`-e .` installs the `bladecam` package in editable mode and registers the
console commands `bladecam-gui`, `bladecam-demo`, `bladecam-benchmark`.

---

## 7. Run it

Make sure `BLADECAM_LIB` is set (step 5) and the venv is active (step 6).

```powershell
# headless end-to-end demo (writes a CSV, no window)
python demo.py

# the 3D GUI
python -m bladecam.viewer
#   ...or, after `pip install -e`:
bladecam-gui
```

Generate the worked-example galleries (needs the `gui` extra for the renders):

```powershell
$env:PYTHONPATH = "python"
python demos\make_demos.py
python demos\make_complex_demos.py     # includes 06_clean_showcase
```

---

## 8. Verify the build (optional but recommended)

```powershell
ctest --test-dir build --output-on-failure
```

All suites should pass. The Python suites need `numpy` (and `gui`/`cad` extras
for the GUI/CAD tests, which otherwise SKIP).

---

## 9. Troubleshooting

**`FileNotFoundError: Could not find bladecam.dll / libbladecam.dll`**
The Python loader couldn't find the core. Set `BLADECAM_LIB` to the full path of
`build\core\libbladecam.dll` (step 5), or confirm the build succeeded.

**`OSError: [WinError 126] The specified module could not be found`** when
loading the DLL
The DLL built, but its MinGW runtime dependencies (`libgfortran-5.dll`,
`libquadmath-0.dll`, `libgcc_s_seh-1.dll`, `libwinpthread-1.dll`) aren't on the
PATH. Either add `C:\msys64\mingw64\bin` to PATH (step 2.3) **or** copy those
four DLLs from `C:\msys64\mingw64\bin` into `build\core\` next to
`libbladecam.dll`.

**CMake picks MSVC instead of gfortran / "No CMAKE_Fortran_COMPILER"**
Build from the **MSYS2 MINGW64** shell, or pass the generator explicitly:
`cmake -S . -B build -G "Ninja"` after putting `C:\msys64\mingw64\bin` on PATH.
(The core is Fortran-only; MSVC has no Fortran compiler.)

**`bladecam-gui` opens no window / PyVista errors**
Update GPU drivers; PyVista needs OpenGL. On a headless/VM box, software
rendering still works for the demo renders but the live GUI wants a real GPU.

**`cadquery-ocp` install is slow or fails**
It's large and optional. Skip it (`".[gui]"` only) unless you need STEP/IGES
import; you can still use the parametric blade generator and CSV import.

**32-bit vs 64-bit mismatch**
A 64-bit Python cannot load a 32-bit DLL (or vice-versa). Use 64-bit for both
(MSYS2 *MINGW64* + 64-bit python.org Python).

---

## Quick reference (all steps, copy-paste)

```powershell
# after installing Git, MSYS2 (with the mingw64 toolchain on PATH), and Python:
git clone https://github.com/slavanyx/crossmath.git
cd crossmath\bladecam
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
$env:BLADECAM_LIB = "$PWD\build\core\libbladecam.dll"
python -m venv .venv; .\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[gui]"
python -m bladecam.viewer
```
