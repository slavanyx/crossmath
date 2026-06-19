"""Real-time BladeCAM GUI (PySide6 + PyVista).

Live 3D view of the blade flank coloured by flank-milling deviation, the
positioned cutter axes, striction curve and neighbour blade, with controls to
change geometry / tool / strategy / machine parameters and see deviation,
orientation jerk, cycle time and collision update in real time.

Run (after building the core, see README):
    pip install -r requirements.txt
    PYTHONPATH=. python -m bladecam.viewer
"""
from __future__ import annotations

import numpy as np

from .pipeline import Params, compute
from .process import MachineLimits, ProcessParams
from . import postproc


def _build_params(s) -> Params:
    return Params(
        nu=int(s["nu"]),
        r_hub=s["r_hub"], r_shroud=s["r_shroud"], z_span=s["z_span"],
        wrap=s["wrap"], twist=s["twist"], n_blades=int(s["n_blades"]),
        R=s["R"], strategy=s["strategy"], smooth_window=int(s["smooth"]),
        machine=MachineLimits(v_rot=s["v_rot"]),
        process=ProcessParams(feed_max_mm_min=s["feed_max"],
                              dev_allow_um=s["dev_allow"]),
    )


def main():
    from PySide6 import QtWidgets, QtCore
    from pyvistaqt import QtInteractor
    import pyvista as pv

    class Window(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("BladeCAM - 5-axis flank milling")
            self.last = None

            central = QtWidgets.QWidget(); self.setCentralWidget(central)
            root = QtWidgets.QHBoxLayout(central)

            # ---- control panel ----
            panel = QtWidgets.QFormLayout()
            self.w = {}
            self._spin(panel, "R", 6.0, 0.5, 30.0, 0.5, "Cutter radius (mm)")
            self._spin(panel, "twist", 0.7, 0.0, 2.0, 0.05, "Blade twist (rad)")
            self._spin(panel, "wrap", 0.6, 0.0, 2.0, 0.05, "Blade wrap (rad)")
            self._spin(panel, "r_hub", 30.0, 5.0, 100.0, 1.0, "Hub radius (mm)")
            self._spin(panel, "r_shroud", 55.0, 5.0, 150.0, 1.0, "Shroud radius (mm)")
            self._spin(panel, "z_span", 20.0, 5.0, 100.0, 1.0, "Blade height (mm)")
            self._spin(panel, "nu", 60, 20, 200, 5, "Stations", decimals=0)
            self._spin(panel, "n_blades", 11, 3, 40, 1, "Blade count", decimals=0)
            self._spin(panel, "smooth", 5, 1, 21, 2, "Smooth window", decimals=0)
            self._spin(panel, "v_rot", 0.6, 0.05, 3.0, 0.05, "Rotary vmax (rad/s)")
            self._spin(panel, "feed_max", 6000, 200, 20000, 200,
                       "Feed ceiling (mm/min)", decimals=0)
            self._spin(panel, "dev_allow", 50, 5, 500, 5,
                       "Deflection budget (um)", decimals=0)

            self.strategy = QtWidgets.QComboBox()
            self.strategy.addItems(["minmax", "smoothed", "two_point"])
            self.strategy.currentTextChanged.connect(self.schedule)
            panel.addRow("Strategy", self.strategy)

            self.save_btn = QtWidgets.QPushButton("Save G-code…")
            self.save_btn.clicked.connect(self.save_gcode)
            panel.addRow(self.save_btn)

            self.stats = QtWidgets.QLabel(); self.stats.setWordWrap(True)
            self.stats.setStyleSheet("font-family: monospace;")
            panel.addRow(self.stats)

            pw = QtWidgets.QWidget(); pw.setLayout(panel); pw.setMaximumWidth(330)
            root.addWidget(pw)

            self.plotter = QtInteractor(central)
            root.addWidget(self.plotter.interactor, stretch=1)

            self._timer = QtCore.QTimer(singleShot=True)
            self._timer.timeout.connect(self.recompute)
            self.recompute()

        def _spin(self, form, key, val, lo, hi, step, label, decimals=2):
            sb = (QtWidgets.QSpinBox() if decimals == 0
                  else QtWidgets.QDoubleSpinBox())
            sb.setRange(int(lo) if decimals == 0 else lo,
                        int(hi) if decimals == 0 else hi)
            if decimals != 0:
                sb.setDecimals(decimals)
            sb.setSingleStep(int(step) if decimals == 0 else step)
            sb.setValue(int(val) if decimals == 0 else val)
            sb.valueChanged.connect(self.schedule)
            self.w[key] = sb
            form.addRow(label, sb)

        def _state(self):
            s = {k: v.value() for k, v in self.w.items()}
            s["strategy"] = self.strategy.currentText()
            return s

        def schedule(self):
            self._timer.start(120)   # debounce rapid slider changes

        def recompute(self):
            p = _build_params(self._state())
            try:
                r = compute(p)
            except Exception as e:  # keep the GUI alive on bad parameter combos
                self.stats.setText(f"error: {e}")
                return
            self.last = (r, p)
            self._draw(r)

        def _draw(self, r):
            pv_local = __import__("pyvista")
            surf = r["surf"]; nu, nv, _ = surf.shape
            grid = pv_local.StructuredGrid()
            grid.points = surf.reshape(-1, 3)
            grid.dimensions = (nv, nu, 1)
            grid["dev_um"] = (r["devfield"].reshape(-1) * 1000.0)

            cam = self.plotter.camera_position
            self.plotter.clear()
            self.plotter.add_mesh(grid, scalars="dev_um", cmap="coolwarm",
                                  scalar_bar_args={"title": "dev (um)"})
            self.plotter.add_mesh(pv_local.lines_from_points(r["strict"]),
                                  color="lime", line_width=3)
            q0 = r["q0"]; al = r["alpha"]
            for i in range(0, nu, max(1, nu // 14)):
                line = pv_local.Line(q0[i] - al[i]*5.0, q0[i] + al[i]*30.0)
                self.plotter.add_mesh(line, color="black", line_width=2)
            self.plotter.camera_position = cam
            if cam is None:
                self.plotter.reset_camera()

            coll = "OK" if r["collision_free"] else "** COLLISION **"
            self.stats.setText(
                f"peak dev   : {r['dev'].max()*1000:7.1f} um\n"
                f"mean dev   : {r['dev'].mean()*1000:7.1f} um\n"
                f"orient jerk: {r['orient_jerk']:7.3f}\n"
                f"cycle time : {r['cycle_time_s']:7.2f} s\n"
                f"path length: {r['path_len_mm']:7.1f} mm\n"
                f"feed cap   : {r['feed_cap_mm_min']:7.0f} mm/min\n"
                f"clearance  : {r['min_clearance']:7.2f} mm  {coll}")

        def save_gcode(self):
            if not self.last:
                return
            r, _ = self.last
            fn, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "Save G-code", "bladecam.nc", "G-code (*.nc)")
            if fn:
                with open(fn, "w") as f:
                    f.write(postproc.to_gcode(r["machine_path"],
                                              r["feed_cap_mm_min"]))

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = Window(); win.resize(1280, 820); win.show()
    app.exec()


if __name__ == "__main__":
    main()
