"""Desktop GUI: PySide6 window embedding a PyVista 3D view of the blade flank
coloured by flank-milling deviation, with the positioned cutter shown.

Run:  python -m bladecam.viewer   (after building the core, see README)

This is the interactive front-end; the heavy geometry runs in the Fortran
core via bladecam.core. It is not exercised by the headless test suite.
"""
from __future__ import annotations

import numpy as np

from . import core, blade


def compute_field(R: float, nv: int = 30):
    """Return (surface[nu,nv,3], dev[nu,nv], axes[list of (q0,alpha)])."""
    a, b = blade.make_blade()
    ap, bp = blade.rail_tangents(a, b)
    nu = a.shape[0]
    surf = blade.surface(a, b, nv)
    v = np.linspace(0.0, 1.0, nv)
    dev = np.empty((nu, nv))
    axes = []
    for i in range(nu):
        q0, alpha = core.two_point(a[i], ap[i], b[i], bp[i], R)
        pts = (1.0 - v)[:, None] * a[i][None, :] + v[:, None] * b[i][None, :]
        dev[i, :] = core.deviation(q0, alpha, R, pts)
        axes.append((q0, alpha))
    return surf, dev, axes


def main():
    # Imported lazily so the headless core/tests have no GUI dependency.
    from PySide6 import QtWidgets
    from pyvistaqt import QtInteractor
    import pyvista as pv

    R_default = 6.0

    class Window(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("BladeCAM - flank milling deviation")
            central = QtWidgets.QWidget()
            self.setCentralWidget(central)
            layout = QtWidgets.QVBoxLayout(central)

            ctrl = QtWidgets.QHBoxLayout()
            ctrl.addWidget(QtWidgets.QLabel("Cutter radius R (mm):"))
            self.spin = QtWidgets.QDoubleSpinBox()
            self.spin.setRange(0.5, 30.0)
            self.spin.setValue(R_default)
            self.spin.setSingleStep(0.5)
            self.spin.valueChanged.connect(self.refresh)
            ctrl.addWidget(self.spin)
            ctrl.addStretch()
            self.stats = QtWidgets.QLabel("")
            ctrl.addWidget(self.stats)
            layout.addLayout(ctrl)

            self.plotter = QtInteractor(central)
            layout.addWidget(self.plotter.interactor)
            self.refresh()

        def refresh(self):
            R = self.spin.value()
            surf, dev, axes = compute_field(R)
            nu, nv, _ = surf.shape
            grid = pv.StructuredGrid()
            grid.points = surf.reshape(-1, 3)
            grid.dimensions = (nv, nu, 1)
            grid["deviation_um"] = (dev.reshape(-1) * 1000.0)

            self.plotter.clear()
            self.plotter.add_mesh(grid, scalars="deviation_um",
                                  cmap="coolwarm", show_edges=False,
                                  scalar_bar_args={"title": "dev (um)"})
            # draw a few cutter axes
            for q0, alpha in axes[::max(1, nu // 12)]:
                p0 = q0 - alpha * 5.0
                p1 = q0 + alpha * 30.0
                self.plotter.add_mesh(pv.Line(p0, p1), color="black",
                                      line_width=2)
            self.plotter.reset_camera()
            self.stats.setText(
                f"max |dev| = {np.max(np.abs(dev))*1000:.1f} um")

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w = Window()
    w.resize(1100, 750)
    w.show()
    app.exec()


if __name__ == "__main__":
    main()
