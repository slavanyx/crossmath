"""Background compute so the UI never blocks during optimization."""
from __future__ import annotations

from PySide6 import QtCore


class _Signals(QtCore.QObject):
    done = QtCore.Signal(object)        # results dict
    compare_done = QtCore.Signal(object)
    failed = QtCore.Signal(str)


class ComputeWorker(QtCore.QRunnable):
    """Runs model.compute_current() (and optionally compare) off the UI thread."""

    def __init__(self, model, want_compare=False):
        super().__init__()
        self.model = model
        self.want_compare = want_compare
        self.signals = _Signals()

    @QtCore.Slot()
    def run(self):
        try:
            res = self.model.compute_current()
            self.signals.done.emit(res)
            if self.want_compare:
                # full stats (dev arrays + scalars) computed once, off the UI thread
                self.signals.compare_done.emit(self.model.compute_compare_full())
        except Exception as e:  # surface errors to the status bar, keep UI alive
            self.signals.failed.emit(f"{type(e).__name__}: {e}")
