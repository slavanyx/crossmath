"""Professional dockable main window for BladeCAM.

Architecture:
  - AppModel        : Qt-free state + compute (gui/model.py)
  - ComputeWorker   : background pipeline runs (gui/worker.py)
  - charts          : Qt-free matplotlib figures (gui/charts.py)
  - MainWindow      : dockable shell wiring views to the model

Docks (all rearrangeable / floatable / closable like real engineering tools):
  Parameters (left) | 3D view (centre) | Results (right) | Analysis plots (bottom)

Run:  PYTHONPATH=. python -m bladecam.gui.main      (or: python -m bladecam.viewer)
"""
from __future__ import annotations

import numpy as np

from PySide6 import QtCore, QtWidgets, QtGui
from pyvistaqt import QtInteractor
import pyvista as pv
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg

from .model import AppModel, PARAM_SPEC, MACHINE_SPEC, TOOL_SPEC, STRATEGIES
from .worker import ComputeWorker, OpWorker
from . import charts
from . import help as helpdoc
from .. import postproc, cadio, workflow
from .. import machine as machine_lib


def _reach_str(r):
    """Format machine reachability for the results panel."""
    if r.get("reachable", True):
        return "yes"
    v = r.get("axis_violations", {})
    return "NO — " + ", ".join(f"{ax} by {e:.0f}" for ax, e in v.items())


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.model = AppModel()
        self.pool = QtCore.QThreadPool.globalInstance()
        self.last = None
        self._editors = {}
        self._overlay = None      # persistent imported-CAD overlay mesh
        self._blisk = None        # list of (a,b) rails for a loaded blisk
        self._blisk_i = 0
        self._stage_idx = None    # None = overview; else index into workflow.STAGES

        self.setWindowTitle("BladeCAM — 5-axis flank milling")
        self.setDockNestingEnabled(True)

        self._build_menu()
        self._build_toolbar()
        self._build_3d_view()       # central
        self._build_param_dock()    # left
        self._build_results_dock()  # right
        self._build_plots_dock()    # bottom
        self._build_guide_dock()    # right (tabbed with Results)
        self.status = self.statusBar()
        self._maybe_welcome()

        self._timer = QtCore.QTimer(singleShot=True)
        self._timer.timeout.connect(lambda: self.recompute(compare=True))
        self.recompute(compare=True)

    # ---- shell --------------------------------------------------------------
    def _build_menu(self):
        mb = self.menuBar()
        # --- File: import / blade source / export ---
        filem = mb.addMenu("&File")
        self._act(filem, "Open project…", self.open_project, "Ctrl+O")
        self._act(filem, "Save project…", self.save_project, "Ctrl+S")
        filem.addSeparator()
        self._act(filem, "Import rails CSV…", self.import_rails)
        self._act(filem, "Load blade from STEP/IGES…", self.load_blade_cad)
        self._act(filem, "Load blisk (all blades)…", self.load_blisk)
        self._act(filem, "Next blisk blade", self.next_blisk_blade)
        self._act(filem, "Overlay CAD (STL/STEP/IGES)…", self.import_cad)
        self._act(filem, "Load fixture/machine body (STL/STEP)…", self.load_fixture)
        self._act(filem, "Load tool-tip FRF (CSV)…", self.load_frf)
        self._act(filem, "Use parametric blade", self.use_parametric)
        filem.addSeparator()
        self._act(filem, "Export blade STL…", self.export_stl)
        self._act(filem, "Export rails CSV…", self.export_rails)
        self._act(filem, "Save G-code…", self.save_gcode)
        self._act(filem, "Save certified G-code…", self.save_certified)
        self._act(filem, "Save Heidenhain klartext (.h)…", self.save_heidenhain)
        filem.addSeparator()
        self._act(filem, "Export presets…", self.export_presets)
        self._act(filem, "Import presets…", self.import_presets)
        filem.addSeparator()
        self._act(filem, "Quit", self.close, "Ctrl+Q")
        # --- Operations: every milling operation as a first-class action ---
        opm = mb.addMenu("&Operations")
        self._act(opm, "Flank finish (live)", lambda: self.recompute(compare=True))
        self._act(opm, "Double-flank channel", self.show_double_flank)
        self._act(opm, "Channel roughing (show passes)", self.show_roughing)
        self._act(opm, "Channel roughing — trochoidal", self.show_trochoidal)
        self._act(opm, "Rest-machining (dexel stock)", self.show_rest_machining)
        self._act(opm, "Edge finishing (point-mill)", self.show_edge_finish)
        self._act(opm, "Root-fillet finishing (ball-nose)", self.show_fillet_machining)
        opm.addSeparator()
        self._act(opm, "Machined envelope (swept surface)", self.show_envelope)
        self._act(opm, "Minimize swept overcut", self.minimize_swept_overcut)
        self._act(opm, "Process plan (full report)", self.show_process_plan)
        self.viewm = mb.addMenu("&View")    # dock toggles added later
        helpm = mb.addMenu("&Help")
        self._act(helpm, "Getting started", self.show_getting_started, "F1")
        self._act(helpm, "Quick start", self.show_quick_start)
        self._act(helpm, "Glossary", self.show_glossary)
        self._act(helpm, "What's this?", QtWidgets.QWhatsThis.enterWhatsThisMode,
                  "Shift+F1")
        helpm.addSeparator()
        self._act(helpm, "About", self.about)

    def _build_toolbar(self):
        # OrcaSlicer-style preset row: Machine / Tool / Strategy presets
        pb = self.addToolBar("Presets")
        self.preset_cbs = {}
        for kind in ("blade", "machine", "tool", "strategy", "post"):
            pb.addWidget(QtWidgets.QLabel(f"  {kind.title()}: "))
            cb = QtWidgets.QComboBox()
            cb.setMinimumWidth(150)
            cb.addItems(self.model.presets.names(kind))
            cb.setCurrentText(self.model.preset_names[kind])
            cb.currentTextChanged.connect(lambda n, k=kind: self._on_preset(k, n))
            self.preset_cbs[kind] = cb
            pb.addWidget(cb)
        pb.addAction("Save preset…", self.save_preset_dialog)
        pb.addAction("Delete preset…", self.delete_preset_dialog)
        pb.addAction("Machine config…", self.edit_machine)
        pb.addAction("Post config…", self.edit_post)
        self.dirty_lbl = QtWidgets.QLabel("")
        self.dirty_lbl.setStyleSheet("color: #c0392b; font-weight: bold;")
        pb.addWidget(self.dirty_lbl)

        tb = self.addToolBar("Main")
        self.insertToolBarBreak(tb)
        # OrcaSlicer-style Prepare / Preview top-level mode
        self.prepare_act = tb.addAction("Prepare", lambda: self._set_mode(False))
        self.preview_act = tb.addAction("Preview", lambda: self._set_mode(True))
        self.prepare_act.setStatusTip("Prepare: edit the blade, tool, machine and "
                                      "strategy parameters.")
        self.preview_act.setStatusTip("Preview: step through the CAM stages and "
                                      "inspect the result in 3D.")
        for a in (self.prepare_act, self.preview_act):
            a.setCheckable(True)
        self.prepare_act.setChecked(True)
        tb.addSeparator()
        tb.addWidget(QtWidgets.QLabel(" Strategy: "))
        self.strategy_cb = QtWidgets.QComboBox()
        self.strategy_cb.addItems(STRATEGIES)
        self.strategy_cb.currentTextChanged.connect(self._on_strategy)
        tb.addWidget(self.strategy_cb)
        tb.addAction("Recompute", lambda: self.recompute(compare=True))
        tb.addAction("Save G-code", self.save_gcode)
        tb.addSeparator()
        # toolpath animation controls
        self.play_act = tb.addAction("▶ Play", self._toggle_play)
        self.anim_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.anim_slider.setRange(0, 1)
        self.anim_slider.valueChanged.connect(self._show_tool_at)
        tb.addWidget(self.anim_slider)
        self._anim_timer = QtCore.QTimer()
        self._anim_timer.setInterval(60)
        self._anim_timer.timeout.connect(self._anim_step)

        # --- workflow stepper: flow through the CAM stages in 3D ---
        wf = self.addToolBar("Workflow")
        self.insertToolBarBreak(wf)
        wf.addWidget(QtWidgets.QLabel(" Workflow: "))
        ov = wf.addAction("⊞ Overview", self._show_overview)
        ov.setStatusTip("Show the whole toolpath and the quick-start guide.")
        pv_ = wf.addAction("◀ Prev", lambda: self._step_stage(-1))
        pv_.setStatusTip("Previous CAM stage.")
        self.stage_lbl = QtWidgets.QLabel("  (overview)  ")
        self.stage_lbl.setStyleSheet("font-weight: bold;")
        wf.addWidget(self.stage_lbl)
        nx = wf.addAction("Next ▶", lambda: self._step_stage(+1))
        nx.setStatusTip("Next CAM stage — the Guide panel explains each one.")
        self._trail = True
        trail_act = wf.addAction("Sweep trail", self._toggle_trail)
        trail_act.setCheckable(True)
        trail_act.setChecked(True)
        self._sim_mode = False
        sim_act = wf.addAction("Simulate removal", self._toggle_sim)
        sim_act.setCheckable(True)

    def _build_3d_view(self):
        self.plotter = QtInteractor(self)
        self.setCentralWidget(self.plotter.interactor)

    def _build_param_dock(self):
        dock = QtWidgets.QDockWidget("Parameters", self)
        scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True)
        host = QtWidgets.QWidget(); vbox = QtWidgets.QVBoxLayout(host)

        groups = {}
        for spec in PARAM_SPEC + TOOL_SPEC + MACHINE_SPEC:
            key, label, lo, hi, step, kind, group = spec
            groups.setdefault(group, []).append(spec)
        for group, specs in groups.items():
            box = QtWidgets.QGroupBox(group); form = QtWidgets.QFormLayout(box)
            for key, label, lo, hi, step, kind, _g in specs:
                ed = (QtWidgets.QSpinBox() if kind == "int"
                      else QtWidgets.QDoubleSpinBox())
                ed.setRange(lo, hi); ed.setSingleStep(step)
                if kind != "int":
                    ed.setDecimals(3)
                ed.setValue(self.model.values[key])
                ed.valueChanged.connect(self._on_param)
                tip = helpdoc.param_tip(key)
                if tip:
                    ed.setToolTip(tip); ed.setStatusTip(tip)
                    ed.setWhatsThis(tip)
                self._editors[key] = ed
                form.addRow(label, ed)
            vbox.addWidget(box)
        vbox.addStretch()
        scroll.setWidget(host); dock.setWidget(scroll)
        dock.setMaximumWidth(340)
        self.param_dock = dock
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, dock)
        self.viewm.addAction(dock.toggleViewAction())

    def _build_results_dock(self):
        dock = QtWidgets.QDockWidget("Results", self)
        self.results_tbl = QtWidgets.QTableWidget(0, 2)
        self.results_tbl.horizontalHeader().setStretchLastSection(True)
        self.results_tbl.setHorizontalHeaderLabels(["metric", "value"])
        self.results_tbl.verticalHeader().setVisible(False)
        dock.setWidget(self.results_tbl); dock.setMaximumWidth(300)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)
        self.viewm.addAction(dock.toggleViewAction())
        self.results_dock = dock

    def _build_guide_dock(self):
        """A learn-as-you-go panel: shows what the current view means and the
        next step, plus a quick-start in Overview. Tabbed with Results."""
        dock = QtWidgets.QDockWidget("Guide", self)
        self.guide = QtWidgets.QTextBrowser()
        self.guide.setOpenExternalLinks(False)
        dock.setWidget(self.guide); dock.setMaximumWidth(340)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)
        if hasattr(self, "results_dock"):
            self.tabifyDockWidget(self.results_dock, dock)
            self.results_dock.raise_()
        self.viewm.addAction(dock.toggleViewAction())
        self.guide_dock = dock
        self._show_overview_guide()

    def _show_overview_guide(self):
        steps = "".join(f"<li>{s}</li>" for s in helpdoc.QUICK_START)
        self.guide.setHtml(
            "<h3>Quick start</h3><ol>" + steps + "</ol>"
            "<p style='color:gray'>Hover any parameter or results row for help. "
            "Help ▸ Glossary explains the terms.</p>")

    def _set_stage_guide(self, key):
        title = next((t for k, t, _b in workflow.STAGES if k == key), key)
        self.guide.setHtml(
            f"<h3>{title}</h3><p>{helpdoc.stage_help(key)}</p>")
        if hasattr(self, "guide_dock"):
            self.guide_dock.raise_()

    def _build_plots_dock(self):
        dock = QtWidgets.QDockWidget("Analysis", self)
        self.tabs = QtWidgets.QTabWidget()
        self.canvases = {}
        for name in ("Deviation", "Machinability", "Kinematics", "Feed",
                     "Compare", "Chatter"):
            from matplotlib.figure import Figure
            c = FigureCanvasQTAgg(Figure(figsize=(5, 3)))
            self.canvases[name] = c
            self.tabs.addTab(c, name)
        dock.setWidget(self.tabs)
        self.plots_dock = dock
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, dock)
        self.viewm.addAction(dock.toggleViewAction())

    # ---- events -------------------------------------------------------------
    def _on_param(self):
        for key, ed in self._editors.items():
            self.model.values[key] = ed.value()
        self._refresh_dirty()
        self._timer.start(150)

    def _on_strategy(self, s):
        self.model.strategy = s
        self._refresh_dirty()
        self._timer.start(50)

    def _on_preset(self, kind, name):
        """Apply a Machine/Tool/Strategy preset and refresh the editors."""
        if not name:
            return
        self.model.apply_preset(kind, name)
        if kind == "strategy":
            self.strategy_cb.setCurrentText(self.model.strategy)
        self._sync_editors()
        self._timer.start(50)

    def _sync_editors(self):
        """Push model.values back into the parameter spin-boxes (after a preset
        changes them) without retriggering recompute per edit."""
        for key, ed in self._editors.items():
            if key in self.model.values:
                ed.blockSignals(True)
                ed.setValue(self.model.values[key])
                ed.blockSignals(False)
        self._refresh_dirty()

    def _refresh_dirty(self):
        """OrcaSlicer 'modified ●' indicator: which presets differ from saved."""
        dirty = self.model.dirty_kinds()
        self.dirty_lbl.setText("  ● modified: " + ", ".join(dirty) if dirty else "")

    def _set_mode(self, preview: bool):
        """Prepare (setup/parameters) vs Preview (toolpath/verify) top-level
        mode, OrcaSlicer-style. Prepare shows the parameter dock; Preview hides
        it for a larger 3D view and surfaces the results/animation."""
        self.prepare_act.setChecked(not preview)
        self.preview_act.setChecked(preview)
        self.param_dock.setVisible(not preview)
        if preview and self.last is not None:
            self._show_overview()          # reset to the full toolpath view

    def export_presets(self):
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export presets", "bladecam.presets.json",
            "Preset bundle (*.json)")
        if fn:
            n = self.model.presets.export_bundle(fn)
            self.status.showMessage(f"exported {n} user presets -> {fn}")

    def import_presets(self):
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Import presets", "", "Preset bundle (*.json)")
        if not fn:
            return
        try:
            n = self.model.presets.import_bundle(fn)
        except Exception as e:
            self.status.showMessage(f"import failed: {e}")
            return
        for kind, cb in self.preset_cbs.items():       # refresh the combos
            cur = cb.currentText()
            cb.blockSignals(True); cb.clear()
            cb.addItems(self.model.presets.names(kind))
            cb.setCurrentText(cur); cb.blockSignals(False)
        self.status.showMessage(f"imported {n} presets from {fn}")

    def save_preset_dialog(self):
        kind, ok = QtWidgets.QInputDialog.getItem(
            self, "Save preset", "Category:", list(self.model.presets.KINDS), 0, False)
        if not ok:
            return
        name, ok = QtWidgets.QInputDialog.getText(
            self, "Save preset", f"Name for the {kind} preset:")
        if not ok or not name:
            return
        self.model.save_preset(kind, name)
        cb = self.preset_cbs[kind]
        if cb.findText(name) < 0:
            cb.addItem(name)
        cb.blockSignals(True); cb.setCurrentText(name); cb.blockSignals(False)
        self._refresh_dirty()
        self.status.showMessage(f"saved {kind} preset '{name}'")

    def delete_preset_dialog(self):
        kind, ok = QtWidgets.QInputDialog.getItem(
            self, "Delete preset", "Category:", list(self.model.presets.KINDS), 0, False)
        if not ok:
            return
        name = self.preset_cbs[kind].currentText()
        if self.model.presets.is_builtin(kind, name):
            self.status.showMessage(f"'{name}' is a built-in preset (read-only)")
            return
        if self.model.presets.delete(kind, name):
            idx = self.preset_cbs[kind].findText(name)
            if idx >= 0:
                self.preset_cbs[kind].removeItem(idx)
            self.status.showMessage(f"deleted {kind} preset '{name}'")

    def edit_machine(self):
        """Machine configuration editor: edit the active profile's travel/rotary
        envelope, kinematic limits and spindle/table geometry, then recompute."""
        from dataclasses import fields, replace
        m = self.model.machine
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle(f"Machine config — {m.name}")
        form = QtWidgets.QFormLayout(dlg)
        editors = {}
        # numeric scalar fields, derived from the dataclass so every machine
        # parameter (incl. structural cradle/column links) is always editable
        rng = ["x_range", "y_range", "z_range", "a_range", "c_range"]
        skip = set(rng) | {"name", "kind"}   # name=title; kind=quick editor
        scal = [f.name for f in fields(m) if f.name not in skip]
        for fn in scal:
            ed = QtWidgets.QDoubleSpinBox()
            ed.setRange(0.0, 1e6); ed.setDecimals(3)
            ed.setValue(float(getattr(m, fn)))
            editors[fn] = ed; form.addRow(fn, ed)
        # range fields (min,max) as two spinboxes
        for fn in rng:
            lo, hi = getattr(m, fn)
            elo = QtWidgets.QDoubleSpinBox(); ehi = QtWidgets.QDoubleSpinBox()
            for e, val in ((elo, lo), (ehi, hi)):
                e.setRange(-1e6, 1e6); e.setDecimals(4); e.setValue(float(val))
            row = QtWidgets.QWidget(); h = QtWidgets.QHBoxLayout(row)
            h.setContentsMargins(0, 0, 0, 0); h.addWidget(elo); h.addWidget(ehi)
            editors[fn] = (elo, ehi); form.addRow(fn, row)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok |
                                        QtWidgets.QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept); bb.rejected.connect(dlg.reject)
        form.addRow(bb)
        if dlg.exec() != QtWidgets.QDialog.Accepted:
            return
        upd = {}
        for fn in scal:
            upd[fn] = editors[fn].value()
        for fn in rng:
            upd[fn] = (editors[fn][0].value(), editors[fn][1].value())
        self.model.machine = replace(m, **upd)
        self.recompute(compare=True)

    def save_project(self):
        """Save the whole job (params, machine, post, blade/CAD rails) to a
        single self-contained .bladecam project file."""
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save project", "job.bladecam", "BladeCAM project (*.bladecam)")
        if fn:
            from . import model as model_mod
            model_mod.save_project(fn, self.model)
            self.status.showMessage(f"project saved -> {fn}")

    def open_project(self):
        """Open a .bladecam project, restore the full editable state, refresh the
        editors/preset combos and recompute."""
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open project", "", "BladeCAM project (*.bladecam)")
        if not fn:
            return
        from . import model as model_mod
        model_mod.load_project(fn, self.model)
        self._sync_editors()                 # push restored values into the spinboxes
        for kind, cb in self.preset_cbs.items():
            cb.setCurrentText(self.model.preset_names.get(kind, cb.currentText()))
        self._refresh_dirty()
        self.recompute(compare=True)
        self.status.showMessage(f"project loaded <- {fn}")

    def edit_post(self):
        """Certified-post editor: control dialect, axis letters/signs, limits and
        tolerances of the active machine/control pairing."""
        from dataclasses import fields, replace
        from .. import post as post_lib
        c = self.model.post
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle(f"Post config — {c.name}")
        form = QtWidgets.QFormLayout(dlg)
        editors = {}
        for f in fields(c):
            val = getattr(c, f.name)
            if f.name == "control":
                ed = QtWidgets.QComboBox(); ed.addItems(["heidenhain", "siemens", "fanuc"])
                ed.setCurrentText(val)
            elif isinstance(val, bool):
                ed = QtWidgets.QCheckBox(); ed.setChecked(val)
            elif isinstance(val, (int, float)) and not isinstance(val, bool):
                ed = QtWidgets.QDoubleSpinBox(); ed.setRange(-1e6, 1e6)
                ed.setDecimals(4); ed.setValue(float(val))
            else:
                ed = QtWidgets.QLineEdit(str(val))
            editors[f.name] = ed; form.addRow(f.name, ed)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok |
                                        QtWidgets.QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept); bb.rejected.connect(dlg.reject)
        form.addRow(bb)
        if dlg.exec() != QtWidgets.QDialog.Accepted:
            return
        upd = {}
        for f in fields(c):
            ed = editors[f.name]
            if isinstance(ed, QtWidgets.QComboBox):
                upd[f.name] = ed.currentText()
            elif isinstance(ed, QtWidgets.QCheckBox):
                upd[f.name] = ed.isChecked()
            elif isinstance(ed, QtWidgets.QDoubleSpinBox):
                v = ed.value()
                upd[f.name] = int(v) if isinstance(getattr(c, f.name), int) else v
            else:
                upd[f.name] = ed.text()
        self.model.post = replace(c, **upd)
        self._refresh_dirty()

    def save_certified(self):
        """Generate the active certified post's program and report whether it
        certifies on the live machine before saving."""
        if not self.last:
            return
        text, rep = self.model.post_program(self.last)
        ok = rep["certified"]
        ext = {"heidenhain": "*.h", "siemens": "*.mpf", "fanuc": "*.nc"}.get(
            self.model.post.control, "*.nc")
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save certified G-code", "bladecam" + ext.strip("*"),
            f"Program ({ext})")
        if not fn:
            return
        with open(fn, "w") as fh:
            fh.write(text)
        if ok:
            self.status.showMessage(f"CERTIFIED on {rep['machine']} "
                                    f"(round-trip {rep['roundtrip_max_err_mm']:.1e} mm) -> {fn}")
        else:
            bad = [k for k in ("within_travel", "within_rotary", "winding_ok",
                               "linearization_ok", "rotary_speed_ok", "roundtrip_ok")
                   if not rep[k]]
            QtWidgets.QMessageBox.warning(
                self, "Post not certified",
                f"Saved, but FAILED certification on {rep['machine']}:\n"
                + ", ".join(bad))

    def _run_bg(self, fn, on_done, busy="working…"):
        """Run a heavy operation off the UI thread so the 3D view stays
        interactive; deliver the result to on_done on the UI thread."""
        self.status.showMessage(busy)
        w = OpWorker(fn)
        w.signals.done.connect(on_done)
        w.signals.failed.connect(lambda m: self.status.showMessage(m))
        self.pool.start(w)

    def recompute(self, compare=False):
        self.status.showMessage("computing…")
        w = ComputeWorker(self.model, want_compare=compare)
        w.signals.done.connect(self._on_results)
        w.signals.compare_done.connect(self._on_compare)
        w.signals.failed.connect(lambda m: self.status.showMessage(m))
        self.pool.start(w)

    def _on_results(self, r):
        self.last = r
        self.anim_slider.setRange(0, r["q0"].shape[0] - 1)
        if self._stage_idx is not None:
            self._goto_stage(self._stage_idx)   # keep the active workflow stage
        else:
            self._draw_3d(r)
            self._fill_results(r)
        self._update_chart("Machinability",
                           charts.machinability_chart, r["delta"], r["dev"])
        self._update_chart("Feed", charts.feed_chart, r["seglen"], r["aprof"])
        self._update_chatter()
        coll = "OK" if r["collision_free"] else "COLLISION"
        # headline the swept (machined-surface) error: it is the real envelope
        # error a user cares about. Per-ruling "dev" is the contact-line residual
        # and is ~0 for a cylinder on an exact ruled surface, so it would mislead
        # as the headline accuracy.
        self.status.showMessage(
            f"cycle {r['cycle_time_s']:.2f}s   surface err "
            f"{r.get('swept_overcut', 0.0)*1000:.1f}µm   clearance "
            f"{r['min_clearance']:.2f}mm  [{coll}]")

    def _on_compare(self, stats):
        # `stats` already holds dev arrays + scalars (computed in the worker);
        # reuse for both charts -- no recompute on the UI thread.
        devmap = {s: stats[s]["dev"] for s in stats}
        self._update_chart("Deviation", charts.deviation_chart, devmap)
        self._update_chart("Compare", charts.compare_chart, stats)

    # ---- drawing ------------------------------------------------------------
    def _dev_surface(self, surf, dev_um, **mesh_kw):
        """Add a structured surface coloured by deviation (µm). `dev_um` may be a
        per-point field or a per-station array (broadcast across v)."""
        nu, nv, _ = surf.shape
        g = pv.StructuredGrid()
        g.points = surf.reshape(-1, 3)
        g.dimensions = (nv, nu, 1)
        d = np.asarray(dev_um)
        g["dev_um"] = (np.repeat(d, nv) if d.size == nu else d.reshape(-1))
        self.plotter.add_mesh(g, scalars="dev_um", cmap="coolwarm", **mesh_kw)
        return g

    def _draw_3d(self, r):
        surf = r["surf"]; nu, nv, _ = surf.shape
        cam = self.plotter.camera_position
        self.plotter.clear()
        # colour by the REAL machined-surface error: swept-envelope overcut depth
        # (max(0,-swept)), which is what the cutter actually removes past the
        # design surface. Far-field clearance is positive and huge, so we show
        # overcut depth, not signed swept distance. Fall back to the per-station
        # residual only if the swept field is unavailable.
        sf = r.get("swept_field")
        if sf is not None:
            field_um = np.maximum(0.0, -sf) * 1000.0
            title = "surface err (µm)"
        else:
            field_um = r["devfield"] * 1000.0
            title = "dev (µm)"
        self._dev_surface(surf, field_um, scalar_bar_args={"title": title})
        self.plotter.add_mesh(pv.lines_from_points(r["strict"]),
                              color="lime", line_width=3)
        q0, al = r["q0"], r["alpha"]
        for i in range(0, nu, max(1, nu // 14)):
            self.plotter.add_mesh(
                pv.Line(q0[i] - al[i]*5.0, q0[i] + al[i]*30.0),
                color="black", line_width=2)
        self._show_tool_at(self.anim_slider.value())
        # re-add any imported CAD overlay (plotter.clear() above wipes it)
        if self._overlay is not None:
            self.plotter.add_mesh(self._overlay, color="lightgray",
                                  opacity=0.4, name="imported_cad")
        if cam is not None:
            self.plotter.camera_position = cam
        else:
            self.plotter.reset_camera()

    # ---- workflow stepper ---------------------------------------------------
    def _show_overview(self):
        self._stage_idx = None
        self.stage_lbl.setText("  (overview)  ")
        if hasattr(self, "guide"):
            self._show_overview_guide()
        if self.last:
            self._draw_3d(self.last)
            self._fill_results(self.last)

    def _step_stage(self, d):
        i = 0 if self._stage_idx is None else self._stage_idx + d
        self._goto_stage(int(np.clip(i, 0, len(workflow.STAGES) - 1)))

    def _goto_stage(self, idx):
        if not self.last:
            return
        self._stage_idx = idx
        scene = workflow.stage_scene(self.last, workflow.STAGE_KEYS[idx],
                                     R=self.model.values["R"])
        self.stage_lbl.setText(f"  {scene['title']}  ")
        self._render_scene(scene)
        self._fill_stage_metrics(scene)
        self._set_stage_guide(scene["key"])
        # (re)draw the analysis chart bound to this stage from the live result,
        # with the current-station cursor, and bring it forward
        chart = workflow.STAGE_CHART.get(scene["key"])
        self._bind_stage_chart(chart, self.anim_slider.value())
        if chart in self.canvases:
            self.tabs.setCurrentWidget(self.canvases[chart])
        # show the cutter on the staged scene so Play sweeps it through this step
        if workflow.STAGE_ANIMATE.get(scene["key"]):
            self._show_tool_at(self.anim_slider.value())

    def _bind_stage_chart(self, chart, station):
        """Redraw the stage's analysis chart from the live result with the
        current-station cursor, so each Preview stage drives its matching chart
        and scrubbing the animation moves the cursor along it."""
        r = self.last
        if not r or chart not in self.canvases:
            return
        mark = int(np.clip(station, 0, r["q0"].shape[0] - 1))
        if chart == "Machinability":
            self._update_chart("Machinability", charts.machinability_chart,
                               r["delta"], r["dev"], mark=mark)
        elif chart == "Deviation":
            self._update_chart("Deviation", charts.deviation_chart,
                               {self.model.strategy: r["dev"]}, mark=mark)
        elif chart == "Kinematics":
            self._update_chart("Kinematics", charts.kinematics_chart,
                               r["machine_path"], mark=mark)
        elif chart == "Feed":
            self._update_chart("Feed", charts.feed_chart,
                               r["seglen"], r["aprof"], mark=mark)
        # Chatter is independent of station; leave its standing render

    def _render_scene(self, scene):
        """Translate a renderer-agnostic workflow scene into PyVista actors."""
        cam = self.plotter.camera_position
        self.plotter.clear()
        for m in scene["meshes"]:
            t = m["type"]
            if t == "surface":
                surf = m["points"]; nu, nv, _ = surf.shape
                g = pv.StructuredGrid()
                g.points = surf.reshape(-1, 3)
                g.dimensions = (nv, nu, 1)
                if m.get("scalar") is not None:
                    g["s"] = np.asarray(m["scalar"]).reshape(-1)
                    self.plotter.add_mesh(g, scalars="s", cmap=m.get("cmap", "coolwarm"),
                                          opacity=m.get("opacity", 1.0),
                                          scalar_bar_args={"title": m.get("title", "")})
                else:
                    self.plotter.add_mesh(g, color=m.get("color", "lightgray"),
                                          opacity=m.get("opacity", 1.0))
            elif t == "polyline":
                self.plotter.add_mesh(pv.lines_from_points(np.asarray(m["points"])),
                                      color=m.get("color", "black"),
                                      line_width=m.get("width", 2))
            elif t == "lines":
                for p0, p1 in m["segments"]:
                    self.plotter.add_mesh(pv.Line(p0, p1),
                                          color=m.get("color", "black"),
                                          line_width=m.get("width", 2))
            elif t == "tube":
                pl = pv.lines_from_points(np.asarray(m["points"]))
                pl["s"] = np.asarray(m["scalar"]).reshape(-1)
                self.plotter.add_mesh(pl.tube(radius=m.get("radius", 1.0)),
                                      scalars="s", cmap=m.get("cmap", "turbo"),
                                      scalar_bar_args={"title": m.get("title", "")})
            elif t == "points":
                self.plotter.add_mesh(pv.PolyData(np.asarray(m["points"])),
                                      color=m.get("color", "red"),
                                      point_size=m.get("size", 8),
                                      render_points_as_spheres=True)
        if self._overlay is not None:
            self.plotter.add_mesh(self._overlay, color="lightgray",
                                  opacity=0.4, name="imported_cad")
        if cam is not None:
            self.plotter.camera_position = cam
        else:
            self.plotter.reset_camera()

    def _fill_stage_metrics(self, scene):
        rows = [("stage", scene["title"])] + list(scene["metrics"])
        self.results_tbl.setRowCount(len(rows))
        for i, (k, v) in enumerate(rows):
            self.results_tbl.setItem(i, 0, QtWidgets.QTableWidgetItem(str(k)))
            self.results_tbl.setItem(i, 1, QtWidgets.QTableWidgetItem(str(v)))

    _TRAIL_N = 8   # number of fading ghost cutters behind the current one
    _SIM_MAX = 30  # max faint cutters drawn for the accumulated swept volume

    def _show_tool_at(self, i):
        """Render the cutter as a solid cylinder at station i (named actor, so
        it is replaced rather than accumulated). When the trail is enabled, a
        few fading ghost cutters behind it show the swept volume of the tool."""
        if not self.last:
            return
        r = self.last
        nu = r["q0"].shape[0]
        i = int(np.clip(i, 0, nu - 1))
        R = self.model.values["R"]
        h = 30.0

        def cutter(j):
            q0, al = r["q0"][j], r["alpha"][j]
            return pv.Cylinder(center=q0 + al * h * 0.5, direction=al,
                               radius=R, height=h, resolution=40)

        # fading ghost trail (swept volume); spaced back along the path
        trail = getattr(self, "_trail", True)
        step = max(1, nu // 40)
        for g in range(self._TRAIL_N):
            name = f"ghost_{g}"
            j = i - (g + 1) * step
            if trail and j >= 0:
                op = 0.22 * (1.0 - g / self._TRAIL_N)
                self.plotter.add_mesh(cutter(j), color="#d4af37",
                                      opacity=max(0.04, op), name=name)
            else:
                try:
                    self.plotter.remove_actor(name)
                except Exception:
                    pass

        self.plotter.add_mesh(cutter(i), color="#d4af37", opacity=0.6, name="tool")
        self.plotter.add_mesh(pv.Sphere(radius=R*0.18, center=r["contact"][i]),
                              color="red", name="contact")
        # in a Preview stage, move the bound chart's station cursor with the scrub
        if self._stage_idx is not None:
            chart = workflow.STAGE_CHART.get(workflow.STAGE_KEYS[self._stage_idx])
            self._bind_stage_chart(chart, i)

        # interactive dexel material-removal simulation: as the slider advances,
        # show the swept tool volume carved so far and the dexel-measured removed
        # volume / % of the final cut.
        if getattr(self, "_sim_mode", False):
            self._sim_render(i, cutter)
        self.plotter.render()

    def _sim_render(self, i, cutter):
        from .. import verify
        r = self.last; nu = r["q0"].shape[0]
        # accumulated swept volume up to station i (faint cylinders, subsampled)
        js = list(range(0, i + 1, max(1, (i // self._SIM_MAX) + 1)))
        for s in range(self._SIM_MAX):
            name = f"swv_{s}"
            if s < len(js):
                self.plotter.add_mesh(cutter(js[s]), color="#6f86c6",
                                      opacity=0.10, name=name)
            else:
                try:
                    self.plotter.remove_actor(name)
                except Exception:
                    pass
        # dexel removed volume with poses 0..i vs the full cut
        R = self.model.values["R"]
        Lf = np.linalg.norm(r["b"] - r["a"], axis=1)
        flat = r["surf"].reshape(-1, 3)
        lo = flat.min(0) - 2*R; hi = flat.max(0) + 2*R
        vol = verify.removed_volume(r["q0"][:i+1], r["alpha"][:i+1], R, Lf[:i+1],
                                    lo, hi, n=48)
        if not hasattr(self, "_sim_full") or self._sim_full <= 0:
            self._sim_full = verify.removed_volume(r["q0"], r["alpha"], R, Lf,
                                                   lo, hi, n=48)
        pct = 100.0 * vol / self._sim_full if self._sim_full > 0 else 0.0
        self.status.showMessage(
            f"material removal — station {i+1}/{nu}: {vol:.0f} mm³ ({pct:.0f}% of cut)")

    def _toggle_sim(self, on):
        self._sim_mode = bool(on)
        self._sim_full = 0.0
        if not on:
            for s in range(self._SIM_MAX):
                try:
                    self.plotter.remove_actor(f"swv_{s}")
                except Exception:
                    pass
        if self.last:
            self._show_tool_at(self.anim_slider.value())

    def _toggle_trail(self, on):
        self._trail = bool(on)
        if self.last:
            self._show_tool_at(self.anim_slider.value())

    def _toggle_play(self):
        if self._anim_timer.isActive():
            self._anim_timer.stop(); self.play_act.setText("▶ Play")
        else:
            self._anim_timer.start(); self.play_act.setText("⏸ Pause")

    def _anim_step(self):
        v = self.anim_slider.value() + 1
        if v > self.anim_slider.maximum():
            v = 0
        self.anim_slider.setValue(v)

    def _fill_results(self, r):
        # primary accuracy = machined-surface (swept-envelope) error; the
        # per-ruling deviation is the contact-line residual (≈0 for a cylinder
        # on an exact ruled surface), shown for diagnostics, not as the headline.
        rows = [
            ("strategy", self.model.strategy),
            ("machined-surface error (swept)",
             f"{r.get('swept_overcut', 0.0)*1000:.1f} µm"),
            ("gouge depth (per station)", f"{r.get('gouge_max', 0.0)*1000:.1f} µm"),
            ("contact-line residual (peak)", f"{r['dev'].max()*1000:.1f} µm"),
            ("contact-line residual (mean)", f"{r['dev'].mean()*1000:.1f} µm"),
            ("orientation jerk", f"{r['orient_jerk']:.3f}"),
            ("cycle time", f"{r['cycle_time_s']:.2f} s"),
            ("path length", f"{r['path_len_mm']:.1f} mm"),
            ("feed cap", f"{r['feed_cap_mm_min']:.0f} mm/min"),
            ("min clearance", f"{r['min_clearance']:.2f} mm"),
            ("assembly clearance", f"{r.get('assembly_clearance', float('nan')):.2f} mm"),
            ("holder clearance", f"{r.get('holder_clearance', float('nan')):.2f} mm"),
            ("hub/shroud clearance",
             ("—" if r.get("hub_clearance", float("inf")) == float("inf")
              else f"{r.get('hub_clearance'):.2f} mm")),
            ("approach / retract clearance",
             f"{r.get('approach_clearance', float('nan')):.2f} / "
             f"{r.get('retract_clearance', float('nan')):.2f} mm"),
            ("blade-index clearance",
             ("—" if r.get("index_clearance", float("inf")) == float("inf")
              else f"{r.get('index_clearance'):.2f} mm")),
            ("structural-link clearance",
             ("—" if r.get("link_clearance", float("inf")) == float("inf")
              else f"{r.get('link_clearance'):.2f} mm")),
            ("fixture/body clearance",
             ("—" if r.get("mesh_clearance", float("inf")) == float("inf")
              else f"{r.get('mesh_clearance'):.2f} mm")),
            ("collision-free", str(r["collision_free"])),
            ("avoidance (rulings tilted)",
             f"{r.get('avoidance_adjusted', 0)}"
             + (f", {len(r['avoidance_infeasible'])} infeasible"
                if r.get('avoidance_infeasible') else "")),
            ("machine", r.get("machine_name", "—")),
            ("reachable", _reach_str(r)),
            ("cut force (peak)", f"{r.get('cut_force_peak_N', 0.0):.0f} N"),
            ("cut power", f"{r.get('cut_power_W', 0.0)/1000.0:.2f} kW"),
            ("feed feasible", str(r.get("feed_feasible", True))),
        ]
        self.results_tbl.setRowCount(len(rows))
        for i, (k, v) in enumerate(rows):
            tip = helpdoc.metric_tip(k)
            mi = QtWidgets.QTableWidgetItem(k); vi = QtWidgets.QTableWidgetItem(v)
            if tip:
                mi.setToolTip(tip); vi.setToolTip(tip)
            self.results_tbl.setItem(i, 0, mi)
            self.results_tbl.setItem(i, 1, vi)

    def _update_chatter(self):
        """Stability lobes from a measured FRF if loaded, else a modal default."""
        from .. import core
        from ..process import ProcessParams
        Kt = ProcessParams().Kt
        nlobes = 6
        if self.model.frf is not None:
            freq, re, im = self.model.frf
            rpm, alim = core.stability_lobes_frf(freq, re, im, Kt, 4, nlobes)
            nptsper = len(freq)
        else:
            nptsper = 80
            rpm, alim = core.stability_lobes(800.0, 0.03, 2.0e4, Kt,
                                             n_teeth=4, nlobes=nlobes, nptsper=nptsper)
        self._update_chart("Chatter", charts.chatter_chart,
                           rpm, alim, nlobes, nptsper, ProcessParams().rpm)

    def load_frf(self):
        from ..process import read_frf_csv
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load tool-tip FRF", "", "CSV (*.csv)")
        if not fn:
            return
        try:
            self.model.frf = read_frf_csv(fn)
        except Exception as e:
            self.status.showMessage(f"FRF load failed: {e}")
            return
        self._update_chatter()
        self.status.showMessage(f"loaded measured FRF: {fn}")

    def _update_chart(self, tab, fn, *args, **kwargs):
        c = self.canvases[tab]
        c.figure.clear()
        fn(*args, fig=c.figure, **kwargs)
        c.draw_idle()

    # ---- file actions -------------------------------------------------------
    def import_rails(self):
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Import rails CSV", "", "CSV (*.csv)")
        if fn:
            self.model.rails = cadio.read_rails_csv(fn)
            self.recompute(compare=True)

    def load_blade_cad(self):
        """Extract ruled hub/shroud rails from a STEP/IGES blade and optimise it."""
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load blade from STEP/IGES", "",
            "B-rep (*.step *.stp *.iges *.igs)")
        if not fn:
            return
        try:
            a, b = cadio.rails_from_cad(fn, nu=int(self.model.values["nu"]))
        except Exception as e:
            self.status.showMessage(f"rail extraction failed: {e}")
            return
        self.model.rails = (a, b)
        self.status.showMessage(f"loaded blade from {fn} ({len(a)} stations)")
        self.recompute(compare=True)

    def load_blisk(self):
        """Extract every blade of a blisk STEP/IGES; show the first."""
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load blisk", "", "B-rep (*.step *.stp *.iges *.igs)")
        if not fn:
            return
        try:
            self._blisk = cadio.rails_list_from_cad(
                fn, nu=int(self.model.values["nu"]))
        except Exception as e:
            self.status.showMessage(f"blisk load failed: {e}")
            return
        self._blisk_i = 0
        self.model.rails = self._blisk[0]
        self.status.showMessage(f"blisk: {len(self._blisk)} blades; showing blade 1")
        self.recompute(compare=True)

    def next_blisk_blade(self):
        if not self._blisk:
            return
        self._blisk_i = (self._blisk_i + 1) % len(self._blisk)
        self.model.rails = self._blisk[self._blisk_i]
        self.status.showMessage(
            f"blisk: blade {self._blisk_i + 1}/{len(self._blisk)}")
        self.recompute(compare=True)

    def use_parametric(self):
        """Drop any loaded CAD blade/overlay/FRF and return to defaults."""
        self.model.rails = None
        self.model.frf = None
        self._overlay = None
        self._blisk = None
        self.recompute(compare=True)

    def show_envelope(self):
        """Render the TRUE swept-envelope surface -- the actual machined geometry
        (design grid projected onto the nearest swept cutter) -- coloured by the
        signed swept error, beside a faint reference of the design surface. The
        deviation from design is exaggerated (adaptively, so the worst error maps
        to ~10% of the blade height) so sub-millimetre overcut/leftover is
        visible without grossly distorting large errors."""
        if not self.last or "envelope_surf" not in self.last:
            return
        self._stage_idx = None
        r = self.last
        design = r["surf"]; env = r["envelope_surf"]
        disp = np.linalg.norm((env - design).reshape(-1, 3), axis=1)
        dmax = float(disp.max())
        height = float(np.linalg.norm(r["b"] - r["a"], axis=1).mean())
        exag_f = 1.0 if dmax < 1e-9 else float(np.clip(0.1 * height / dmax, 1.0, 50.0))
        exag = design + exag_f * (env - design)
        sf = r.get("swept_field")
        cam = self.plotter.camera_position
        self.plotter.clear()
        # faint design reference
        gd = pv.StructuredGrid()
        gd.points = design.reshape(-1, 3)
        gd.dimensions = (design.shape[1], design.shape[0], 1)
        self.plotter.add_mesh(gd, color="lightgray", opacity=0.25, name="design_ref")
        # machined envelope, coloured by signed swept error (um)
        ge = pv.StructuredGrid()
        ge.points = exag.reshape(-1, 3)
        ge.dimensions = (env.shape[1], env.shape[0], 1)
        if sf is not None:
            ge["err_um"] = (sf * 1000.0).reshape(-1)
            self.plotter.add_mesh(ge, scalars="err_um", cmap="coolwarm",
                                  scalar_bar_args={"title": "swept err (µm)"})
        else:
            self.plotter.add_mesh(ge, color="#c0a020")
        self.status.showMessage(
            f"machined swept envelope (displacement x{exag_f:.0f}); "
            f"overcut {r.get('swept_overcut', 0.0)*1000:.0f} µm")
        if cam is not None:
            self.plotter.camera_position = cam
        else:
            self.plotter.reset_camera()

    def minimize_swept_overcut(self):
        """Close the loop on the audit finding: per-ruling deviation is ~0 for a
        cylinder on a ruled surface, so the real error is the swept-envelope
        overcut. This switches to the global strategy and turns on the swept
        penalty (swept_weight) so the optimiser actually reduces that error,
        trading a little per-ruling residual. One click, then recompute."""
        self.model.strategy = "global"
        self.strategy_cb.setCurrentText("global")
        w = 0.5 if self.model.values.get("swept_weight", 0.0) <= 0.0 else \
            self.model.values["swept_weight"]
        self.model.values["swept_weight"] = w
        if "swept_weight" in self._editors:
            self._editors["swept_weight"].setValue(w)   # reflect in the panel
        self.status.showMessage(
            f"minimizing swept overcut (global, swept_weight={w})…")
        self.recompute(compare=True)

    def show_double_flank(self):
        """One-shot double-flank channel view: both walls coloured by deviation
        with the single tool tangent to both."""
        from ..pipeline import double_flank_channel
        r = double_flank_channel(self.model.build_params())
        self.plotter.clear()
        for surf, dev, nm in ((r["surfL"], r["devL"], "L"),
                              (r["surfR"], r["devR"], "R")):
            self._dev_surface(surf, dev * 1000.0, name=f"wall_{nm}")
        q0, al = r["q0"], r["alpha"]
        for i in range(0, q0.shape[0], max(1, q0.shape[0] // 12)):
            cyl = pv.Cylinder(center=q0[i] + al[i]*15, direction=al[i],
                              radius=self.model.values["R"], height=30)
            self.plotter.add_mesh(cyl, color="#d4af37", opacity=0.4,
                                  name=f"tool_{i}")
        self.plotter.reset_camera()
        self._overlay = None
        self.status.showMessage(
            f"double-flank channel: wall-L max {r['devL'].max()*1000:.1f}µm, "
            f"wall-R max {r['devR'].max()*1000:.1f}µm")

    def show_roughing(self):
        """Visualise the layered channel-roughing passes in the 3D view."""
        from ..pipeline import rough_channel
        rg = rough_channel(self.model.build_params())
        self.plotter.clear()
        for poly in rg["passes"][::max(1, len(rg["passes"]) // 60)]:
            self.plotter.add_mesh(pv.lines_from_points(poly), color="#ff7f0e",
                                  line_width=1)
        self.plotter.reset_camera(); self._overlay = None
        self.status.showMessage(
            f"roughing: {rg['n_axial']}×{rg['n_radial']} passes, "
            f"{rg['total_len_mm']:.0f} mm, {rg['cycle_s']:.0f} s, "
            f"vol {rg['removed_volume_mm3']:.0f} mm³")

    def show_trochoidal(self):
        """Visualise the engagement-controlled trochoidal roughing coil."""
        from ..pipeline import rough_channel_trochoidal
        r = rough_channel_trochoidal(self.model.build_params())
        self.plotter.clear()
        self.plotter.add_mesh(pv.lines_from_points(r["points"]),
                              color="#ff7f0e", line_width=1)
        self.plotter.reset_camera(); self._overlay = None
        self.status.showMessage(
            f"trochoidal: {r['n_loops']} loops, engagement {r['engagement_deg']:.0f}°, "
            f"{r['path_len_mm']:.0f} mm, {r['cycle_s']:.0f} s")

    def show_edge_finish(self):
        """Visualise point-mill (ball-nose) rows on the leading-edge patch."""
        from ..pipeline import edge_finish
        ef = edge_finish(self.model.build_params())
        cl = ef["cl"]
        self.plotter.clear()
        for k in range(cl.shape[1]):
            self.plotter.add_mesh(pv.lines_from_points(cl[:, k, :]),
                                  color="cyan", line_width=1)
        self.plotter.reset_camera(); self._overlay = None
        self.status.showMessage(
            f"edge finishing: {ef['n_rows']} rows, "
            f"scallop {ef['scallop']*1000:.1f} µm, path {ef['path_len_mm']:.0f} mm")

    def show_process_plan(self):
        """Full report: stacked finishing + edge finishing + real roughing."""
        from ..pipeline import (stacked_flank_passes, edge_finish, rough_channel)
        p = self.model.build_params()
        st = stacked_flank_passes(p)
        ef = edge_finish(p)
        rg = rough_channel(p)
        QtWidgets.QMessageBox.information(
            self, "Process plan",
            f"Blade height: {st['blade_height']:.1f} mm\n\n"
            f"ROUGHING (layered): {rg['n_axial']}×{rg['n_radial']} passes\n"
            f"  removed volume: {rg['removed_volume_mm3']:.0f} mm³\n"
            f"  cycle: {rg['cycle_s']:.0f} s\n\n"
            f"FLANK FINISH: {st['n_passes']} stacked pass(es)\n"
            f"  peak deviation: {st['dev_max']*1000:.1f} µm\n"
            f"  cycle: {st['cycle_total_s']:.1f} s\n\n"
            f"EDGE FINISH (point-mill): {ef['n_rows']} rows\n"
            f"  scallop: {ef['scallop']*1000:.1f} µm, "
            f"path {ef['path_len_mm']:.0f} mm")

    def show_fillet_machining(self):
        """Recognised-fillet finishing: a ball-nose toolpath rolling along the
        root fillet. Renders the cross-passes and reports the no-gouge margin."""
        from ..pipeline import fillet_machining
        p = self.model.build_params()
        self._run_bg(lambda: fillet_machining(p), self._draw_fillet,
                     busy="fillet finishing (building path)…")

    def _draw_fillet(self, fm):
        import pyvista as pv
        self.plotter.clear()
        if self.last is not None:
            import pyvista as _pv
            surf = self.last["surf"]
            g = _pv.StructuredGrid()
            g.points = surf.reshape(-1, 3)
            g.dimensions = (surf.shape[1], surf.shape[0], 1)
            self.plotter.add_mesh(g, color="lightgray", opacity=0.3)
        C = fm["centers"]
        for k in range(C.shape[0]):
            self.plotter.add_mesh(pv.lines_from_points(np.ascontiguousarray(C[k])),
                                  color="#17becf", line_width=3)
        for k in range(fm["contacts"].shape[0]):
            self.plotter.add_mesh(
                pv.lines_from_points(np.ascontiguousarray(fm["contacts"][k])),
                color="#bcbd22", line_width=2)
        self.plotter.reset_camera()
        self.status.showMessage(
            f"root-fillet finishing: r_fillet {fm['fillet_r']:.1f} mm, "
            f"ball R{fm['r_ball']:.1f}, {fm['n_passes']} passes, "
            f"path {fm['path_len_mm']:.0f} mm — "
            f"{'gouge-free' if fm['gouge_free'] else 'GOUGE'} "
            f"(wall {fm['min_wall_dist_mm']:.2f} mm)")

    def show_rest_machining(self):
        """Stock-aware rest-machining: carry a dexel stock through roughing then
        finishing and report how much the finish removes (the rest material) vs
        finishing the raw stock."""
        from ..pipeline import rest_machining
        p = self.model.build_params()
        self._run_bg(lambda: rest_machining(p), self._report_rest_machining,
                     busy="rest-machining (carving stock)…")

    def _report_rest_machining(self, rm):
        QtWidgets.QMessageBox.information(
            self, "Rest-machining (dexel stock)",
            f"Channel stock: {rm['stock_volume_mm3']:.0f} mm³\n\n"
            f"After roughing: {rm['after_rough_mm3']:.0f} mm³  "
            f"(removed {rm['rough_removed_mm3']:.0f} mm³)\n"
            f"After finishing: {rm['after_finish_mm3']:.0f} mm³  "
            f"(removed {rm['finish_removed_mm3']:.0f} mm³)\n\n"
            f"REST MATERIAL the finish cuts: {rm['finish_removed_mm3']:.0f} mm³\n"
            f"vs finishing RAW stock: {rm['finish_from_raw_mm3']:.0f} mm³\n"
            f"rest fraction: {rm['rest_fraction']*100:.0f}%  "
            f"(roughing pre-cleared the rest)")
        self.status.showMessage(
            f"rest-machining: finish cuts {rm['finish_removed_mm3']:.0f} mm³ "
            f"({rm['rest_fraction']*100:.0f}% of raw)")

    def load_fixture(self):
        """Load a fixture / machine-body triangle mesh (STL/STEP/IGES, in part
        coordinates) for sub-mm tool-assembly collision checking."""
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load fixture / machine body", "",
            "CAD (*.stl *.step *.stp *.iges *.igs)")
        if not fn:
            return
        try:
            v, f = cadio.read_cad(fn)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Fixture load failed", str(e))
            return
        self.model.fixture_mesh = (v, f)
        self.status.showMessage(f"fixture loaded ({len(f)} triangles) — "
                                "checking tool-assembly clearance")
        self.recompute(compare=True)

    def import_cad(self):
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Import CAD", "",
            "CAD (*.stl *.step *.stp *.iges *.igs)")
        if not fn:
            return
        try:
            v, f = cadio.read_cad(fn)
        except Exception as e:        # OCP missing or bad file
            self.status.showMessage(f"CAD import failed: {e}")
            return
        faces = np.hstack([np.full((len(f), 1), 3), f]).ravel()
        # persist so the overlay survives the plotter.clear() in _draw_3d
        self._overlay = pv.PolyData(v, faces)
        self.plotter.add_mesh(self._overlay, color="lightgray",
                              opacity=0.4, name="imported_cad")
        self.plotter.render()

    def export_stl(self):
        if not self.last:
            return
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export blade STL", "blade.stl", "STL (*.stl)")
        if fn:
            v, f = cadio.surface_to_triangles(self.last["surf"])
            cadio.write_stl(fn, v, f)

    def export_rails(self):
        if not self.last:
            return
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export rails CSV", "blade_rails.csv", "CSV (*.csv)")
        if fn:
            cadio.write_rails_csv(fn, self.last["a"], self.last["b"])

    def save_gcode(self):
        if not self.last:
            return
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save G-code", "bladecam.nc", "G-code (*.nc)")
        if fn:
            with open(fn, "w") as fh:
                fh.write(postproc.to_gcode(self.last["machine_path"],
                                           self.last["feed_cap_mm_min"],
                                           move_times=self.last.get("move_times_s")))

    def save_heidenhain(self):
        if not self.last:
            return
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Heidenhain klartext", "bladecam.h", "Heidenhain (*.h)")
        if fn:
            with open(fn, "w") as fh:
                fh.write(postproc.to_heidenhain(
                    self.last["contact"], self.last["alpha"],
                    self.last["feed_cap_mm_min"],
                    move_times=self.last.get("move_times_s")))

    def about(self):
        QtWidgets.QMessageBox.about(
            self, "BladeCAM",
            "BladeCAM — 5-axis flank-milling tool-positioning toolkit\n"
            "Fortran numeric core + PySide6/PyVista GUI.")

    # ---- help / onboarding --------------------------------------------------
    def _doc_dialog(self, title, html):
        dlg = QtWidgets.QDialog(self); dlg.setWindowTitle(title)
        dlg.resize(560, 460)
        v = QtWidgets.QVBoxLayout(dlg)
        br = QtWidgets.QTextBrowser(); br.setHtml(html); v.addWidget(br)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Close)
        bb.rejected.connect(dlg.reject); bb.accepted.connect(dlg.accept)
        v.addWidget(bb)
        dlg.exec()

    def show_getting_started(self):
        html = "<h2>Getting started</h2>" + "".join(
            f"<p>{p}</p>" for p in helpdoc.GETTING_STARTED.split("\n\n"))
        self._doc_dialog("Getting started", html)

    def show_quick_start(self):
        steps = "".join(f"<li>{s}</li>" for s in helpdoc.QUICK_START)
        self._doc_dialog("Quick start", "<h2>Quick start</h2><ol>" + steps + "</ol>")

    def show_glossary(self):
        rows = "".join(f"<p><b>{t}</b> — {d}</p>"
                       for t, d in helpdoc.GLOSSARY.items())
        self._doc_dialog("Glossary", "<h2>Glossary</h2>" + rows)

    def _maybe_welcome(self):
        """First-run welcome that offers the getting-started guide."""
        from PySide6 import QtCore as _qc
        s = _qc.QSettings("BladeCAM", "BladeCAM")
        if s.value("seen_welcome", False, type=bool):
            return
        s.setValue("seen_welcome", True)
        m = QtWidgets.QMessageBox(self)
        m.setWindowTitle("Welcome to BladeCAM")
        m.setText("BladeCAM positions a 5-axis flank-milling cutter on impeller "
                  "blades and posts the G-code.")
        m.setInformativeText("Open the getting-started guide? You can always reach "
                             "it from Help ▸ Getting started, and hover any control "
                             "for inline help.")
        m.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        m.setDefaultButton(QtWidgets.QMessageBox.Yes)
        if m.exec() == QtWidgets.QMessageBox.Yes:
            self.show_getting_started()

    def _act(self, menu, text, slot, shortcut=None):
        a = QtGui.QAction(text, self)
        a.triggered.connect(slot)
        if shortcut:
            a.setShortcut(shortcut)
        menu.addAction(a)
        return a


def apply_dark_theme(app):
    """Apply a professional dark Fusion palette."""
    app.setStyle("Fusion")
    pal = QtGui.QPalette()
    c = QtGui.QColor
    pal.setColor(QtGui.QPalette.Window, c(43, 43, 43))
    pal.setColor(QtGui.QPalette.WindowText, c(220, 220, 220))
    pal.setColor(QtGui.QPalette.Base, c(30, 30, 30))
    pal.setColor(QtGui.QPalette.AlternateBase, c(45, 45, 45))
    pal.setColor(QtGui.QPalette.Text, c(220, 220, 220))
    pal.setColor(QtGui.QPalette.Button, c(53, 53, 53))
    pal.setColor(QtGui.QPalette.ButtonText, c(220, 220, 220))
    pal.setColor(QtGui.QPalette.Highlight, c(38, 110, 183))
    pal.setColor(QtGui.QPalette.HighlightedText, c(255, 255, 255))
    app.setPalette(pal)


def main():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    apply_dark_theme(app)
    win = MainWindow()
    win.resize(1480, 900)
    win.show()
    app.exec()


if __name__ == "__main__":
    main()
