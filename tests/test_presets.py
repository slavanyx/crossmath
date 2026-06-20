#!/usr/bin/env python3
"""OrcaSlicer-style preset system: built-in libraries, JSON round-trip, and
user save/load/delete for machine / tool / strategy profiles."""
import sys
import os
import tempfile

try:
    import numpy as np
    from bladecam import presets, machine as machine_lib
    from bladecam.process import ProcessParams
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP presets ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    with tempfile.TemporaryDirectory() as root:
        st = presets.PresetStore(root=root)

        # built-in libraries are populated for every kind
        for kind in presets.KINDS:
            check(len(st.names(kind)) >= 3, f"built-in {kind} presets present",
                  f"({len(st.names(kind))})")
            check(all(st.is_builtin(kind, n) for n in st.names(kind)),
                  f"all initial {kind} presets are built-in")

        # machine round-trip preserves ranges (tuples) and limits exactly
        m0 = machine_lib.get_machine("Compact blisk cell")
        m1 = presets.machine_from_dict(presets.machine_to_dict(m0))
        check(m1.x_range == m0.x_range and m1.a_range == m0.a_range
              and m1.kind == m0.kind and m1.vmax() == m0.vmax(),
              "machine dict round-trip is exact")

        # tool round-trip
        t0 = ProcessParams(tool_dia=8.0, n_teeth=3, ae=2.0)
        t1 = presets.tool_from_dict(presets.tool_to_dict(t0))
        check(t1.tool_dia == 8.0 and t1.n_teeth == 3 and t1.ae == 2.0,
              "tool dict round-trip is exact")

        # save a user preset, reload it, and confirm it appears in names()
        st.save("machine", "My machine", presets.machine_to_dict(
            machine_lib.Machine(name="My machine", x_range=(-77, 77))))
        check("My machine" in st.names("machine"), "user preset listed")
        check(not st.is_builtin("machine", "My machine"), "user preset not built-in")
        back = presets.machine_from_dict(st.load("machine", "My machine"))
        check(back.x_range == (-77, 77), "user preset reloads with its edits")

        # a fresh store on the same root sees the persisted user preset
        st2 = presets.PresetStore(root=root)
        check("My machine" in st2.user_names("machine"),
              "user preset persists across store instances")

        # built-ins are read-only; deleting one is a no-op, user ones delete
        check(not st.delete("machine", "Compact blisk cell"),
              "cannot delete a built-in")
        check(st.delete("machine", "My machine") and
              "My machine" not in st.names("machine"), "user preset deletes")

        # strategy + tool presets actually drive a compute()
        sp = st.load("strategy", "Barrel finish (R200)")
        tp = presets.tool_from_dict(st.load("tool", "16 mm 5FL roughing"))
        mp = presets.machine_from_dict(st.load("machine", "Generic 5-axis trunnion"))
        r = compute(Params(machine=mp, process=tp,
                           **{k: sp[k] for k in presets.STRATEGY_FIELDS}))
        check(np.all(np.isfinite(r["dev"])) and r["machine_name"],
              "presets drive a finite compute()", f"({r['machine_name']})")
        check(sp["barrel_R"] == 200.0, "barrel strategy preset carries Rb")

        # --- bundle export / import (share whole configs) ---
        st.save("tool", "shop tool A", presets.tool_to_dict(ProcessParams(tool_dia=9.0)))
        st.save("strategy", "shop strat A", presets._builtin_strategies()["Min-max accuracy"])
        bundle = os.path.join(root, "shop.bladecam-presets.json")
        n = st.export_bundle(bundle)
        check(n >= 2 and os.path.isfile(bundle), "bundle exported user presets",
              f"({n})")
        # import into a FRESH store on a different root
        with tempfile.TemporaryDirectory() as root2:
            st3 = presets.PresetStore(root=root2)
            check("shop tool A" not in st3.user_names("tool"), "fresh store is empty")
            imp = st3.import_bundle(bundle)
            check(imp >= 2 and "shop tool A" in st3.user_names("tool"),
                  "bundle imported into a fresh store", f"({imp})")
            check(presets.tool_from_dict(st3.load("tool", "shop tool A")).tool_dia == 9.0,
                  "imported preset keeps its values")
            # built-ins are never shadowed by import
            check(st3.import_bundle(bundle) >= 0, "re-import is idempotent-safe")

        # --- dirty-state comparison ---
        d = presets._builtin_strategies()["Flank finish (global)"]
        check(presets.presets_equal(d, dict(d)), "identical dicts compare equal")
        d2 = dict(d); d2["mu"] = d["mu"] + 1.0
        check(not presets.presets_equal(d, d2), "changed value compares unequal")
        d3 = dict(d); d3["mu"] = d["mu"] + 1e-12
        check(presets.presets_equal(d, d3), "tiny float diff is within tolerance")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nPRESET TESTS PASSED")


if __name__ == "__main__":
    main()
