#!/usr/bin/env python3
"""Help-system completeness: every parameter editor, workflow stage and results
metric the GUI shows must have learn-as-you-go help text, so a user can hover or
click anything and learn it. This is the usability analogue of the
config-completeness invariant."""
import sys

try:
    from bladecam.gui import help as H
    from bladecam.gui.model import PARAM_SPEC, TOOL_SPEC, MACHINE_SPEC
    from bladecam import workflow
except ImportError as e:
    print(f"SKIP help ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    # 1) every parameter editor has a non-trivial tooltip
    keys = [s[0] for s in PARAM_SPEC + TOOL_SPEC + MACHINE_SPEC]
    missing = [k for k in keys if len(H.param_tip(k)) < 8]
    check(not missing, "every parameter editor has help text", f"(missing {missing})")

    # 2) every workflow stage has guidance ending in a 'next step' pointer
    smiss = [k for k in workflow.STAGE_KEYS if len(H.stage_help(k)) < 8]
    check(not smiss, "every workflow stage has guidance", f"(missing {smiss})")

    # 3) the results metrics the GUI renders are all explained (kept in sync with
    #    _fill_results in gui/main.py)
    metrics = [
        "strategy", "machined-surface error (swept)", "gouge depth (per station)",
        "contact-line residual (peak)", "contact-line residual (mean)",
        "orientation jerk", "cycle time", "path length", "feed cap",
        "min clearance", "assembly clearance", "holder clearance",
        "structural-link clearance", "fixture/body clearance", "collision-free",
        "machine", "reachable", "cut force (peak)", "cut power", "feed feasible",
    ]
    mmiss = [m for m in metrics if len(H.metric_tip(m)) < 8]
    check(not mmiss, "every results metric is explained", f"(missing {mmiss})")

    # 4) the glossary covers the core domain terms a newcomer will hit
    need = ["Flank milling", "Ruled surface", "Distribution parameter δ",
            "Swept envelope / overcut", "TOPP", "TCPM", "Rest machining",
            "Barrel tool", "Reachability"]
    gmiss = [t for t in need if t not in H.GLOSSARY]
    check(not gmiss, "glossary covers the core terms", f"(missing {gmiss})")
    check(all(len(v) > 20 for v in H.GLOSSARY.values()), "glossary entries are real")

    # 5) onboarding content present
    check(len(H.QUICK_START) >= 4 and all(len(s) > 20 for s in H.QUICK_START),
          "quick-start steps present")
    check(len(H.GETTING_STARTED) > 200, "getting-started text present")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nHELP TESTS PASSED")


if __name__ == "__main__":
    main()
