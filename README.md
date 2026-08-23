# Flight Dynamics and Aerospace GNC

A MATLAB-first, Khan-Academy-style learning track with 24 guided modules.

Each implemented module combines:

- a concise lesson and physical mental model;
- MATLAB `%%` notebook cells;
- deterministic plots;
- actual UI sliders, spinners, or dropdowns;
- two parameter sweeps;
- one deliberately broken case;
- executable numerical checks;
- a tutor protocol that asks one observation question at a time.

## Start

From a shell:

```bash
./bin/learn start
./bin/learn start P01
./bin/learn start P02
./bin/learn start P03
./bin/learn start P04
./bin/learn list
./bin/learn status
```

On Windows PowerShell:

```powershell
python .\bin\learn.py start
```

In MATLAB:

```matlab
launch_lesson("P01")
launch_lesson("P02")
launch_lesson("P03")
launch_lesson("P04")
run_module_checks("P01")
run_module_checks("P02")
run_module_checks("P03")
run_module_checks("P04")
```

`P01` remains the reference implementation; `P02`, `P03`, and `P04` are the implemented frame-transform, atmosphere-model, and point-mass force-trim lessons. P04 carries P03 density and true airspeed into visible lift, drag, and thrust balances. Implemented modules always form a contiguous prefix; `curriculum/modules.json` is authoritative as later governed batches advance that frontier. Scaffolded modules remain intentionally non-runnable until their own bounded batch is complete.

## Module layout

```text
modules/01-example/
├── README.md
├── lesson.m
├── model.m
├── experiment.m
├── interactive.m
├── lesson.md
├── walkthrough.md
├── checks.md
└── run_checks.m
```

## Learning contract

The flow is always:

> question → mental model → baseline → manipulate levers → observe plots → break an assumption → explain → check → teach back

This repository is compatible with the same tutor/build split used by `dsp-radar_learning`.
