# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an industrial robotic inspection and marking system for helmet quality control. A Techman collaborative robot arm equipped with a ZED stereo camera autonomously detects and physically marks defects on protective helmets. The system uses spherical coordinate navigation around the dome-shaped workpiece.

## Running the System

**No build step required** — pure Python.

Primary execution is via Jupyter notebooks:
```bash
jupyter notebook PHASE2_inspection_marking_together.ipynb   # Full pipeline
jupyter notebook PHASE1_opening.ipynb                       # Visor open/close only
jupyter notebook "Helmet positioning.ipynb"                 # Calibration
```

Standalone module runs (useful for testing without hardware):
```bash
python spherical_movement.py     # Renders the radius surface model
python kinematics_v2.py          # Smoke-tests coordinate transforms
```

**Hardware prerequisites before running notebooks:**
- Robot TCP at `IP_ROBOT = "192.168.1.3"` (port 5890), configured in [Variables.py](Variables.py) and [inspection_and_marking.py](inspection_and_marking.py)
- ZED stereo camera connected via USB 3.0

## Dependencies

No requirements.txt — install manually:
```
numpy, opencv-python, matplotlib, pyzed (ZED SDK), pymodbus
```
`tm_libraries/` (Techman robot SDK) is bundled in the repo.

## Architecture

### Module Layers

```
CONFIGURATION
  Variables.py               — calibration poses, helmet center, tool offsets

BASE LAYER
  robot_control.py           — Techman arm via Modbus TCP; blocking move commands
  kinematics_v2.py           — homogeneous transforms, spherical↔Cartesian
  camera_scripts_v2.py       — ZED SDK, HSV thresholding, point cloud extraction

MIDDLE LAYER
  spherical_movement.py      — safe arc planning with variable radius
  defects_id_wrapper.py      — pixel→camera→global transforms, filtering, dedup

ORCHESTRATION
  inspection_and_marking.py  — full workflows: inspect → refine → mark

EXECUTION
  PHASE2_inspection_marking_together.ipynb  — primary entry point
```

### Key Design Concepts

**Spherical coordinate navigation** — The robot navigates around the helmet using `[r, alpha, beta]` spherical coordinates rather than raw Cartesian. `spherical_movement.py` computes variable-radius arcs that follow the helmet contour and auto-splits arcs > 90° to avoid singularities.

**Chained homogeneous transforms** — Defect positions flow through a matrix chain: `image pixel → 3D camera frame → end-effector frame → global frame`. This chain is assembled in `defects_id_wrapper.py` using rotation matrices from `kinematics_v2.py`.

**Three-level defect filtering** in `defects_id_wrapper.py`:
1. Cylindrical filter in camera space (removes out-of-range depths)
2. Positional filter in global space (removes out-of-bounds coordinates)
3. Duplicate removal (merge detections within 10–25 mm threshold)

**Inspection → refinement → marking pipeline** in `inspection_and_marking.py`:
1. Global scan at predefined positions, detect green HSV regions
2. Close-up refinement shots per defect, average depth readings to reduce ZED noise
3. Safety-validate spherical coordinates, plan arc to marking position
4. Advance marker linearly, retract

### Configuration

All calibration data lives in [Variables.py](Variables.py): end-effector poses, helmet center offset from robot base, camera-to-EE and marker-to-EE rigid offsets, and inspection waypoints. Change hardware layout → update `Variables.py` first.

HSV color thresholds for defect detection are in [camera_scripts_v2.py](camera_scripts_v2.py) (`lower_green` / `upper_green`). Area and depth filter thresholds are also in that file.
