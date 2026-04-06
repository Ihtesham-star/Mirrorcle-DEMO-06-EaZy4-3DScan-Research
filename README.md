# DEMO-06 3D Scan Trajectory Experiments

C++ research code for trajectory-driven 3D scanning using Mirrorcle DEMO-06 class hardware (EaZy4.0 family), a Basler camera, and laser-line triangulation.

This is the exact codebase used for trajectory comparison work that I reference in my CV.

## What This Repo Contains

- A working MTICamera-3DScan demo extension with custom scan trajectories
- Real experiment outputs (CSV and summary files)
- Camera and device configuration files used during experiments

Main source file:
- src/MTICamera-3DScan-Demo.cpp

## Implemented Scan Trajectories

- Raster
- Sinusoidal Lissajous
- Triangular Lissajous
- Bidirectional Cartesian
- Radial Lissajous

## Hardware Context

- Mirrorcle DEMO-06 / EaZy4.0 family setup
- MEMS mirror: A7M10.2-1000AL
- USB-SL MZ controller
- USB 3.0 Basler camera

Note:
- Mirrorcle documentation commonly references DEMO-06 with EaZy4.0V configuration.
- This workflow also applies to other EaZy4.0 variants used in lab setups (for example green-laser builds), with matching calibration and settings.

## Repository Layout

```text
src/                    Main scanner implementation
include/mticamera/      Camera API wrappers
include/mtidevice/      Device control and data generation APIs
config/                 Runtime .ini files
results/                Exported experiment outputs
figures/                Add screenshots/plots for GitHub page
```

## Dependencies (Not Bundled)

Install locally before building:

- Mirrorcle Software Suite / C++ SDK
- Basler Pylon SDK
- OpenCV (Windows)
- Visual Studio with C++ Desktop Development

Vendor SDK binaries are intentionally not committed to this repository.

## Build (Windows)

1. Open or create a Visual Studio C++ project.
2. Add include paths:
   - include/mticamera
   - include/mtidevice
   - your local Mirrorcle, Pylon, and OpenCV include folders
3. Add linker library paths and required vendor libs.
4. Copy files from config/ to the executable working directory.
5. Build x64 Release.

## Run

1. Connect MEMS controller, scan module, and camera.
2. Confirm COM and camera detection.
3. Run the app and choose trajectory mode.
4. Export scan data to CSV for analysis.

Practical conditions for stable first runs:
- low/medium ambient light
- target near calibration distance

## Included Results

Examples already included in results/:

- all_scans_comparison.csv
- NP_comparison_study.csv
- camera_pattern_2d.csv
- trajectory summary text files

## Scope

This is research code, not a production package. Recalibration and parameter tuning are expected when hardware geometry changes.

## Third-Party and Licensing

- Repository license: MIT (see LICENSE)
- Third-party usage notes: see THIRD_PARTY_NOTICE.md
- This project is not an official Mirrorcle repository
