# Mirrorcle DEMO-06 (EaZy4.0) 3D Scanning Trajectory Research

Research prototype for 3D laser line scanning and trajectory comparison using Mirrorcle's DEMO-06 development kit architecture, Basler camera capture, and MEMS beam steering.

## Kit Identification
This repository is aligned to the official Mirrorcle user-guide naming for:
- Product: 3D Scanning Demonstrator Kit
- Part Number: DEMO-06
- Scan Module Family: EaZy4.0
- MEMS Mirror: A7M10.2-1000AL
- Controller: USB-SL MZ

Based on the DEMO-06 user guide, typical reference hardware includes a USB 3.0 camera and calibrated triangulation geometry.

## Variant Note (Important)
Mirrorcle documentation for DEMO-06 (May 2025 revision) references an EaZy4.0V violet-laser configuration.
This research code may also be used with other EaZy4.0 variants (including EaZy4.0G green-laser configurations) depending on your lab hardware.
Always verify wavelength, laser power, and calibration files for your specific unit.

## Project Summary
This repository contains the trajectory-comparison implementation used in research experiments for:
- Raster scanning
- Sinusoidal Lissajous scanning
- Triangular Lissajous scanning
- Bidirectional Cartesian scanning
- Radial Lissajous scanning

The main scanner application builds 3D points through camera-laser triangulation and exports summary and comparison data for trajectory studies.

The implementation extends SDK demo ideas with custom trajectory experiments and 3D analysis exports.

## Main Entry Point
- src/MTICamera-3DScan-Demo.cpp

## Repository Structure
- src/
  - Main scanner and trajectory logic
- include/mticamera/
  - Camera abstraction and Pylon camera wrappers
- include/mtidevice/
  - Device control, serial communications, and data generation
- config/
  - Runtime configuration templates
- results/
  - Sample research outputs and summaries
- figures/
  - Place screenshots/plots for GitHub presentation

## Hardware Used
- Mirrorcle DEMO-06 class hardware stack (EaZy4.0 scan module + controller + camera)
- Mirrorcle MEMS controller and driver chain
- Basler camera
- Laser source integrated in scan module

## Software Dependencies
This repository does not bundle proprietary vendor SDK binaries.
Install these locally before building:
- Mirrorcle Software Suite and C++ SDK
- Basler Pylon SDK
- OpenCV (Windows)
- Visual Studio with C++ desktop tools

## Build Notes (Windows + Visual Studio)
1. Create or open your Visual Studio C++ project.
2. Add include paths for:
   - include/mticamera
   - include/mtidevice
   - Local vendor SDK include directories (Mirrorcle, Pylon, OpenCV)
3. Add library paths and linker inputs for your installed vendor SDKs.
4. Copy config files from config/ into the runtime working directory.
5. Build for x64 Release (recommended for acquisition performance).

## Run Notes
1. Connect DEMO-06 hardware chain (or equivalent EaZy4.0 setup) and camera.
2. Confirm serial device connectivity and camera visibility.
3. Launch the executable and select trajectory mode from the UI controls.
4. Export CSV summaries from test runs for comparison analysis.

Recommended from guide context:
- Use low to medium ambient lighting for initial operation.
- Keep target geometry near calibrated working distance and recalibrate if setup changes.

## Included Sample Outputs
- all_scans_comparison.csv
- NP_comparison_study.csv
- camera_pattern_2d.csv
- Multiple trajectory summary text reports

## Reproducibility and Scope
This repository is a research codebase and may require local calibration and device-specific tuning before use in a different setup.

## Third-Party Notice
Third-party SDKs are required but not redistributed here. See THIRD_PARTY_NOTICE.md.

## Citation and CV Use
If this project is referenced in a CV, include:
- Repository link
- Brief statement of your direct technical contributions
- Key quantitative outcomes from results/
