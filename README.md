# Malletwand: the Pendulum as a Handheld Interface to Musical Timing

**Malletwand** is a handheld controller device in the form of a pendulum (a handle with a swinging bob), which can act as an input interface to the timing of musical playback.

We present this at [NIME](https://nime.org/) 2024 as a short paper ([PDF](paper/paper.pdf); video to-be-updated) in paper session 10, on Friday, 6 Sep.

![Banner image: the white balls of the controllers swinging in the air, in front of a prototype self-playing glockenspiel](paper/Mw_Banner.jpg)

## Contents

The following directories contain all the files for design and implementation (hardware, firmware, and mechanical).
- **wand/**: the handheld controller (the "Wand").
  - Hardware is a single PCB; firmware for the two controllers are in subdirectories **fw/** (the main one) and **fw_bob/** (the small one in the bob/ball) respectively.
- **mallet/**: one unit of the self-playing (robotic) glockenspiel instrument (the "Mallet").
- **mallet_central/**: the central unit for the instrument, mostly managing communications (both wireless and wired).

To use the files, the following software packages are required (versions for reference):
- Hardware: KiCad (7.0.10)
- Firmware: Arm GNU Toolchain (GCC 10.3.1), STM32Cube ('G0 1.5.0, 'L0 1.12.1), PlatformIO (6.1.15)
- Mechanical: FreeCAD (0.21.1), OpenSCAD (2021.01)

The following directories contain miscellaneous resources:
- **numerical/**: Numerical experiments for the filtering algorithms (EKF and ALS), implemented in Julia.
- **paper/**: The paper's LaTeX source and images.
- **testdrive/**: A testbed prototype for the sensors, created in the early days of development. Includes both hardware and firmware, and a desktop client to receive the BLE signals. Currently not well documented, but might be relevant for inspection (and visualisation?).
- **releases/**: Gerber files submitted to the PCB fabrication house.

## Licence
All content in this repository is distributed under CERN-OHL-S (CERN Open Hardware Licence Version 2, Strongly Reciprocal). See full text at **COPYING.txt** or [online](https://ohwr.org/cern_ohl_s_v2.txt).
