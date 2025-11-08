# Gemini Workspace Context: Smart Arm v2

This document provides a comprehensive overview of the "Smart Arm v2" robotics project for the Gemini AI assistant.

## Project Overview

The goal of this project is to train a "Smart Arm v2" robotic arm in a simulated environment. The technology stack for this project includes:
*   **OS:** Ubuntu 24.04 LTS
*   **Simulation:** NVIDIA Isaac Sim 5.1.0
*   **Language:** Python

The first step towards training the arm is to collect a dataset of movements. The current focus is on setting up a teleoperation system to control the arm in the simulation and record this data.

The project is currently structured around three main functionalities:
1.  **Calibration**: A Python script (`calibration.py`) is used to calibrate the arm's six potentiometers by reading their minimum and maximum values over a serial connection and saving them to `calib.json`.
2.  **Manual Teleoperation**: The main script (`control_arm.py`) loads a URDF model of the arm into Isaac Sim and allows for real-time joint control via terminal input.
3.  **Hardware Teleoperation**: A bridge script (`teleop_bridge.py`) reads data from an Arduino connected to potentiometers, filters it for smooth control, and pipes it to the main simulation script.

The robot model is provided in various formats (URDF, USD, MJCF) within the `Models/` directory.

## Key Files

*   `control_arm.py`: The main script for controlling the robot arm in Isaac Sim. It can be run directly for manual input or as a subprocess by the teleop bridge.
*   `teleop_bridge.py`: A script that connects to an Arduino, reads potentiometer values, maps them to joint angles, and sends them to `control_arm.py` for real-time hardware control.
*   `calibration.py`: A script to calibrate the arm's potentiometers. It requires a serial connection to the arm's hardware.
*   `test_load.py`: A simple utility script to load the arm model into Isaac Sim to verify that it loads correctly.
*   `calib.json`: The output of the calibration process, containing the min and max values for each of the 6 potentiometers.
*   `Models/`: This directory contains all the 3D models and simulation files for the robot arm.
    *   `armv1.usd`: The main USD file used by the Isaac Sim scripts.

## Building and Running

This project has dependencies on NVIDIA Isaac Sim and the `pyserial` Python library.

### 1. Calibration

To run the arm calibration process:

1.  Connect the robot arm hardware via USB.
2.  Ensure the `SERIAL_PORT` in `calibration.py` is set to the correct device.
3.  Run the script:
    ```bash
    python3 calibration.py
    ```
4.  Move each joint to its mechanical limits to record the full range of motion.
5.  Press Enter to save the `calib.json` file.

### 2. Hardware Teleoperation (Recommended)

This method uses potentiometers connected to an Arduino for real-time control.

1.  Ensure calibration has been completed and `calib.json` exists.
2.  Connect your Arduino device via USB.
3.  Verify that the `SERIAL_PORT` in `teleop_bridge.py` is set to the correct device.
4.  Make sure you are in an environment where Isaac Sim is available.
5.  Run the bridge script. It will automatically launch the simulation.
    ```bash
    python3 teleop_bridge.py
    ```
6.  Move the potentiometers to control the arm in the simulation. Press `Ctrl+C` to exit.

### 3. Manual Teleoperation (Terminal Input)

This method allows you to type in joint angles directly.

1.  Make sure you are in an environment where Isaac Sim is available.
2.  Run the main control script:
    ```bash
    python3 control_arm.py
    ```
3.  Select "Teleoperation" mode (1).
4.  Enter 6 floating-point values (in degrees) in the terminal to set the target position for the arm's joints: `shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper`.

## Development Conventions

*   The simulation is built using the `isaacsim` Python API.
*   The `teleop_bridge.py` script is the primary entry point for hardware-based control. It spawns and communicates with `control_arm.py` via standard input.
*   **Input Smoothing**: To achieve smooth, low-latency motion, `teleop_bridge.py` uses a **rate-limiting filter**. This limits the maximum velocity of each joint, preventing jerky movements from noisy potentiometer readings without introducing significant delay. The speeds can be tuned in the `MAX_CHANGE_PER_SECOND` array.
*   The `control_arm.py` script uses a smoothstep function to create fluid movement between setpoints when receiving commands.
*   The project relies on a mix of URDF and USD files for the robot model, with `armv1.usd` being the primary file for the simulation scripts.
