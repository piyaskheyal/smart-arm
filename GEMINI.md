# Gemini Workspace Context: Smart Arm v2

This document provides a comprehensive overview of the "Smart Arm v2" robotics project for the Gemini AI assistant.

## Project Overview

The goal of this project is to train a "Smart Arm v2" robotic arm in a simulated environment. The technology stack for this project includes:
*   **OS:** Ubuntu 24.04 LTS
*   **Simulation:** NVIDIA Isaac Sim 5.1.0
*   **Language:** Python

The first step towards training the arm is to collect a dataset of movements. The current focus is on setting up a teleoperation system to control the arm in the simulation and record this data.

The project is currently structured around two main functionalities:
1.  **Calibration**: A Python script (`calibration.py`) is used to calibrate the arm's six potentiometers by reading their minimum and maximum values over a serial connection and saving them to `calib.json`.
2.  **Teleoperation**: The main script (`control_arm.py`) loads a URDF model of the arm into Isaac Sim and allows for real-time joint control via terminal input. It features a smooth motion controller to interpolate between joint position commands.

The robot model is provided in various formats (URDF, USD, MJCF) within the `Models/` directory.

## Key Files

*   `control_arm.py`: The main script for controlling the robot arm in Isaac Sim. It offers two modes: teleoperation and calibration.
*   `calibration.py`: A script to calibrate the arm's potentiometers. It requires a serial connection to the arm's hardware.
*   `test_load.py`: A simple utility script to load the arm model into Isaac Sim to verify that it loads correctly.
*   `calib.json`: The output of the calibration process, containing the min and max values for each of the 6 potentiometers.
*   `Models/`: This directory contains all the 3D models and simulation files for the robot arm.
    *   `armv1.usd`: The main USD file used by the Isaac Sim scripts.
    *   `SO101/`: Contains the URDF and MuJoCo models for the "SO101" version of the arm, including different calibration versions.
    *   `SO101/README.md`: Provides details on the different calibration methods for the SO101 model.

## Building and Running

This project has dependencies on NVIDIA Isaac Sim and the `pyserial` Python library.

### Calibration

To run the arm calibration process:

1.  Connect the robot arm via USB.
2.  Ensure the `SERIAL_PORT` in `calibration.py` is set to the correct device.
3.  Run the script:
    ```bash
    python3 calibration.py
    ```
4.  Move each joint to its mechanical limits to record the full range of motion.
5.  Press Enter to save the `calib.json` file.

### Running the Simulation

To control the arm in the Isaac Sim environment:

1.  Make sure you are in an environment where Isaac Sim is available.
2.  Run the main control script:
    ```bash
    python3 control_arm.py
    ```
3.  Select "Teleoperation" mode (1).
4.  Enter 6 floating-point values (in degrees) in the terminal to set the target position for the arm's joints: `shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper`.

## Development Conventions

*   The simulation is built using the `isaacsim` Python API.
*   The main control loop in `control_arm.py` uses a queue to handle user input asynchronously.
*   Motion control uses a smoothstep function to create fluid movement between setpoints.
*   The project relies on a mix of URDF and USD files for the robot model, with `armv1.usd` being the primary file for the simulation scripts.
