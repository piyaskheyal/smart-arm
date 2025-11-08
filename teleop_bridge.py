import serial
import json
import subprocess
import sys
import time
import os
import numpy as np

from calibration import calibrate_arm

while True:
    print('''
        Teleoperation [1]
        Calibration   [2]
        ''')

    mode = input("Select mode (1/2): ").strip()
    if mode == '1':
        print("Teleoperation mode selected.")
        break
    elif mode == '2':
        print("Calibration mode selected.")
        calibrate_arm()
    else:
        print("Invalid mode selected. Exiting.")
        exit(1)


# --- Configuration ---
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 9600
NUM_POTS = 6
CALIB_FILE = "calib.json"
CONTROL_ARM_SCRIPT = "control_arm.py"

# Rate Limiting: Max speed in degrees per second for each joint.
# Tune these values to balance responsiveness and smoothness.
MAX_CHANGE_PER_SECOND = np.array([
    180.0,  # Joint 1 (Shoulder Pan)
    120.0,  # Joint 2 (Shoulder Lift)
    180.0,  # Joint 3 (Elbow Flex)
    200.0,  # Joint 4 (Wrist Flex)
    200.0,  # Joint 5 (Wrist Roll)
    80.0,   # Joint 6 (Gripper)
])

# Define the output angle range (in degrees) for each joint.
JOINT_ANGLE_RANGES = [
    [-110, 110],  # Joint 1
    [-100, 100],    # Joint 2
    [-96, 96],  # Joint 3
    [-95, 95],    # Joint 4
    [-157, 157],  # Joint 5
    [-10, 100],      # Joint 6
]

def load_calibration(filename):
    """Loads potentiometer calibration data from a JSON file."""
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Error: Calibration file not found at '{filename}'")
        print("Please run the calibration script first.")
        sys.exit(1)
    except json.JSONDecodeError:
        print(f"Error: Could not parse the calibration file '{filename}'.")
        sys.exit(1)

def map_value(value, from_min, from_max, to_min, to_max):
    """Maps a value from one range to another (linear interpolation)."""
    if from_max == from_min:
        return to_min
    value = max(from_min, min(value, from_max))
    from_span = from_max - from_min
    to_span = to_max - to_min
    value_scaled = float(value - from_min) / float(from_span)
    return to_min + (value_scaled * to_span)

def main():
    """
    Main function to run the teleoperation bridge.
    - Connects to the Arduino.
    - Spawns and pipes data to the control_arm.py script.
    """
    try:
        import serial
    except ImportError:
        # ... (rest of the dependency check is unchanged)
        pass

    calib_data = load_calibration(CALIB_FILE)
    pot_calib = calib_data.get("potentiometers", [])
    if len(pot_calib) != NUM_POTS:
        print(f"Error: Calibration file contains data for {len(pot_calib)} pots, expected {NUM_POTS}.")
        sys.exit(1)

    print("Calibration data loaded successfully.")

    print(f"Starting '{CONTROL_ARM_SCRIPT}' in teleoperation mode...")
    try:
        control_process = subprocess.Popen(
            [sys.executable, "-u", CONTROL_ARM_SCRIPT],
            stdin=subprocess.PIPE,
            stdout=sys.stdout,
            stderr=sys.stderr,
            text=True,
            bufsize=1
        )
        time.sleep(3)
        control_process.stdin.write("1\n")
        control_process.stdin.flush()
        print(f"'{CONTROL_ARM_SCRIPT}' started. Waiting for initial output...")
        time.sleep(5)
    except Exception as e:
        print(f"Failed to start '{CONTROL_ARM_SCRIPT}': {e}")
        sys.exit(1)

    # --- Rate Limiting and Serial Communication ---
    current_values = np.zeros(NUM_POTS)
    is_first_reading = True
    last_update_time = time.monotonic()

    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
            print(f"Successfully opened serial port {SERIAL_PORT}.")
            print("Forwarding potentiometer data to simulation. Press Ctrl+C to exit.")
            
            while True:
                if control_process.poll() is not None:
                    print(f"'{CONTROL_ARM_SCRIPT}' has terminated unexpectedly.")
                    break

                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8').strip()
                        if not line: continue

                        raw_values = [int(v) for v in line.split(',')]
                        if len(raw_values) != NUM_POTS: continue

                        # --- Map raw values to desired degrees ---
                        desired_values = np.zeros(NUM_POTS)
                        for i in range(NUM_POTS):
                            pot_info = pot_calib[i]
                            min_angle, max_angle = JOINT_ANGLE_RANGES[i]
                            desired_values[i] = map_value(raw_values[i], pot_info['min'], pot_info['max'], min_angle, max_angle)

                        # --- Initialize or apply Rate Limiting ---
                        current_time = time.monotonic()
                        delta_t = current_time - last_update_time
                        last_update_time = current_time

                        if is_first_reading:
                            current_values = desired_values
                            is_first_reading = False
                        else:
                            # Calculate max allowed change for this time step
                            max_change = MAX_CHANGE_PER_SECOND * delta_t
                            
                            # Calculate the difference between desired and current
                            change = desired_values - current_values
                            
                            # Clamp the change to the max allowed
                            clamped_change = np.clip(change, -max_change, max_change)
                            
                            # Apply the clamped change
                            current_values += clamped_change

                        # --- Format and send to simulation ---
                        output_str = " ".join(f"{v:.2f}" for v in current_values)
                        print(f"Sending to sim: {output_str}", end='\r')
                        
                        control_process.stdin.write(output_str + "\n")
                        control_process.stdin.flush()

                    except (ValueError, IndexError, UnicodeDecodeError):
                        continue # Ignore malformed lines
                    except IOError as e:
                        print(f"\nIOError communicating with subprocess: {e}")
                        break
                
                time.sleep(0.01) # Loop rate, ~100Hz

    except serial.SerialException as e:
        print(f"\nError: Could not open serial port {SERIAL_PORT}. Details: {e}")
    except KeyboardInterrupt:
        print("\nShutdown requested.")
    finally:
        print("Closing subprocess and exiting.")
        if 'control_process' in locals() and control_process.poll() is None:
            control_process.stdin.write("quit\n")
            control_process.stdin.flush()
            control_process.terminate()
            control_process.wait(timeout=5)
        print("Bridge script finished.")

if __name__ == "__main__":
    main()
