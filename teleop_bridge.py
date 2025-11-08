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
# ------------------------------------------------------------


# --- Configuration ---
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 9600
NUM_POTS = 6
CALIB_FILE = "calib.json"
CONTROL_ARM_SCRIPT = "control_arm.py"
SMOOTHING_FACTOR = 0.1  # Adjust this between 0 (no smoothing) and 1 (max smoothing). 0.1-0.3 is a good range.

# Define the output angle range (in degrees) for each joint.
# You can customize these ranges based on your robot's mechanical limits.
# Format: [min_angle, max_angle]
JOINT_ANGLE_RANGES = [
    [-180, 180],  # Joint 1 (Shoulder Pan)
    [-90, 90],    # Joint 2 (Shoulder Lift)
    [-150, 150],  # Joint 3 (Elbow Flex)
    [-90, 90],    # Joint 4 (Wrist Flex)
    [-180, 180],  # Joint 5 (Wrist Roll)
    [0, 40],      # Joint 6 (Gripper) - Assuming 0 is open, 40 is closed
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
    # Avoid division by zero
    if from_max == from_min:
        return to_min
    
    # Clamp the input value to the source range
    value = max(from_min, min(value, from_max))
    
    # Map the value
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
    # Check for pyserial dependency
    try:
        import serial
    except ImportError:
        print("The 'pyserial' library is required but not installed.")
        if input("Do you want to install it now? (y/n): ").lower() == 'y':
            subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
            print("'pyserial' installed successfully. Please run the script again.")
        else:
            print("Please install 'pyserial' manually by running: pip install pyserial")
        sys.exit(1)

    # Load calibration data
    calib_data = load_calibration(CALIB_FILE)
    pot_calib = calib_data.get("potentiometers", [])
    if len(pot_calib) != NUM_POTS:
        print(f"Error: Calibration file contains data for {len(pot_calib)} potentiometers, but {NUM_POTS} are expected.")
        sys.exit(1)

    print("Calibration data loaded successfully.")

    # --- Start the control_arm.py script as a subprocess ---
    print(f"Starting '{CONTROL_ARM_SCRIPT}' in teleoperation mode...")
    try:
        # We use unbuffered stdout/stdin and send a '1' to select teleop mode
        control_process = subprocess.Popen(
            [sys.executable, "-u", CONTROL_ARM_SCRIPT],
            stdin=subprocess.PIPE,
            stdout=sys.stdout, # Show simulation output directly
            stderr=sys.stderr,
            text=True,
            bufsize=1
        )
        time.sleep(3) # Give the simulation time to start up
        control_process.stdin.write("1\n")
        control_process.stdin.flush()
        print(f"'{CONTROL_ARM_SCRIPT}' started. Waiting for initial output...")
        time.sleep(5) # Wait for Isaac Sim to initialize
        
    except FileNotFoundError:
        print(f"Error: The script '{CONTROL_ARM_SCRIPT}' was not found.")
        sys.exit(1)
    except Exception as e:
        print(f"Failed to start '{CONTROL_ARM_SCRIPT}': {e}")
        sys.exit(1)

    # --- Connect to Arduino and start data forwarding ---
    smoothed_values = np.zeros(NUM_POTS)
    is_first_reading = True

    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
            print(f"Successfully opened serial port {SERIAL_PORT}.")
            print("Forwarding potentiometer data to simulation. Press Ctrl+C to exit.")
            
            while True:
                # Check if the subprocess is still running
                if control_process.poll() is not None:
                    print(f"'{CONTROL_ARM_SCRIPT}' has terminated unexpectedly.")
                    break

                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8').strip()
                        if not line:
                            continue

                        raw_values = [int(v) for v in line.split(',')]
                        if len(raw_values) != NUM_POTS:
                            print(f"Warning: Received {len(raw_values)} values, expected {NUM_POTS}.")
                            continue

                        # Map raw values to degrees
                        degree_values = np.zeros(NUM_POTS)
                        for i in range(NUM_POTS):
                            pot_info = pot_calib[i]
                            raw_val = raw_values[i]
                            min_calib = pot_info['min']
                            max_calib = pot_info['max']
                            min_angle, max_angle = JOINT_ANGLE_RANGES[i]
                            
                            mapped_val = map_value(raw_val, min_calib, max_calib, min_angle, max_angle)
                            degree_values[i] = mapped_val

                        # On the first reading, initialize smoothed values to the current values
                        if is_first_reading:
                            smoothed_values = degree_values
                            is_first_reading = False
                        else:
                            # Apply exponential moving average filter
                            smoothed_values = (SMOOTHING_FACTOR * degree_values) + ((1 - SMOOTHING_FACTOR) * smoothed_values)

                        # Format for control_arm.py and send
                        output_str = " ".join(f"{v:.2f}" for v in smoothed_values)
                        print(f"Sending to sim: {output_str}", end='\r')
                        
                        control_process.stdin.write(output_str + "\n")
                        control_process.stdin.flush()

                    except (ValueError, IndexError) as e:
                        print(f"\nWarning: Could not parse line: '{line}'. Error: {e}")
                    except UnicodeDecodeError:
                        print(f"\nWarning: UnicodeDecodeError from serial.")
                    except IOError as e:
                        print(f"\nIOError communicating with subprocess: {e}")
                        break
                
                time.sleep(0.02) # Limit update rate to ~50Hz

    except serial.SerialException as e:
        print(f"\nError: Could not open serial port {SERIAL_PORT}.")
        print(f"Details: {e}")
    except KeyboardInterrupt:
        print("\nShutdown requested.")
    finally:
        print("Closing subprocess and exiting.")
        if 'control_process' in locals() and control_process.poll() is None:
            control_process.stdin.write("quit\n")
            control_process.stdin.flush()
            control_process.terminate() # Forcefully terminate if it doesn't close
            control_process.wait(timeout=5)
        print("Bridge script finished.")

if __name__ == "__main__":
    main()
