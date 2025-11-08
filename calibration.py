import serial
import json
import threading
import sys
import os
import time

# --- Configuration ---
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 9600
NUM_POTS = 6
OUTPUT_FILE = "calib.json"

# --- Globals for threading ---
stop_thread = False
latest_values = [0] * NUM_POTS
min_vals = [float('inf')] * NUM_POTS
max_vals = [float('-inf')] * NUM_POTS

def listen_for_enter():
    """Waits for the user to press Enter and then sets the stop_thread flag."""
    global stop_thread
    input("Calibration running. Press [Enter] to stop and save...\n")
    stop_thread = True

def print_table():
    """Clears the screen and prints the calibration table."""
    # ANSI escape code to clear screen and move cursor to top-left
    os.system('cls' if os.name == 'nt' else 'clear')
    print("--- ARM CALIBRATION ---")
    print("Move each joint to its minimum and maximum positions.")
    print("Press [Enter] when you are finished.\n")
    
    header = f"{'Potentiometer':<15} | {'Current':>10} | {'Min':>10} | {'Max':>10}"
    print(header)
    print("-" * len(header))
    
    for i in range(NUM_POTS):
        pot_str = f"Pot {i+1}"
        current_val = latest_values[i] if latest_values[i] is not None else "N/A"
        min_val = min_vals[i] if min_vals[i] != float('inf') else "N/A"
        max_val = max_vals[i] if max_vals[i] != float('-inf') else "N/A"
        print(f"{pot_str:<15} | {current_val:>10} | {min_val:>10} | {max_val:>10}")
    
    print("\n" + "-" * len(header))

def calibrate_arm():
    """
    Reads potentiometer values from the serial bus, tracks min/max,
    and saves the calibration data to a JSON file upon completion.
    """
    global latest_values, min_vals, max_vals

    # Start a non-blocking thread to listen for the Enter key
    input_thread = threading.Thread(target=listen_for_enter, daemon=True)
    input_thread.start()

    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
            print(f"Successfully opened serial port {SERIAL_PORT}. Starting calibration...")
            time.sleep(2) # Wait for the serial connection to initialize

            while not stop_thread:
                if ser.in_waiting > 0:
                    try:
                        # Read a line and decode it
                        line = ser.readline().decode('utf-8').strip()
                        if not line:
                            continue

                        # Parse the comma-separated values
                        values = [int(v) for v in line.split(',')]
                        
                        if len(values) == NUM_POTS:
                            latest_values = values
                            # Update min and max values
                            for i in range(NUM_POTS):
                                if values[i] < min_vals[i]:
                                    min_vals[i] = values[i]
                                if values[i] > max_vals[i]:
                                    max_vals[i] = values[i]
                            
                            # Update the display
                            print_table()
                        else:
                            print(f"Warning: Received {len(values)} values, expected {NUM_POTS}. Line: '{line}'")

                    except (ValueError, IndexError) as e:
                        print(f"Warning: Could not parse line: '{line}'. Error: {e}")
                    except UnicodeDecodeError:
                        print(f"Warning: UnicodeDecodeError on line from serial.")
                
                time.sleep(0.001) # Small delay to prevent high CPU usage

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}.")
        print(f"Details: {e}")
        print("Please ensure the device is connected and you have the correct permissions.")
        return

    # --- Save calibration data ---
    if any(v != float('inf') for v in min_vals):
        calibration_data = {
            "potentiometers": [
                {"id": i, "min": min_vals[i], "max": max_vals[i]}
                for i in range(NUM_POTS)
            ]
        }
        with open(OUTPUT_FILE, 'w') as f:
            json.dump(calibration_data, f, indent=4)
        print(f"\nCalibration finished. Data saved to '{OUTPUT_FILE}'.")
    else:
        print("\nCalibration stopped. No data received, so nothing was saved.")

if __name__ == "__main__":
    # Before running, check for pyserial and offer to install it
    try:
        import serial
    except ImportError:
        print("The 'pyserial' library is required but not installed.")
        if input("Do you want to install it now? (y/n): ").lower() == 'y':
            import subprocess
            subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])
            print("'pyserial' installed successfully. Please run the script again.")
        else:
            print("Please install 'pyserial' manually by running: pip install pyserial")
        sys.exit(1)
        
    calibrate_arm()
