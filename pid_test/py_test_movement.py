import time

# Default values
DEFAULT_RPM = 1000
MAX_RPM = 2000
MIN_RPM = 500
RPM_STEP = 50  # Increment or decrement per iteration

# Initialize motor speeds
motor_speeds = {
    "motor1": DEFAULT_RPM,
    "motor2": DEFAULT_RPM,
    "motor3": DEFAULT_RPM,
    "motor4": DEFAULT_RPM,
}

def adjust_motor_speeds(command):
    """
    Adjust motor speeds incrementally based on the user command.
    """
    global motor_speeds

    if command == "left":
        # Tilt left: decrease left motors, increase right motors
        motor_speeds["motor1"] = max(MIN_RPM, motor_speeds["motor1"] - RPM_STEP)
        motor_speeds["motor3"] = max(MIN_RPM, motor_speeds["motor3"] - RPM_STEP)
        motor_speeds["motor2"] = min(MAX_RPM, motor_speeds["motor2"] + RPM_STEP)
        motor_speeds["motor4"] = min(MAX_RPM, motor_speeds["motor4"] + RPM_STEP)

    elif command == "right":
        # Tilt right: decrease right motors, increase left motors
        motor_speeds["motor1"] = min(MAX_RPM, motor_speeds["motor1"] + RPM_STEP)
        motor_speeds["motor3"] = min(MAX_RPM, motor_speeds["motor3"] + RPM_STEP)
        motor_speeds["motor2"] = max(MIN_RPM, motor_speeds["motor2"] - RPM_STEP)
        motor_speeds["motor4"] = max(MIN_RPM, motor_speeds["motor4"] - RPM_STEP)

    elif command == "forward":
        # Tilt forward: decrease front motors, increase back motors
        motor_speeds["motor1"] = max(MIN_RPM, motor_speeds["motor1"] - RPM_STEP)
        motor_speeds["motor2"] = max(MIN_RPM, motor_speeds["motor2"] - RPM_STEP)
        motor_speeds["motor3"] = min(MAX_RPM, motor_speeds["motor3"] + RPM_STEP)
        motor_speeds["motor4"] = min(MAX_RPM, motor_speeds["motor4"] + RPM_STEP)

    elif command == "back":
        # Tilt backward: decrease back motors, increase front motors
        motor_speeds["motor1"] = min(MAX_RPM, motor_speeds["motor1"] + RPM_STEP)
        motor_speeds["motor2"] = min(MAX_RPM, motor_speeds["motor2"] + RPM_STEP)
        motor_speeds["motor3"] = max(MIN_RPM, motor_speeds["motor3"] - RPM_STEP)
        motor_speeds["motor4"] = max(MIN_RPM, motor_speeds["motor4"] - RPM_STEP)
        
    elif command == "up":
        # Ascend, increase all motors
        motor_speeds["motor1"] = min(MAX_RPM, motor_speeds["motor1"] + RPM_STEP)
        motor_speeds["motor2"] = min(MAX_RPM, motor_speeds["motor2"] + RPM_STEP)
        motor_speeds["motor3"] = max(MIN_RPM, motor_speeds["motor3"] + RPM_STEP)
        motor_speeds["motor4"] = max(MIN_RPM, motor_speeds["motor4"] + RPM_STEP)
    
    elif command == "down":
        # Descend, decrease all motors
        motor_speeds["motor1"] = min(MAX_RPM, motor_speeds["motor1"] - RPM_STEP)
        motor_speeds["motor2"] = min(MAX_RPM, motor_speeds["motor2"] - RPM_STEP)
        motor_speeds["motor3"] = max(MIN_RPM, motor_speeds["motor3"] - RPM_STEP)
        motor_speeds["motor4"] = max(MIN_RPM, motor_speeds["motor4"] - RPM_STEP)

    elif command == "stay":
        # Reset all motors to default RPM
        for motor in motor_speeds:
            motor_speeds[motor] = DEFAULT_RPM

def display_motor_speeds():
    """
    Display the current motor speeds.
    """
    print("\nCurrent motor speeds:")
    for motor, speed in motor_speeds.items():
        print(f"  {motor}: {speed:.2f} rpm")
    print("-" * 30)

# Main loop
print("Quadcopter Motor Speed Controller")
print("Commands: left, right, forward, back, stay, exit")

current_command = "stay"  # Default command
while True:
    # Display motor speeds
    display_motor_speeds()

    # Check for user input to change command
    user_input = input("Enter command (leave blank to continue current action): ").strip().lower()
    if user_input == "exit":
        print("Exiting program.")
        break
    elif user_input in ["left", "right", "forward", "back", "stay", "up", "down"]:
        current_command = user_input

    # Adjust motor speeds based on the current command
    adjust_motor_speeds(current_command)

    # Small delay for smoother monitoring
    time.sleep(0.2)
