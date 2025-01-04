import time

# Function to simulate sending a speed signal to the motor
def set_motor_speed(speed):
    # For demonstration, we print the speed, but in real cases,
    # you would send this value to the motor controller
    print(f"Motor Speed: {speed:.2f} RPM")

# Initialize PID controller constants
Kp = 1.2  # Proportional gain
Ki = 0.01  # Integral gain
Kd = 0.01  # Derivative gain

target_speed = 100  # Target speed in RPM
current_speed = 0  # Initial speed (RPM)

# Initialize PID variables
integral = 0
previous_error = 0

# Run the PID loop for 10 iterations
for i in range(10):
    # Calculate error
    error = target_speed - current_speed

    # Proportional term
    P = Kp * error

    # Integral term
    integral += error
    I = Ki * integral

    # Derivative term
    derivative = error - previous_error
    D = Kd * derivative

    # Total PID output
    pid_output = P + I + D

    # Update the motor speed (this simulates how the motor responds to the control signal)
    current_speed += pid_output

    # Clamp the motor speed to reasonable limits
    if current_speed < 0:
        current_speed = 0
    elif current_speed > 200:  # Assume max speed is 200 RPM
        current_speed = 200

    # Set the motor speed
    set_motor_speed(current_speed)

    # Update previous error for the next loop iteration
    previous_error = error

    # Simulate control loop timing (e.g., every 100 ms)
    time.sleep(0.1)
