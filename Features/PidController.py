from imu import *
from motors import *
import threading
import pretty_errors

kP = 1.0
kI = 0.5
kD = 0.025
set_point = 0.0
total_error = 0.0
prev_error = 0.0
result = 0.0
error = 0.0

def calculate(m_input):
    global kP, kI, kD
    global set_point
    global total_error, prev_error, error
    global result
    sign = True

    # Calculate the error signal
    error = set_point - m_input

    # Integrate the errors as long as the upcoming integrator does
    # not exceed the minimum and maximum output thresholds.
    if 1300 > abs(total_error + error) * kI > 800:
        total_error = total_error + error

    # Perform the PID calculation
    result = (kP * error) + (kI * total_error) + (kD * (error - prev_error))

    # Save the current error to the previous error for the next cycle.
    prev_error = error

    # Record sign of result
    if result < 0:
        sign = False

    # Make sure the final result is within bounds. If we constrain the result, we make
    # sure the sign of the constrained result matched the original result sign.
    if abs(result) > 1300:
        result = 1300
        if sign:
            # turn back
    elif abs(result) < 800:
        result = 800
        if sign:
            # turn back


def pid_reset():
    global total_error, prev_error, result
    total_error = 0.0
    prev_error = 0.0
    result = 0.0

target = False
def on_target():
    global target, error
    if error < abs(0.01 * (current_angle - goal_ang)):
        target = True


def imu_thread():
    global angle

    while True:
        data = ser.readline()
        decoded_bytes = data.decode("utf-8")
        if data:
            angle = float(decoded_bytes)


thread_imu = threading.Thread(target=imu_thread)
thread_imu.start()

while True:
    val = input()
    if val == '1':
        pid_reset()
        current_angle = angle
        goal_ang = current_angle + 90
        set_point = goal_ang

        while True:
            calculate(angle)
            setspeedm1(result)
            setspeedm2(result)
            left()
            if not on_target():
                stop()
                break

        print("Final angle is: " + str(angle))
    elif val == '2':
        break