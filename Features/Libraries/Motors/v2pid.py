from imu import *
from motors import *
import threading
import pretty_errors
import time

class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback

        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time










angle = 0.0

def imu_thread():
    global angle

    while True:
        data = ser.readline()
        decoded_bytes = data.decode("utf-8")
        if data:
            try:
                angle = float(decoded_bytes)
            except Exception:
                pass

P = 5.5
I = 1
D = 1

def rightpid():
    global angle
    rangle = angle
    goala = rangle + 90
    pid = PID(P, I, D)
    pid.SetPoint = goala
    pid.setSampleTime(1)
    while True:
        pid.update(angle)
        targetPWM = pid.output
        #targetPWM = max(min(int(targetPWM), 1300),800)
        print("Goal Angle: "+str(goala))
        print("Current Angle: "+str(angle))
        print("Target PWM: "+str(targetPWM))
        print("\n")
        speed = round(targetPWM + 500)
        print(speed)
        setspeedm1(speed)
        setspeedm2(speed)
        right()
        if (goala - angle) < 1.5:
            stop()
            break        


def leftpid():
    global angle
    rangle = angle
    goala = rangle - 90
    pid = PID(P, I, D)
    pid.SetPoint = goala
    pid.setSampleTime(1)
    while True:
        pid.update(angle)
        targetPWM = pid.output
        #targetPWM = max(min(int(targetPWM), 1300),800)
        print("Goal Angle: "+str(goala))
        print("Current Angle: "+str(angle))
        print("Target PWM: "+str(targetPWM))
        print("\n")
        speed = round(targetPWM + 500)
        print(speed)
        setspeedm1(speed)
        setspeedm2(speed)
        left()
        if (goala - angle) > 1.5:
            stop()
            break 

# Start IMU Threaded Readings
imu_th = threading.Thread(target=imu_thread)
imu_th.start()

while True:
    val = input()
    
    if val == '1':
        print("Current Angle: " + str(angle) + "\n")
    elif val == '2':
        rightpid()
