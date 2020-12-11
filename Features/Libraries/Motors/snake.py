from motors import *
from imu import *
import time
import serial
import pretty_errors
import threading

vector_list = []
encoder1 = 0
encoder2 = 0

class PID:
    """PID Controller
    """

    def __init__(self, P=5.5 , I=1, D=1, current_time=None):

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


def rightpid():
    global angle 
    rangle = angle
    goala = rangle + 90
    pid = PID()
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
    pid = PID()
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
 

def zoom(ticks):
    global encoder1
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<ticks):
        setspeedm1(4095)
        setspeedm2(4095)
        forward()
    stop()
    print(encoder1)


def return_base():
    global angle
    global vector_list

    rangle = angle
    
    res_mag, res_deg = calc_resultant(vector_list)
    if (res_deg - 180) < 0:
        res_deg = (res_deg - 180) % 360
    else:
        res_deg = (res_deg -180)
        
    pid = PID()
    pid.SetPoint = res_deg
    pid.setSampleTime(1)
    while True:
        pid.update(angle)
        targetPWM = pid.output
        #targetPWM = max(min(int(targetPWM), 1300),800)
        print("Goal Angle: "+str(res_deg))
        print("Current Angle: "+str(angle))
        print("Target PWM: "+str(targetPWM))
        print("\n")
        speed = round(targetPWM + 500)
        print(speed)
        setspeedm1(speed)
        setspeedm2(speed)
        left()
        if abs(res_deg - angle) < 1.5:
            stop()
            break
 

def create_vector():
    global encoder1, encoder2
    global angle
    global vector_list

    enc_avg = round((encoder1 + encoder2) / 2)
    vector_list.append(enc_avg, angle)
    enc_res()
    time.sleep(.5)


def snake():    
    fwrd()
    create_vector()
    rightpid()   
    time.sleep(1)
    fwrd_short()
    create_vector() 
    rightpid()
    time.sleep(1)
    fwrd()
    create_vector() 
    leftpid()
    sleep(1)
    fwrd_short()
    create_vector()    
    leftpid()
    sleep(1)
    fwrd()
    create_vector()
    print(vector_list)
    return_base()


def encoder_thread():
    global encoder1
    global encoder2
    elist = []
    while True:
        while serial_e != 0:
            data = serial_e.readline()
            decoded_bytes = data[0:len(data)-2].decode("utf-8")
            if data:
                elist.append(int(decoded_bytes))
            if len(elist) == 2:
                encoder1 = elist[0]
                encoder2 = elist[1]
            if len(elist) == 2:
                elist = []



def enc_res():
    serial_e.write(b"0")
    print("Reset")


def fwrd():
    global encoder1
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<10000):
        setspeedm1(1400)
        setspeedm2(1400)
        forward()
        print(encoder1)
    stop()
    print(encoder1)


def fwrd_short():
    global encoder1
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<6943):
        setspeedm1(1400)
        setspeedm2(1400)
        forward()
    stop()
    print(encoder1)

#Threaded Serial Readers
enc_th = threading.Thread(target=encoder_thread)
enc_th.start()
