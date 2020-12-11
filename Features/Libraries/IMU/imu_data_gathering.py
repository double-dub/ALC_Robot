from array import *
import adafruit_bno055
import board
import busio
import math
import time


# Use these lines for I2C
i2c = busio.I2C(board.SCL, board.SDA)

#creates a sensor object to make easier calls to the BNO055 class
sensor = adafruit_bno055.BNO055_I2C(i2c)

# give the BNO055 some time to power on completely
time.sleep(10)

run = True

angle_vals = array('f', [])

while run:
    print()
    
    a = sensor.euler
    (direction, y_dir, z_dir) = a
    
    try:
        init_value = float(direction)
        angle_vals.append(math.radians(init_value))
        print(init_value)
    except Exception:
        init_value = init_value
        angle_vals.append(math.radians(init_value))
        print("The data is invalid.")
        
    if len(angle_vals) == 10:
        run = False

    time.sleep(0.005)
    
print(angle_vals)

sin_sum = 0
cos_sum = 0
for i in angle_vals:
    sin_sum += math.sin(i)
    cos_sum += math.cos(i)

avg_angle = math.atan(sin_sum / cos_sum)
print("Average angle is: " , math.degrees(avg_angle))
