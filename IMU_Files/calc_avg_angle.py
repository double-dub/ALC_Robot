from array import *
import math

# Sample array to compare to standard mean
angle_values1 = array('f', [math.radians(0.0), math.radians(30.0),
                            math.radians(60.0), math.radians(90.0)])
# Sample array to compare with 360-deg values
angle_values2 = array('f', [math.radians(0.0), math.radians(270.0),
                            math.radians(300.0), math.radians(330.0)])

sin_sum = 0
cos_sum = 0

for i in angle_values2:
    sin_sum += math.sin(i)
    cos_sum += math.cos(i)

avg_angle = math.degrees(math.atan(sin_sum / cos_sum))

if avg_angle < 0:
    avg_angle += 360

print("Average angle is: ", avg_angle)
