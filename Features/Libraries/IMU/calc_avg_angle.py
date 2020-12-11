from array import *
import math

# Sample array to compare to standard mean
angle_values1 = array('f', [0.0, 30.0, 60.0, 90.0])

# Sample array to compare with 360-deg values
angle_values2 = array('f', [0.0, 270.0, 300.0, 330.0])


# This function takes in a list of angles. Converts angles to radians
# in order to use trig functions.
# Parameter: a list of float values, angles should be in degrees
# Return: mean of circular quantities in degrees
def calc_avg_angle(float_arr):
    for x in range(len(float_arr)):
        float_arr[x] = math.radians(float_arr[x])

    sin_sum = 0
    cos_sum = 0

    for i in float_arr:
        sin_sum += math.sin(i)
        cos_sum += math.cos(i)

    avg_angle = math.degrees(math.atan(sin_sum / cos_sum))

    # Negative angles are converted to their positive equivalent
    if avg_angle < 0:
        avg_angle += 360

    return avg_angle


print("Average angle is: ", calc_avg_angle(angle_values1))
print("Average angle is: ", calc_avg_angle(angle_values2))
