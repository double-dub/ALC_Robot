import math


# Calculates the resultant vector
# Takes in a list (v_list) where each element is a vector.
# The vector is a tuple with  magnitude (distance in ticks)
# and direction (angle in degrees).
# Returns the vector that points in the direction of home base.
def calc_resultant(v_list):
    x_component = 0.0
    y_component = 0.0
    index = 0

    # Traverse every element in the vector list
    for _ in v_list:
        # Summation of each x-component in the vector
        x_component += (v_list[index][0] * math.cos(math.radians(v_list[index][1])))
        
        # Summation of each y-component in the vector
        y_component += (v_list[index][0] * math.sin(math.radians(v_list[index][1])))
        
        index += 1

    # Square root of sum of squares using x- and y-components
    num = math.pow(x_component, 2) + math.pow(y_component, 2)
    resultant_mag = math.sqrt(num)

    # Calculate the angle  between the components to find direction
    resultant_deg = math.atan(y_component/x_component)

    return [round(resultant_mag, 3), round(math.degrees(resultant_deg), 3)]

