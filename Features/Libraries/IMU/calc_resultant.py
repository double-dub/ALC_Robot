import math

vector_list = [
    [1.0, 0.0],
    [1.2, 45.0],
    [1.5, 135]
]


def calc_resultant(array2d):
    x_component = 0.0
    y_component = 0.0
    index = 0

    for _ in array2d:
        x_component += (array2d[index][0] * math.cos(math.radians(array2d[index][1])))
        y_component += (array2d[index][0] * math.sin(math.radians(array2d[index][1])))
        index += 1

    num = math.pow(x_component, 2) + math.pow(y_component, 2)
    resultant_mag = math.sqrt(num)
    resultant_deg = math.atan(y_component/x_component)

    return [round(resultant_mag, 3), round(math.degrees(resultant_deg), 3)]


res_vector = calc_resultant(vector_list)
print(res_vector)
