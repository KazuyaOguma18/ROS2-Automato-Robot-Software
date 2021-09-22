import numpy as np
import math

def division(color_image, width_color, height_color, Division_number):
    div = int(math.sqrt(Division_number))
    color_div = [[''for i in range(div)] for j in range(div) ]
    y = 0
    x = 0
    for i in range(div):
        x = 0
        for j in range(div):
            color_div[j][i] = color_image[int(y) : int(y + height_color/div), int(x) : int(x + width_color/div)]
            x += width_color/div

        y += height_color/div

    color_image = color_div

    return color_image