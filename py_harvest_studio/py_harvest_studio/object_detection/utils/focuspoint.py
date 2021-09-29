import cv2

def ajust_FocusPoint(k, boxes, focuspoint, scale):
    #center = 0, leftup = 1, leftdown = 2, rightup = 3, rightdown = 4
    pixel_x = boxes[k][1] + (boxes[k][3]-boxes[k][1])/2
    pixel_y = boxes[k][0] + (boxes[k][2]-boxes[k][0])/2

    if focuspoint == 0:
        ajust_pixel_x_center = (pixel_x - 0.5) *2 / scale
        ajust_pixel_y_center = (pixel_y - 0.5) *2 / scale
        ajust_pixel_x_left = (boxes[k][1] - 0.5) *2 / scale
        ajust_pixel_x_right = (boxes[k][3] - 0.5) *2/ scale

    elif focuspoint == 1:
        ajust_pixel_x_center = (pixel_x / scale -0.5) *2
        ajust_pixel_y_center = (pixel_y / scale -0.5) *2
        ajust_pixel_x_left = (boxes[k][1] / scale -0.5) *2
        ajust_pixel_x_right = (boxes[k][3] / scale -0.5) *2

    elif focuspoint == 2:
        ajust_pixel_x_center = (pixel_x / scale -0.5) *2
        ajust_pixel_y_center = ((1 - pixel_y) / scale - 0.5)*2
        ajust_pixel_x_left = (boxes[k][1] / scale -0.5) *2
        ajust_pixel_x_right = (boxes[k][3] / scale -0.5) *2

    elif focuspoint == 3:
        ajust_pixel_x_center = ((1 - pixel_x) / scale - 0.5) *2
        ajust_pixel_y_center = (pixel_y / scale -0.5) *2
        ajust_pixel_x_left = ((1 - boxes[k][1]) / scale - 0.5) *2
        ajust_pixel_x_right = ((1 - boxes[k][3]) / scale - 0.5) *2

    elif focuspoint == 4:
        ajust_pixel_x_center = ((1 - pixel_x) / scale - 0.5) *2
        ajust_pixel_y_center = ((1 - pixel_y) / scale - 0.5)*2
        ajust_pixel_x_left = ((1 - boxes[k][1]) / scale - 0.5) *2
        ajust_pixel_x_right = ((1 - boxes[k][3]) / scale - 0.5) *2

    return ajust_pixel_x_center, ajust_pixel_y_center, ajust_pixel_x_left, ajust_pixel_x_right

def calculate_FocusPoint(k, boxes, X_c, Y_c, scale):
    pixel_x = boxes[k][1] + (boxes[k][3]-boxes[k][1])/2
    pixel_y = boxes[k][0] + (boxes[k][2]-boxes[k][0])/2

    cal_X = (pixel_x/scale - 0.5)*2 + (X_c + 1)*(scale - 1)/scale
    cal_Y = (pixel_y/scale - 0.5)*2 + (Y_c + 1)*(scale - 1)/scale
    cal_X_left = (boxes[k][1]/scale - 0.5)*2 + (Y_c + 1)*(scale - 1)/scale
    cal_X_right = (boxes[k][3] /scale - 0.5)*2 + (Y_c + 1)*(scale - 1)/scale
    cal_Y_up = (boxes[k][2] /scale - 0.5)*2 + (X_c + 1)*(scale - 1)/scale
    cal_Y_low = (boxes[k][4] /scale - 0.5)*2 + (X_c + 1)*(scale - 1)/scale

    return cal_X, cal_Y, cal_X_left, cal_X_right, cal_Y_up, cal_Y_low


def calculate_image(image1, value, X_c, Y_c):
    height, width = image1.shape[:2]
    scale = 1 + value
    img_resize1 = cv2.resize(image1, dsize=None, fx=scale, fy=scale).copy()
    reh, rew = img_resize1.shape[:2]
    X_s = (X_c + 1)/2
    Y_s = (Y_c + 1)/2
    changed_image1 = img_resize1[int(height*(scale - 1)*Y_s) : int(height*(scale - 1)*Y_s + height), int(width*(scale - 1)*X_s) : int(width*(scale -1)*X_s + width)].copy()
    return changed_image1, scale


