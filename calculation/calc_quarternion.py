import math
import sys

def main():
    try:
        roll = float(sys.argv[1])*math.pi/180.0
        pitch = float(sys.argv[2])*math.pi/180.0
        yaw = float(sys.argv[3])*math.pi/180.0

        print(quaternion_from_euler(roll, pitch, yaw))

    except:
        print("usage: python calc_pot_radius.py roll(deg) pitch (deg) yaw(deg)")



def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

if __name__=="__main__":
    main()