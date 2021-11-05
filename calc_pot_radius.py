import math
import sys

def main():
    a = 212.5
    b = 83.5
    c = 222.5
    d = 64.58
    w = 190.0
    r1 = 68.97/2
    r2 = 70.0/2

    try:
        theta = float(sys.argv[1]) / 180.0 *math.pi

        A = pow(a, 2) + a*w -2*(a + w/2)*(c*math.sin(theta) + d*math.cos(theta)) + pow(c, 2) + pow(d, 2) + pow(r1, 2) - pow(r2, 2) + 2*b*(c*math.cos(theta)-d*math.sin(theta)) + pow(b, 2)
        B = c*math.cos(theta) - d*math.sin(theta) + b
        C = pow((r1 - r2)/B, 2) - 1
        D = A/pow(B, 2) * (r1 - r2) - 2*r1
        E = pow(A/(2*B), 2) + pow(w/2, 2) - pow(r1, 2)

        R = (- D - math.sqrt(pow(D, 2) - 4*C*E))/(2*C)
        '''
        r = 209.91/2
        
        print(A + 2*(r1-r2)*r - 2*B*math.sqrt(pow(r1+r, 2)-pow(w/2, 2)))

        print(pow(a+w/2-c*math.sin(theta)-d*math.cos(theta), 2) + pow(math.sqrt(pow(r1+r, 2) - pow(w/2, 2)) - c * math.cos(theta) + d*math.sin(theta) - b, 2) - pow(r2+r, 2))

        l1 = pow(a+w/2, 2) - 2*(a+w/2)*(c*math.sin(theta) + d*math.cos(theta)) + pow(c*math.sin(theta) + d*math.cos(theta), 2)
        l2 = pow(r1+r, 2)- pow(w/2, 2) - 2*math.sqrt(pow(r1+r, 2) - pow(w/2, 2))*(b+c*math.cos(theta)-d*math.sin(theta)) + pow(b+c*math.cos(theta) -d*math.sin(theta), 2)

        print("A : {0}".format(A))
        print("B : {0}".format(B))
        print("C : {0}".format(C))
        print("D : {0}".format(D))
        print("E : {0}".format(E))

        print("1 : {0}".format(pow(a+w/2-c*math.sin(theta)-d*math.cos(theta), 2)- l1))
        print("2 : {0}".format(pow(math.sqrt(pow(r1+r, 2) - pow(w/2, 2)) - c * math.cos(theta) + d*math.sin(theta) - b, 2)- l2))
        '''

        print("pot radius : {0} mm".format(R*2))


    except:
        print("usage: python calc_pot_radius.py theta_value(0~90)")


if __name__=="__main__":
    main()