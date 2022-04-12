import numpy as np
from math import *
import matplotlib.pyplot as plt
from kinematic_model import DeltaRobot

if __name__ == '__main__':

    dr = DeltaRobot()

    point_A = np.array([dr.O[0], dr.O[1]-dr.K, dr.Rh])
    point_B = np.array([dr.O[0]+dr.K*cos(radians(150)),
                       dr.O[1]+dr.K*sin(radians(150)), dr.O[2]+dr.Rh])
    point_C = np.array([dr.O[0]+dr.K*cos(radians(30)),
                       dr.O[1]+dr.K*cos(radians(30)),  dr.O[2]+dr.Rh])

    dr.inverse_kinematic(point_A, point_B, point_C)

    plt_ik = plt.subplot(121, projection='3d')
    plt_ik.set_title("IK")
    dr.plot(plt_ik)


    input_angle = np.array([88.74,71.214,10])
    dr.forward_kinematic(input_angle)

    plt_fk = plt.subplot(122, projection='3d')
    plt_fk.set_title("FK")
    dr.plot(plt_fk)

    plt.show()

