from tracemalloc import start
import numpy as np
from math import *
import matplotlib.pyplot as plt
from kinematic_model import DeltaRobot
import argparse

def move(dr, target):

    '''
    Plot the robot at target point.

    Parameters
    ----------
    dr: DeltaRobot
        The robot object.
    target: list
        A 3D position as the target point for G.
    '''

    plt_ik = plt.subplot(111, projection='3d')
    dr.inverse_kinematic(G=target)
    dr.plot(plt_ik)
    plt.show()

def make_trajectory(start, end, samples):

    '''
    Parameters
    ----------
    start: list
        start position, `[x, y, z]`.
    end: list
        end position, `[x, y, z]`. 
    samples: int
        How many point to generate between this two points.
    
    Returns
    ----------
    list
        A list of points inside, each element is represent a point as `[x, y, z]`.
    '''

    diff_x = end[0] - start[0]
    diff_y = end[1] - start[1]
    diff_z = end[2] - start[2]

    interval_x = diff_x/(samples+1)
    interval_y = diff_y/(samples+1)
    interval_z = diff_z/(samples+1)

    points = []
    for i in range(1, samples+1):
        x = start[0] + interval_x*i
        y = start[1] + interval_y*i
        z = start[2] + interval_z*i
        points.append([x, y, z])
    
    return points
def make_circle(start, samples):
    '''
    Parameters
    ----------
    start: list
        start position, `[x, y, z]`.

    samples: int
        How many point to generate between this two points.
    
    Returns
    ----------
    list
        A list of points inside, each element is represent a point as `[x, y, z]`.
    '''
    point = []
    for i in range (samples):
        x = start[0]*cos(radians(i*360/samples))
        y = start[1]*sin(radians(i*360/samples))
        z = start[2]
        point.append([x, y, z])
    return point,samples
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='A program to simulate the movement of 3-Dimension Delta robot.')
    parser.add_argument("-r", "--round", default=1, type=int, help="Round to move in follow_points")
    parser.add_argument("-v", "--velocity", default=1, type=float, help="Movement velocity.")
    args = parser.parse_args()

    height = -223.6
    dr = DeltaRobot()
    dr.inverse_kinematic(G=[0, 0, height])

    rounds = args.round
    velocity = args.velocity
    follow_points = [[ 50,  100, height],
                     [ -50,  100, height],  
                     [ -50, 0, height],  
                     [ 50, 0, height]]

    plt_ik = plt.subplot(111, projection='3d')
    
    for i in range(len(follow_points)*rounds):
        start_point_index = i%len(follow_points)
        start_point = follow_points[start_point_index]
        end_point_index = (i+1)%len(follow_points)
        end_point = follow_points[end_point_index]

        move_list = [start_point]
        move_list += make_trajectory(start_point, end_point, samples=10)
        move_list.append(end_point)

        for point in move_list:
            plt.cla()
            dr.inverse_kinematic(G=point)
            dr.plot(plt_ik)
            plt.draw()
            plt.pause(0.01/velocity)
    move_lists,sample = make_circle([30,30,-223.6], samples = 36)
    plot_circle = plt.subplot(111, projection='3d')
    for i in range (sample):
        plt.cla()
        dr.inverse_kinematic(G=move_lists[i])
        dr.plot(plot_circle)
        plt.draw()
        plt.pause(0.01/velocity)


        