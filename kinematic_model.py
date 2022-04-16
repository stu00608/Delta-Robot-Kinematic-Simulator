import numpy as np
import yaml
from math import *
import matplotlib.pyplot as plt
import warnings

class DeltaRobot:

    '''
    Attributes
    ----------
    R : float
        固定平面頂板之邊長
        https://imgur.com/1YKbVEV

    r : float
        活動平面底板為之邊長
        https://imgur.com/1YKbVEV

    M : float
        大臂長
        https://imgur.com/1YKbVEV

    u : float
        小臂長
        https://imgur.com/1YKbVEV

    O : ndarray
        頂板中心之參考點
        https://imgur.com/1YKbVEV

    G : ndarray
        底板中心之末端點
        https://imgur.com/1YKbVEV

    Rh : float
        頂板到馬達軸心距離
        https://imgur.com/MRVvXDq

    Rd : float
        頂板到馬達軸心投影距離
        https://imgur.com/MRVvXDq

    rh : float
        底板到活動軸心距離
        https://imgur.com/4vKcMPy

    rd : float
        底板到活動軸心投影距離
        https://imgur.com/4vKcMPy

    K : float
        頂板中心O至大臂連接處 A,B,C 於x-y平面的正射影長度
        https://imgur.com/7Tm5DOr

    k : float
        底板中心G至小臂連接處 a,b,c 於x-y平面的正射影長度
        https://imgur.com/UaKzcCA

    '''

    def __init__(self, config: str = "params.yaml", **kwargs) -> None:
        self.load_params(config, kwargs)

        # Initialize.
        self.pose_a = np.array([])
        self.pose_b = np.array([])
        self.pose_c = np.array([])
        self.pose_TA = np.array([])
        self.pose_TB = np.array([])
        self.pose_TC = np.array([])
        self.theta_A = 0.
        self.theta_B = 0.
        self.theta_C = 0.

    def __str__(self) -> str:
        return f'''
    Robot Model Description
    ----------

    Static Parameters:
    
    R  = {self.R:.3f}   r  = {self.r:.3f}   O  = {self.O}
    M  = {self.M:.3f}   u  = {self.u:.3f}   G  = {self.G}
    Rh = {self.Rh:.3f}  Rd = {self.Rd:.3f}
    rh = {self.rh:.3f}  rd = {self.rd:.3f}
    K  = {self.K:.3f}   
    k  = {self.k:.3f}

    Position & Angles:

    A:
        motor_pose_A : {self.motor_pose_A}
        pose_TA      : {self.pose_TA}
        theta_A (rad): {self.theta_A:.3f}
        theta_A (deg): {degrees(self.theta_A):.2f}
    B:
        motor_pose_B : {self.motor_pose_B}
        pose_TB      : {self.pose_TB}
        theta_B (rad): {self.theta_B:.3f}
        theta_B (deg): {degrees(self.theta_B):.2f}
    C:
        motor_pose_C : {self.motor_pose_C}
        pose_TC      : {self.pose_TC}
        theta_C (rad): {self.theta_C:.3f}
        theta_C (deg): {degrees(self.theta_C):.2f}
        
        '''

    def load_params(self, config: str, kwargs: dict):      

        params = yaml.safe_load(open(config, 'r'))
        try:
            self.DEBUG = kwargs['debug']
        except:
            self.DEBUG = params['debug']
        try:
            self.R = kwargs['R']
        except:
            self.R = params['R']
        try:
            self.r = kwargs['r']
        except:
            self.r = params['r']
        try:
            self.M = kwargs['M']
        except:
            self.M = params['M']
        try:
            self.u = kwargs['u']
        except:
            self.u = params['u']
        try:
            self.O = np.array(kwargs['O'])
        except:
            self.O = np.array(params['O'])
        try:
            self.G = np.array(kwargs['G'])
        except:
            self.G = np.array(params['G'])
        try:
            self.Rh = kwargs['Rh']
        except:
            self.Rh = params['Rh']
        try:
            self.Rd = kwargs['Rd']
        except:
            self.Rd = params['Rd']
        try:
            self.rh = kwargs['rh']
        except:
            self.rh = params['rh']
        try:
            self.rd = kwargs['rd']
        except:
            self.rd = params['rd']

        self.K = sqrt(self.R**2 - (self.R/2)**2) * 1/3 - self.Rd
        self.k = sqrt(self.r**2 - (self.r/2)**2) * 2/3 - self.rd

        self.motor_pose_A = np.array([self.O[0], self.O[1]-self.K, self.Rh])
        self.motor_pose_B = np.array([self.O[0]+self.K*cos(radians(150)),
                        self.O[1]+self.K*sin(radians(150)), self.O[2]+self.Rh])
        self.motor_pose_C = np.array([self.O[0]+self.K*cos(radians(30)),
                        self.O[1]+self.K*cos(radians(30)),  self.O[2]+self.Rh])

    def compute_T(self, theta_A: float, theta_B: float, theta_C: float):

        '''
        Calculate TA, TB, TC points from motor position and theta.

        Returns
        ----------
        (np.ndarray, np.ndarray, np.ndarray)
            Tuple of TA, TB, TC position.
        '''

        self.pose_TA = np.array(
            [self.motor_pose_A[0],
             self.motor_pose_A[1] - self.M * cos(theta_A),
             self.motor_pose_A[2] - self.M * sin(theta_A)])
        self.pose_TB = np.array([self.motor_pose_B[0] + self.M * cos(radians(150)) * cos(theta_B),
                                 self.motor_pose_B[1] + self.M * sin(radians(150)) * cos(theta_B),
                                 self.motor_pose_B[2] - self.M * sin(theta_B)])
        self.pose_TC = np.array([self.motor_pose_C[0] + self.M * cos(radians(30)) * cos(theta_C),
                                 self.motor_pose_C[1] + self.M * sin(radians(30)) * cos(theta_C),
                                 self.motor_pose_C[2] - self.M * sin(theta_C)])

        return (self.pose_TA, self.pose_TB, self.pose_TC)

    def compute_theta(
            self, connection_coordinate: np.ndarray, A: float, B: float, isA: bool = False):
        
        '''
        Calculate theta base on elbow point and static parameters.

        Returns
        ----------
        float
            theta value in radians.
        '''

        # TODO: isA is not a good style, need to figure out why there is a double in B and C.
        square_dist = (
            connection_coordinate[0] ** 2 + connection_coordinate[1] ** 2 +
            connection_coordinate[2] ** 2)
        isA = 2 if isA else 1
        theta = asin((-self.u**2 + square_dist + self.M**2) /
                     (isA*self.M*sqrt(A**2 + B**2))) - atan(B/A)

        return theta

    def inverse_kinematic(self, **kwargs):
        '''
        Calculate inverse kinematic, save data in the object.

        Returns
        ----------
        (float, float, float)
            A tuple of computed theta A, B, C.
        '''

        try:
            self.G = np.array(kwargs['G'])
        except:
            warnings.warn("No G parameter detect, use the default G position from config.", Warning)
        
        if self.DEBUG:
            print("Inverse Kinematic calculating.")

        # TODO: Explaination of these three points, should make a better var name.
        self.pose_a = np.array([self.G[0], self.G[1]-self.k, self.G[2]+self.rh])
        self.pose_b = np.array([self.G[0]+self.k*cos(radians(150)),
                                self.G[1]+self.k*sin(radians(150)), self.G[2]+self.rh])
        self.pose_c = np.array([self.G[0]+self.k*cos(radians(30)),
                                self.G[1]+self.k*sin(radians(30)), self.G[2]+self.rh])

        # NOTE: The elbow point.
        connection_coordinate_A = self.motor_pose_A - self.pose_a
        connection_coordinate_B = self.motor_pose_B - self.pose_b
        connection_coordinate_C = self.motor_pose_C - self.pose_c

        self.theta_A = self.compute_theta(
            connection_coordinate_A, connection_coordinate_A[1],
            connection_coordinate_A[2], isA=True)
        self.theta_B = self.compute_theta(
            connection_coordinate_B, -2 * connection_coordinate_B[2],
            -sqrt(3) * connection_coordinate_B[0] + connection_coordinate_B[1])
        self.theta_C = self.compute_theta(
            connection_coordinate_C, -2 * connection_coordinate_C[2],
            sqrt(3) * connection_coordinate_C[0] + connection_coordinate_C[1])

        # TODO: Make a better var name for this, clarify with connect_coordinate point.
        self.compute_T(self.theta_A, self.theta_B, self.theta_C)

        return (self.theta_A, self.theta_B, self.theta_C)

    def forward_kinematic(self, input_angle: np.ndarray):
        '''
        Parameters
        ----------
        input_angle: np.ndarray
            Angles of theta_A, theta_B, theta_C in degrees.
        '''

        theta_A = radians(input_angle[0])
        theta_B = radians(input_angle[1])
        theta_C = radians(input_angle[2])

        (pose_TA, pose_TB, pose_TC) = self.compute_T(theta_A, theta_B, theta_C)
        self.pose_TA = pose_TA
        self.pose_TB = pose_TB
        self.pose_TC = pose_TC

        # Set lambdaArray, alphaArray, betaArray, gammaArray, phiArray
        lambdaArray = np.array([self.u**2-pose_TA[0]**2-pose_TA[1]**2-pose_TA[2]**2, self.u **
                               2-pose_TB[0]**2-pose_TB[1]**2-pose_TB[2]**2, self.u**2-pose_TC[0]**2-pose_TC[1]**2-pose_TC[2]**2])

        alphaArray = np.array([sqrt(3)*self.k-2*pose_TA[0],   -4*(3/4)*self.k-2*pose_TA[1],    -2*pose_TA[2]])
        betaArray = np.array([-2*pose_TB[0],             -2*pose_TB[1],              -2*pose_TB[2]])
        gammaArray = np.array([2*(3**0.5)*self.k-2*pose_TC[0], -2*pose_TC[1],              -2*pose_TC[2]])

        phiArray = np.array([lambdaArray[0]-((3/4)*self.k**2)-(4*(self.k**2)*0.5625)+(3**0.5) *
                            self.k*pose_TA[0]-3*self.k*pose_TA[1], lambdaArray[1], lambdaArray[2]-3*(self.k**2)+2*(3**0.5)*self.k*pose_TC[0]])

        A = ((alphaArray[1]-betaArray[1])*(alphaArray[2]-gammaArray[2]) - (alphaArray[2]-betaArray[2])*(alphaArray[1]-gammaArray[1])) / \
            ((alphaArray[0]-betaArray[0])*(alphaArray[1]-gammaArray[1]) -
             (alphaArray[1]-betaArray[1])*(alphaArray[0]-gammaArray[0]))
        B = ((alphaArray[1]-gammaArray[1])*(phiArray[0]-phiArray[1]) - (alphaArray[1]-betaArray[1])*(phiArray[0]-phiArray[2])) / \
            ((alphaArray[0]-betaArray[0])*(alphaArray[1]-gammaArray[1]) -
             (alphaArray[1]-betaArray[1])*(alphaArray[0]-gammaArray[0]))
        C = ((alphaArray[0]-betaArray[0])*(alphaArray[2]-gammaArray[2]) - (alphaArray[2]-betaArray[2])*(alphaArray[0]-gammaArray[0])) / \
            ((alphaArray[1]-betaArray[1])*(alphaArray[0]-gammaArray[0]) -
             (alphaArray[0]-betaArray[0])*(alphaArray[1]-gammaArray[1]))
        D = ((alphaArray[0]-gammaArray[0])*(phiArray[0]-phiArray[1]) - (alphaArray[0]-betaArray[0])*(phiArray[0]-phiArray[2])) / \
            ((alphaArray[1]-betaArray[1])*(alphaArray[0]-gammaArray[0]) -
             (alphaArray[0]-betaArray[0])*(alphaArray[1]-gammaArray[1]))

        a = alphaArray[0] - betaArray[0]
        b = alphaArray[1] - betaArray[1]
        c = alphaArray[2] - betaArray[2]
        # get a, b, c position
        zb_temp = (float(phiArray[0])-float(phiArray[1])-a*B-b*D) / (a*A+b*C+c)
        if self.DEBUG:
            print("lambdaArray", lambdaArray)
            print("TA = ", pose_TA)
            print("TB = ", pose_TB)
            print("TC = ", pose_TC)
            print("phiArray =", phiArray)
            print("a*B", a*B)
            print("b*D", b*D)
            print("分子 = ", (float(phiArray[0])-float(phiArray[1])-a*B-b*D))
            print("分母", (a*A+b*C+c))
            print("A = ", A)
            print("B = ", B)
            print("C = ", C)
            print("D = ", D)
            print("a = ", a)
            print("b = ", b)
            print("c = ", c)
            print("zb = ", zb_temp)
        self.pose_b = np.array([A*zb_temp + B, C*zb_temp + D, zb_temp])
        self.pose_a = np.array([self.pose_b[0]+self.k*sin(radians(60)), self.pose_b[1]-2*self.k*(sin(radians(60))**2), self.pose_b[2]])
        self.pose_c = np.array([self.pose_b[0]+2*self.k*sin(radians(60)), self.pose_b[1], self.pose_b[2]])
        # TODO: Update self.G?
        self.pose_G = np.array([self.pose_a[0], self.pose_a[1]+self.k, self.pose_a[2]-self.rh])
        if self.DEBUG:
            print("u = ", sqrt((self.pose_TA[0]-self.pose_a[0])**2 +
                (self.pose_TA[1]-self.pose_a[1])**2+(self.pose_TA[2]-self.pose_a[2])**2))
            print("u = ", sqrt((self.pose_TB[0]-self.pose_b[0])**2 +
                (self.pose_TB[1]-self.pose_b[1])**2+(self.pose_TB[2]-self.pose_b[2])**2))
            print("u = ", sqrt((self.pose_TC[0]-self.pose_c[0])**2 +
                (self.pose_TC[1]-self.pose_c[1])**2+(self.pose_TC[2]-self.pose_c[2])**2))
        

    def plot(self, canvas):
        canvas.scatter(self.motor_pose_A[0], self.motor_pose_A[1], self.motor_pose_A[2], c='b')
        canvas.scatter(self.motor_pose_B[0], self.motor_pose_B[1], self.motor_pose_B[2], c='g')
        canvas.scatter(self.motor_pose_C[0], self.motor_pose_C[1], self.motor_pose_C[2], c='y')

        canvas.scatter(self.pose_a[0], self.pose_a[1], self.pose_a[2], c='r')
        canvas.scatter(self.pose_b[0], self.pose_b[1], self.pose_b[2], c='r')
        canvas.scatter(self.pose_c[0], self.pose_c[1], self.pose_c[2], c='r')
        canvas.plot([self.pose_a[0], self.pose_b[0]], [self.pose_a[1], self.pose_b[1]],
                    [self.pose_a[2], self.pose_b[2]], linewidth=1, c='m')
        canvas.plot([self.pose_a[0], self.pose_c[0]], [self.pose_a[1], self.pose_c[1]],
                    [self.pose_a[2], self.pose_c[2]], linewidth=1, c='m')
        canvas.plot([self.pose_b[0], self.pose_c[0]], [self.pose_b[1], self.pose_c[1]],
                    [self.pose_b[2], self.pose_c[2]], linewidth=1, c='m')

        canvas.scatter(self.pose_TA[0], self.pose_TA[1], self.pose_TA[2], c='b')
        canvas.scatter(self.pose_TB[0], self.pose_TB[1], self.pose_TB[2], c='g')
        canvas.scatter(self.pose_TC[0], self.pose_TC[1], self.pose_TC[2], c='y')

        canvas.plot([self.motor_pose_A[0], self.pose_TA[0]], [self.motor_pose_A[1], self.pose_TA[1]], [
                    self.motor_pose_A[2], self.pose_TA[2]], linewidth=1, c='k')
        canvas.plot([self.pose_a[0], self.pose_TA[0]], [self.pose_a[1], self.pose_TA[1]], [
                    self.pose_a[2], self.pose_TA[2]], linewidth=1, c='k')
        canvas.plot([self.motor_pose_B[0], self.pose_TB[0]], [self.motor_pose_B[1], self.pose_TB[1]], [
                    self.motor_pose_B[2], self.pose_TB[2]], linewidth=1, c='k')
        canvas.plot([self.pose_b[0], self.pose_TB[0]], [self.pose_b[1], self.pose_TB[1]], [
                    self.pose_b[2], self.pose_TB[2]], linewidth=1, c='k')
        canvas.plot([self.motor_pose_C[0], self.pose_TC[0]], [self.motor_pose_C[1], self.pose_TC[1]], [
                    self.motor_pose_C[2], self.pose_TC[2]], linewidth=1, c='k')
        canvas.plot([self.pose_c[0], self.pose_TC[0]], [self.pose_c[1], self.pose_TC[1]], [
                    self.pose_c[2], self.pose_TC[2]], linewidth=1, c='k')

        canvas.set_xlim(-300, 300)
        canvas.set_ylim(-300, 300)
        canvas.set_zlim(-300, 300)
        canvas.set_zlabel("Z")
        canvas.set_ylabel("Y")
        canvas.set_xlabel("X")
