import time, abc, math, sys
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread, QMutex
from scipy.optimize import minimize, root
import numpy as np
from qtgl import QGLViewer

def eulerAxis(Q):
    """
    Computes the euler axis eX, eY, eZ, which orientation is
    the axis around we are turning and the norm is how much
    we are turning 
    """
    x = Q[2, 1] - Q[1, 2]
    y = Q[0, 2] - Q[2, 0]
    z = Q[1, 0] - Q[0, 1]
    r = math.sqrt(x**2 + y**2 + z**2)
    t = math.atan2(r, Q[0, 0] + Q[1, 1] + Q[2, 2] -1) 

    if abs(r) < 1e-8:
        # The matrix is almost and identity matrix producing no
        # rotation, we can represent it with a null euler axis,
        # to avoid dividing by 0
        return 0, 0, 0

    eX = t*x/r
    eY = t*y/r
    eZ = t*z/r

    return eX, eY, eZ
    

class ChainItem:
    """
    An item of the kinematic chain
    """
    def __init__(self):
        self.object = None
        self.matrix = np.matrix(np.eye(4))

    def gl_draw(self):
        """
        Drawing the item using OpenGL
        """
        pass

class ChainTranslation(ChainItem):
    """
    A translation in the kinematic chain
    """
    def __init__(self, translation):
        super().__init__()
        self.translation = np.array(translation)
        self.matrix[3, 0] = translation[0]
        self.matrix[3, 1] = translation[1]
        self.matrix[3, 2] = translation[2]

    
    def gl_draw(self):
        glLineWidth(5)
        glBegin(GL_LINES)
        glColor3f(0.75, 0.75, 0.75)
        glVertex3f(0, 0, 0)
        glVertex3f(self.matrix[3, 0], self.matrix[3, 1], self.matrix[3, 2])
        glEnd()


class ChainRotation(ChainItem):
    """
    A rotation around one of the x, y or z axis
    """
    def __init__(self, axis):
        super().__init__()
        self.axis = axis

    def gl_draw(self):
        """
        Draw the rotation axis with OpenGL
        """
        glLineWidth(3)
        glBegin(GL_LINES)
        if self.axis == 'x':
            glColor3ub(188, 0, 0)
            glVertex3f(-0.05, 0, 0)
            glVertex3f(.05, 0, 0)
        if self.axis == 'y':
            glColor3ub(111, 188, 0)
            glVertex3f(0, -0.05, 0)
            glVertex3f(0, .05, 0)
        if self.axis == 'z':
            glColor3ub(0, 4, 188)
            glVertex3f(0, 0, -0.05)
            glVertex3f(0, 0, .05)
        glEnd()

    def set_theta(self, theta):
        """
        Sets the value of the rotation
        """
        if self.axis == 'x':
            self.matrix = np.matrix([[1,               0,                0, 0],
                                     [0, math.cos(theta), -math.sin(theta), 0],
                                     [0, math.sin(theta),  math.cos(theta), 0],
                                     [0,                0,               0, 1]])
        if self.axis == 'y':
            self.matrix = np.matrix([[ math.cos(theta), 0, math.sin(theta), 0],
                                     [               0, 1,               0, 0],
                                     [-math.sin(theta), 0, math.cos(theta), 0],
                                     [               0, 0,               0, 1]])
        if self.axis == 'z':
            self.matrix = np.matrix([[math.cos(theta), -math.sin(theta), 0, 0],
                                     [math.sin(theta),  math.cos(theta), 0, 0],
                                     [              0,                0, 1, 0],
                                     [              0,                0, 0, 1]])

class Robot:
    """
    Building the robot
    """
    def __init__(self):
        self.chain =[]
        self.joints = {}
        self.matrix = np.matrix(np.eye(4))
        self.add_joint('hip_yaw', 'z')
        self.add_joint('hip_roll', 'x')
        self.add_joint('hip_pitch', 'y')
        self.add_translation([0.027, 0, -0.027])
        self.add_joint('hip_parallel_top', 'y')
        self.add_translation([0, 0, -0.250])
        self.add_joint('hip_parallel_bottom', 'y')
        self.add_translation([0, 0, -0.104])
        self.add_joint('knee_parallel_top', 'y')
        self.add_translation([0, 0, -0.215])
        self.add_joint('knee_parallel_bottom', 'y')
        self.add_translation([-0.027, 0, -0.027])
        self.add_joint('ankle_pitch', 'y')
        self.add_joint('ankle_roll', 'x')
        self.add_translation([0, 0, -0.046])
        self.theta = np.array([0, 0, 0, -0.05, 0, 0])
        self.h = self.tip()[3, 2]
        self.matrix[3, 2] = -self.h/2;
        self.history = []
        self.mutex = QMutex()

    def add_translation(self, matrix):
        """
        Append a translation to the robot
        """
        self.chain += [ChainTranslation(matrix)]

    def add_joint(self, name, axis):
        """
        Append a joint
        """
        rotation = ChainRotation(axis)
        self.chain += [rotation]
        self.joints[name] = rotation

    def get_joint(self, name):
        """
        Getting joint
        """
        return self.joints[name]

    def gl_draw(self):
        """
        Drawing the robot in OpenGL
        """
        self.mutex.lock()
        glPushMatrix()
        glMultMatrixf(self.matrix)

        for item in self.chain:
            item.gl_draw()
            glMultMatrixf(item.matrix)

        footW = 0.15
        footH = 0.1
        glLineWidth(1)
        glColor3f(0.5, 0.5, 0.5)
        glBegin(GL_LINES)
        glVertex3f(footW, footH, 0)
        glVertex3f(footW, -footH, 0)

        glVertex3f(footW, -footH, 0)
        glVertex3f(-footW, -footH, 0)

        glVertex3f(-footW, -footH, 0)
        glVertex3f(-footW, footH, 0)
        
        glVertex3f(-footW, footH, 0)
        glVertex3f(footW, footH, 0)
        
        glVertex3f(footW, footH, 0)
        glVertex3f(-footW, -footH, 0)
        
        glVertex3f(footW, -footH, 0)
        glVertex3f(-footW, footH, 0)
        glEnd()
        
        glPopMatrix()
        self.mutex.unlock()

    def tip(self):
        """
        Computing the foot tip of the robot
        """
        matrix = np.matrix(np.eye(4))
        for item in self.chain:
            matrix = item.matrix*matrix
        return matrix

    def set_thetas(self, theta):
        """
        Setting all the theta values for all the joint
        """
        self.theta = theta
        robot.get_joint('hip_yaw').set_theta(theta[0])
        robot.get_joint('hip_roll').set_theta(theta[1])
        robot.get_joint('hip_pitch').set_theta(0)
        robot.get_joint('hip_parallel_top').set_theta(-theta[2])
        robot.get_joint('hip_parallel_bottom').set_theta(theta[2])
        robot.get_joint('knee_parallel_top').set_theta(theta[3])
        robot.get_joint('knee_parallel_bottom').set_theta(-theta[3])
        robot.get_joint('ankle_pitch').set_theta(theta[4])
        robot.get_joint('ankle_roll').set_theta(theta[5])

    def ik(self, target):
        """
        Computes the ik for the robot using scipy optimize
        This will produce a 
        """
        def forward(theta):
            # Setting theta for DOFs
            self.set_thetas(theta)
            # Computing forward kinematics
            Q = robot.tip()
            # Extracting x, y and z from the matrix
            x, y, z = np.array(Q)[3][0:3]
            # Getting the euler axis
            eX, eY, eZ = eulerAxis(Q)
            
            return np.array([x, y, z, eX, eY, eZ])

        def loss(theta):
            R = forward(theta)

            # Sum of the square errors
            return np.sum((R-target)**2)
 
        self.mutex.lock()
        # Avoiding starting with bad knee
        if self.theta[3] > 0:
            self.theta[3] = -0.05

        res = minimize(loss, self.theta, method='nelder-mead',
                options={'xtol': 1e-6, 'disp': False, 'maxiter': 5000})
        print(res)
        self.set_thetas(res.x)
        self.mutex.unlock()

        return forward(self.theta)

class RobotWindow(QGLViewer):
    """
    Using QGLViewer for drawing
    """
    def __init__(self, robot):
        super().__init__()
        self.robot = robot

    def gl_draw(self):
        # Grid
        glPushMatrix()
        glTranslatef(0, 0, robot.h/2)
        glLineWidth(1)
        glColor3f(0.3, 0.3, 0.3)
        glBegin(GL_LINES)
        r = 0.5
        D = np.linspace(-r, r, 10)
        for x in D:
            glVertex3f(x, -r, 0)
            glVertex3f(x, r, 0)
        for y in D:
            glVertex3f(-r, y, 0)
            glVertex3f(r, y, 0)

        glEnd()
        glPopMatrix()

        # History
        glLineWidth(1)
        glColor3f(0.7, 0.3, 0.3)
        glBegin(GL_LINES)
        last = None
        for h in robot.history:
            if last != None:
                glVertex3fv(last)
                glVertex3fv(h)
            last = h
        glEnd()

        self.robot.gl_draw()

class AnimationThread(QThread):
    def __init__(self, robot):
        super().__init__()
        self.robot = robot
        self.t = 0

    def run(self):
        while True:
            self.t += 0.02
            time.sleep(0.02)

            # Using some functions to produce kinematics order
            target = np.array([-math.cos(self.t*6)*0.1, 
                               math.sin(self.t*6)*0.05, 
                               -0.6 + max(0,math.sin(self.t*6)*0.1), 
                               0,
                               0, 
                               0
                               ])

            # Solving the IK 
            pos = self.robot.ik(target)

            # Putting the position in the history
            pos[2] -= self.robot.h/2
            robot.history += [pos[0:3]]
            while len(robot.history) > 100:
                robot.history.pop(0)
 
# Building the robot
robot = Robot()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    thread = AnimationThread(robot)
    thread.start()
    window = RobotWindow(robot) 
    window.show()
    app.exec_()

