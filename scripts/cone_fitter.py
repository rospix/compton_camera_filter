#!/usr/bin/env python2

import rospy

import sympy as sy
import numpy as np
import math as m
import scipy as sc
import scipy.optimize as op
import timeit

from gazebo_rad_msgs.msg import Cone as ConeMsg
from geometry_msgs.msg import PoseStamped as PoseStampedMsg

class ConeFitter:

    def coneDist(self, center, direction, theta):
    
        out = self.cone_dist
    
        for idx,x in enumerate(self.sy_center):
            out = out.subs(x, center[idx]) 
    
        for idx,x in enumerate(self.sy_direction):
            out = out.subs(x, direction[idx]) 
    
        out = out.subs(self.sy_theta, theta) 
    
        return out
    
    def multiConeDist(self, cc, dd, tt):
        
        out = 1
        for idx,i in enumerate(cc):
    
            out = out + sy.Pow(self.coneDist(cc[idx], dd[idx], tt[idx]), 2)
    
        return out
    
    def constraints(self, cc, dd):
    
        cons = []
    
        for idx,i in enumerate(cc):
    
            center = cc[idx]
            direction = dd[idx]
    
            cons.append({'type': 'ineq', 'fun': lambda x: direction[0]*x[0] + direction[1]*x[1] + direction[2]*x[2] - (direction[0]*center[0] + direction[1]*center[1] + direction[2]*center[2])})
    
        return cons

    def subs_scal(self, scal, x, values):

        for idx1,var in enumerate(x):

            scal = scal.subs(var, values[idx1]) 

        return sy.N(scal)

    def subs_vec(self, vec, x, values):

        out = np.zeros((len(vec)))

        for idx1,cell in enumerate(vec):

            out[idx1] = self.subs_scal(cell, x, values)

        return out

    def subs_mat(self, mat, x, values):

        out = np.zeros((size(vec)))

        for idx1,vec in enumerate(mat):

            out[idx1] = self.subs_vec(vec, x, values)

        return out

    def f(self, point):
        return self.subs_scal(self.func, self.sy_x, point)

    def jacobian(self, point):
        return np.array(self.subs_vec(self.J, self.sy_x, point))

    def hessian(self, point):
        return np.matrix(self.subs_mat(self.H, self.sy_x, point))

    # #{ __init__()
    
    def __init__(self):

        self.is_initialized = False
    
        rospy.init_node('cone_fitter', anonymous=True)
    
        # parameters
        # self.rad_timer_dt = rospy.get_param('~rad_timer_dt')
        # subscribers

        rospy.Subscriber("~cone_in", ConeMsg, self.callbackCone, queue_size=1)

        self.got_cone = False

        self.cones = []
    
        # publishers
        self.publisher_cones = rospy.Publisher("~pose_out", PoseStampedMsg, queue_size=1)

        self.sy_x = sy.symbols('x y z')
        self.sy_center = sy.symbols('a b c')
        self.sy_direction = sy.symbols('o p q')
        self.sy_theta = sy.symbols('Theta')

        norm = sy.Pow(sy.Pow(self.sy_x[0] - self.sy_center[0], 2) + sy.Pow(self.sy_x[1] - self.sy_center[1], 2) + sy.Pow(self.sy_x[2] - self.sy_center[2], 2), 0.5)
        self.cone_dist = norm*sy.sin(sy.acos(((self.sy_x[0]-self.sy_center[0])*self.sy_direction[0] + (self.sy_x[1]-self.sy_center[1])*self.sy_direction[1] + (self.sy_x[2]-self.sy_center[2])*self.sy_direction[2])/(norm)) - self.sy_theta)

        self.is_initialized = True
    
        rospy.spin()
    
    # #} end of __init__(self)

    # #{ callbackCone()
    
    def callbackCone(self, data):
    
        if not self.is_initialized:
            return 

        rospy.loginfo_once('[ConeFitter]: getting radiation')
        self.got_cone = True

        self.cones.append(data)

        centers = []
        directions = []
        thetas = []

        for idx,cone in enumerate(self.cones):
            centers.append([cone.pose.position.x, cone.pose.position.y, cone.pose.position.z]) 
            directions.append([cone.direction.x, cone.direction.y, cone.direction.z]) 
            thetas.append(cone.angle)

        self.func = self.multiConeDist(centers, directions, thetas)
        self.cons = self.constraints(centers, directions)

        self.J = [self.func.diff(var) for var in self.sy_x]
        # self.H = [[self.func.diff(var1).diff(var2) for var1 in sy_x] for var2 in sy_x]

        res = op.minimize(self.f, [0, 0, 0], method="SLSQP", jac=self.jacobian, options={'gtol': 1e-6, 'disp': True}, constraints=self.cons)

        print("res: {}".format(res))

        msg_out = PoseStampedMsg()
        msg_out.header.frame_id = "local_origin"
        msg_out.pose.position.x = res.x[0]
        msg_out.pose.position.y = res.x[1]
        msg_out.pose.position.z = res.x[2]
        self.publisher_cones.publish(msg_out)

    # #} end of callbackCone()

if __name__ == '__main__':
    try:
        compton_camera = ConeFitter()
    except rospy.ROSInterruptException:
        pass
