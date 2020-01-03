#!/usr/bin/env python2

import rospy

import sympy as sy
import numpy as np
import math as m
import scipy as sc
import scipy.optimize as op
import time

from gazebo_rad_msgs.msg import Cone as ConeMsg
from geometry_msgs.msg import PoseWithCovarianceStamped as PoseWithCovarianceStampedMsg

from std_srvs.srv import SetBool as SetBoolSrv
from std_srvs.srv import Trigger as TriggerSrv

from std_srvs.srv import SetBoolResponse as SetBoolSrvReponse
from std_srvs.srv import TriggerResponse as TriggerSrvResponse

class ConeFitter:

    # #{ dist3D()
    
    def dist3D(self, x, y):
    
        return np.linalg.norm(np.array(x) - np.array(y))
    
    # #} end of dist3D

    # #{ coneDist
    
    def coneDist(self, center, direction, theta):
    
        out = self.cone_dist
    
        for idx,x in enumerate(self.sy_center):
            out = out.subs(x, center[idx]) 
    
        for idx,x in enumerate(self.sy_direction):
            out = out.subs(x, direction[idx]) 
    
        out = out.subs(self.sy_theta, theta) 
    
        return out
    
    # #} end of coneDist
    
    # #{ multiConeDist
    
    def multiConeDist(self, cc, dd, tt):
    
        out = 1
        for idx,i in enumerate(cc):
    
            out = out + sy.Pow(self.coneDist(cc[idx], dd[idx], tt[idx]), 2)
    
        return out
    
    # #} end of multiConeDist
    
    # #{ generate_constraints
    
    def constraints(self, cc, dd):
    
        cons = []
    
        for idx,i in enumerate(cc):
    
            center = cc[idx]
            direction = dd[idx]
    
            cons.append({'type': 'ineq', 'fun': lambda x: +(direction[0]*(x[0]+direction[0]*self.constraint_offset) + direction[1]*(x[1]+direction[1]*self.constraint_offset) + direction[2]*(x[2]+direction[2]*self.constraint_offset) - (direction[0]*center[0] + direction[1]*center[1] + direction[2]*center[2]))})
    
        return cons
    
    # #} end of generate_constraints

    # #{ substitude_scalar
    
    def subs_scal(self, scal, x, values):
    
        for idx1,var in enumerate(x):
    
            scal = scal.subs(var, values[idx1]) 
    
        return sy.N(scal)
    
    # #} end of substitude_scalar

    # #{ substitute_vector
    
    def subs_vec(self, vec, x, values):
    
        out = np.zeros((len(vec)))
    
        for idx1,cell in enumerate(vec):
    
            out[idx1] = self.subs_scal(cell, x, values)
    
        return out
    
    # #} end of substitute_vector

    # #{ substitute_matrix
    
    def subs_mat(self, mat, x, values):
    
        out = np.zeros((len(mat), len(mat[0])))
    
        for idx1,vec in enumerate(mat):
    
            out[idx1] = self.subs_vec(vec, x, values)
    
        return out
    
    # #} end of substitute_matrix

    # #{ critetion (obsolete, slow)
    
    def f(self, point):
        start = time.time()
        f_value = self.subs_scal(self.func, self.sy_x, point)
        end = time.time()
        print("f_value: {} ({} s)".format(f_value, end-start))
        return f_value
    
    # #} end of critetion (obsolete, slow)

    # #{ jacobian (obsolete, slow)
    
    def jacobian(self, point):
        start = time.time()
        jacob = np.array(self.subs_vec(self.J, self.sy_x, point))
        end = time.time()
        print("jacob: {} ({} s)".format(jacob, end-start))
        return jacob
    
    # #} end of jacobian (obsolete, slow)

    # #{ hessian (obsolete, slow)
    
    def hessian(self, point):
        return np.matrix(self.subs_mat(self.H, self.sy_x, point))
    
    # #} end of hessian (obsolete, slow)

    # #{ __init__()
    
    def __init__(self):

        self.is_initialized = False
        self.is_enabled = True
    
        rospy.init_node('cone_fitter', anonymous=True)
    
        # parameters
        self.uav_name_ = rospy.get_param('~uav_name')

        self.cone_num = rospy.get_param('~cone_num')
        self.constraint_offset = rospy.get_param('~constraint_offset')

        # subscribers
        rospy.Subscriber("~cone_in", ConeMsg, self.callbackCone, queue_size=1)

        # services
        self.service_server_enable = rospy.Service('~enable_in', SetBoolSrv, self.callbackEnable)
        self.service_server_reset = rospy.Service('~reset_in', TriggerSrv, self.callbackReset)

        # publishers
        self.publisher_cones = rospy.Publisher("~pose_out", PoseWithCovarianceStampedMsg, queue_size=1)

        self.got_cone = False
        self.got_new_cone = False

        self.cones = []
    
        self.sy_x = sy.symbols('x y z')
        self.sy_center = sy.symbols('a b c')
        self.sy_direction = sy.symbols('o p q')
        self.sy_theta = sy.symbols('Theta')

        norm = sy.Pow(sy.Pow(self.sy_x[0] - self.sy_center[0], 2) + sy.Pow(self.sy_x[1] - self.sy_center[1], 2) + sy.Pow(self.sy_x[2] - self.sy_center[2], 2), 0.5)
        self.cone_dist = norm*sy.sin(sy.acos(((self.sy_x[0]-self.sy_center[0])*self.sy_direction[0] + (self.sy_x[1]-self.sy_center[1])*self.sy_direction[1] + (self.sy_x[2]-self.sy_center[2])*self.sy_direction[2])/(norm)) - self.sy_theta)

        self.is_initialized = True

        rospy.Timer(rospy.Duration(1.0), self.mainTimer)
    
        rospy.spin()
    
    # #} end of __init__(self)

    # #{ callbackCone()
    
    def callbackCone(self, data):
    
        if not self.is_initialized:
            return 

        rospy.loginfo_once('[ConeFitter]: getting radiation')
        self.got_cone = True

        too_close = False

        for idx,cone in enumerate(self.cones):
            if self.dist3D([data.pose.position.x, data.pose.position.y, data.pose.position.z], [cone.pose.position.x, cone.pose.position.y, cone.pose.position.z]) <= 5.0:
                too_close = True
                rospy.loginfo('cone too close')

        if not too_close:
            self.cones.append(data)

        if len(self.cones) > self.cone_num:
            self.cones.pop(0)

        self.got_new_cone = True

    # #} end of callbackCone()

    # #{ callbackEnable()
    
    def callbackEnable(self, req):

        if not self.is_initialized:
            return 
    
        self.is_enabled = req.data

        if self.is_enabled:
            rospy.loginfo('[Optimizer]: enabled')
        else:
            rospy.loginfo('[Optimizer]: disabled')

        resp = SetBoolSrvResponse()
        resp.message = ""
        resp.success = True

        return resp
    
    # #} end of callbackEnable
    
    # #{ callbackReset()
    
    def callbackReset(self, req):

        if not self.is_initialized:
            return 
    
        self.cones = []

        rospy.loginfo('[Optimizer]: resetting')

        resp = TriggerSrvResponse()
        resp.message = ""
        resp.success = True

        return resp
    
    # #} end of callbackEnable

    def mainTimer(self, event):

        if not self.is_initialized:
            return 

        if not self.is_enabled:
            return 

        if len(self.cones) < self.cone_num:
            rospy.loginfo_throttle(1.0, 'waiting for cones (have {}/{})'.format(len(self.cones), self.cone_num))
            return

        if not self.got_new_cone:
            return

        self.got_new_cone = False

        rospy.loginfo('optimizing')

        time_start = rospy.Time.now()

        centers = []
        directions = []
        thetas = []

        for idx,cone in enumerate(self.cones):
            centers.append([cone.pose.position.x, cone.pose.position.y, cone.pose.position.z]) 
            directions.append([cone.direction.x, cone.direction.y, cone.direction.z]) 
            thetas.append(cone.angle)

        self.func = self.multiConeDist(centers, directions, thetas)
        self.cons = self.constraints(centers, directions)

        time_before_J = rospy.Time.now()

        self.J = [self.func.diff(var) for var in self.sy_x]
        # self.H = [[self.func.diff(var1).diff(var2) for var1 in self.sy_x] for var2 in self.sy_x]

        time_before_lamdification = rospy.Time.now()

        # lamdification        
        f = sy.lambdify(self.sy_x, self.func, "math")
        jac = sy.lambdify(self.sy_x, self.J, "math")

        def f_v(x):
            return f(*tuple(x))

        def jac_v(x):
            return jac(*tuple(x))

        time_before_optimization = rospy.Time.now()

        res = op.minimize(f_v, [0, 0, 0], method="SLSQP", jac=jac_v, options={'disp': False}, constraints=self.cons)

        if len(self.cones) < self.cone_num:
          return

        # calculate the maximum distance to any of the cones
        max_distance = 0
        for idx,cone in enumerate(self.cones):

            cone_dist = self.coneDist([cone.pose.position.x, cone.pose.position.y, cone.pose.position.z], [cone.direction.x, cone.direction.y, cone.direction.z], cone.angle)
            dist = self.subs_scal(cone_dist, self.sy_x, res.x)

            if dist > max_distance:
                max_distance = dist

        time_after_optimization = rospy.Time.now()

        rospy.loginfo('Optimized for {} cones, max distance {}: total time {}, initialization {}, jacobian {}, lamdification {}, optimization {}'.format(len(self.cones), max_distance, (time_after_optimization - time_start).to_sec(), (time_before_J - time_start).to_sec(), (time_before_lamdification - time_before_J).to_sec(), (time_before_optimization - time_before_lamdification).to_sec(), (time_after_optimization - time_before_optimization).to_sec()))

        msg_out = PoseWithCovarianceStampedMsg()
        msg_out.header.frame_id = self.uav_name_+"/local_origin"
        msg_out.header.stamp = rospy.Time.now()
        msg_out.pose.pose.position.x = res.x[0]
        msg_out.pose.pose.position.y = res.x[1]
        msg_out.pose.pose.position.z = res.x[2]
        msg_out.pose.pose.orientation.z = 1.0
        msg_out.pose.covariance[0] = max_distance*max_distance
        msg_out.pose.covariance[7] = max_distance*max_distance
        msg_out.pose.covariance[14] = max_distance*max_distance
        self.publisher_cones.publish(msg_out)

if __name__ == '__main__':
    try:
        compton_camera = ConeFitter()
    except rospy.ROSInterruptException:
        pass
