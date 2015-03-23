#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
#  Copyright (c) 2014, Ocean Systems Laboratory, Heriot-Watt University, UK.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Heriot-Watt University nor the names of
#     its contributors may be used to endorse or promote products
#     derived from this software without specific prior written
#     permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

from __future__ import division, with_statement

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('bvt_pantilt')

from sensor_msgs.msg import JointState
from bvt_pantilt.msg import PanTiltOrientation

# topics
TOPIC_ORIENT = 'pantilt/orientation'
TOPIC_REQUEST = 'pantilt/orientation_request'
TOPIC_JOINT = '/joint_states'

# constants
INIT_ROLL = 0.0                 # rad
INIT_POSE = [0.0, 0.0]          # rad, rad
INIT_DELTA = 0.10               # sec

DEFAULT_AZ_MIN = -179.0         # deg
DEFAULT_AZ_MAX = 179.0          # deg
DEFAULT_EL_MIN = -30.0          # deg
DEFAULT_EL_MAX = 70.0           # deg

DEFAULT_AZ_SPD = 2.0            # deg/s
DEFAULT_EL_SPD = 2.0            # deg/s


def wrapTo2Pi(theta):
    """Normalize an angle in radians to [0, 2*pi]"""
    return theta % (2.0 * np.pi)

def wrapToPi(theta):
    """Normalize an angle in radians to [-pi, pi]"""
    return (wrapTo2Pi(theta + np.pi) - np.pi)


class PanTiltSimulator(object):
    def __init__(self, name):
        self.node_name = name

        # initial state
        self.roll = rospy.get_param('pantilt_roll', INIT_ROLL)
        self.time_delta = rospy.get_param('pantilt_delta', INIT_DELTA)
        self.current_pose = rospy.get_param('pantilt_pose', INIT_POSE)

        self.goal_pose = self.current_pose

        # current config (rads)
        self.azimuth_min_ang = np.deg2rad(rospy.get_param('pantilt_az_min', DEFAULT_AZ_MIN))
        self.azimuth_max_ang = np.deg2rad(rospy.get_param('pantilt_az_max', DEFAULT_AZ_MAX))
        self.elevation_min_ang = np.deg2rad(rospy.get_param('pantilt_el_min', DEFAULT_EL_MIN))
        self.elevation_max_ang = np.deg2rad(rospy.get_param('pantilt_el_max', DEFAULT_EL_MAX))

        self.azimuth_speed = rospy.get_param('pantilt_az_spd', DEFAULT_AZ_SPD)
        self.elevation_speed = rospy.get_param('pantilt_el_spd', DEFAULT_EL_SPD)

        # ros interface
        self.joint = rospy.Publisher(TOPIC_JOINT, JointState, queue_size=10)
        self.sub = rospy.Subscriber(TOPIC_REQUEST, PanTiltOrientation, self.pantilt_callback, queue_size=1)
        self.pub = rospy.Publisher(TOPIC_ORIENT, PanTiltOrientation, latch=True, queue_size=10)
                
        self.tim = rospy.Timer(rospy.Duration(self.time_delta), self.pantilt_executor)


    def pantilt_executor(self, event=None):   
        if self.goal_pose != self.current_pose:
            # azimuth
            if self.goal_pose[0] > self.current_pose[0]:
                self.current_pose[0] += np.deg2rad(self.azimuth_speed * self.time_delta)
                self.current_pose[0] = np.min(self.current_pose[0], self.goal_pose[0]) 
            else:
                self.current_pose[0] -= np.deg2rad(self.azimuth_speed * self.time_delta)
                self.current_pose[0] = np.max(self.current_pose[0], self.goal_pose[0])

            # elevation 
            if self.goal_pose[1] > self.current_pose[1]:
                self.current_pose[1] += np.deg2rad(self.elevation_speed * self.time_delta)
                self.current_pose[1] = np.min(self.current_pose[1], self.goal_pose[1]) 
            else:
                self.current_pose[1] -= np.deg2rad(self.elevation_speed * self.time_delta)
                self.current_pose[1] = np.max(self.current_pose[1], self.goal_pose[1])
  
        self.current_pose = self.validate_orientation(self.current_pose)

        # publish current pose here 
        joint = JointState(
            name = ['pan', 'tilt', 'roll'],
            position = [
                self.current_pose[0],
                self.current_pose[1], 
                self.roll
            ],
            velocity = []
        )

        joint.header.stamp = rospy.Time.now()
        self.joint.publish(joint)
        
        # send position feedback
        orientation = PanTiltOrientation()
        orientation.header.stamp = joint.header.stamp
        orientation.azimuth = np.rad2deg(self.current_pose[0])
        orientation.elevation = np.rad2deg(self.current_pose[1])
        self.pub.publish(orientation)

    def pantilt_callback(self, data):
        """Get current orientation request and execute the movement"""
        req = [np.deg2rad(data.azimuth), np.deg2rad(data.elevation)]
        self.goal_pose = self.validate_orientation(req)

    def validate_orientation(self, data):
        # angle wrap
        data[0] = wrapToPi(data[0])   
        data[1] = wrapToPi(data[1]) 

        # list [azimuth, elevation]
        data[0] = np.clip(data[0], self.azimuth_min_ang, self.azimuth_max_ang)
        data[1] = np.clip(data[1], self.elevation_min_ang, self.elevation_max_ang)

        return data

if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    name = rospy.get_name()

    rospy.loginfo("[{}] initializing the node..\n".format(self.name))

    try:
        sim = PanTiltSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
