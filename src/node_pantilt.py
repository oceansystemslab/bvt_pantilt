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

import roslib
import rospy
roslib.load_manifest('bvt_pantilt')

import numpy as np
import time
import traceback

from driver_pantilt import PanTiltDriver, ProtocolException
from serial import SerialException

from bvt_pantilt.msg import PanTiltOrientation
from sensor_msgs.msg import JointState

# topics
TOPIC_ORIENT = 'pantilt/orientation'
TOPIC_REQUEST = 'pantilt/orientation_request'
TOPIC_JOINT = '/joint_states'

TIMER_JOINT = 0.1   # default 10Hz for joint_state info publishing

# geometry compensation
OFFSET_PAN = 0      # (optional) add an offset to pan (nessie: 45 - 17.5)
OFFSET_TILT = 0     # (optional) add an offset to tilt (nessie: 0)
MIN_DELTA = 0.55    # minimum angle difference


class PanTiltInterface(object):

    def __init__(self, name, serial_config, offset_pan=OFFSET_PAN, offset_tilt=OFFSET_TILT):
        # driver status (fsm)
        #   0:  not connected
        #   1:  idle
        #   2:  operating
        #   4:  emergency
        self.status = 0
        self.new_request = True
        self.serial_config = serial_config
        self.name = name

        # geometry conf
        self.offset_pan = offset_pan
        self.offset_tilt = offset_tilt

        # final geometry limits
        self.MIN_AZIMUTH = -180 + offset_pan     # limit pan taking into account the offset (+) 
        self.MAX_AZIMUTH = 180 - offset_pan     # limit pan taking into account the offset (-)
        self.MIN_ELEVATION = -30 - offset_tilt        # limit tilt taking into account the offset (+) 
        self.MAX_ELEVATION = 70 + offset_tilt   # limit tilt taking into account the offset (-)

        # zero config
        self.init_theta = 180 - offset_pan  # set initial azimuth to 0      (nessie: 135 + 17.5)
        self.init_phi = 180 - offset_tilt    # set initial elevation to 0    (nessie: 180)

        # initialize driver
        self.dev = PanTiltDriver(self.serial_config)
        self.theta_des = self.init_theta
        self.phi_des = self.init_phi
        self.prev_theta = 0.0
        self.prev_phi = 0.0
        self.azimuth = 0.0
        self.elevation = 0.0

        # initialize ros handlers
        self.pub = rospy.Publisher(TOPIC_ORIENT, PanTiltOrientation, latch=True, queue_size=10)
        self.sub = rospy.Subscriber(TOPIC_REQUEST, PanTiltOrientation, self.cb_orientation, queue_size=1)

        # joint
        self.joint = rospy.Publisher(TOPIC_JOINT, JointState)
        self.timer_joint = rospy.Timer(rospy.Duration(TIMER_JOINT), self.cb_joints)


    def cb_joints(self, event):
        # publish joint state
        pan = np.deg2rad(self.azimuth)
        tilt = np.deg2rad(self.elevation)
        roll = 0.0

        joint = JointState(
            name = ['pan', 'tilt', 'roll'],
            position = [pan, tilt, roll],
            velocity = []
        )

        joint.header.stamp = rospy.Time.now()
        self.joint.publish(joint)


    def cb_orientation(self, data):
        """Note that all angles are in degrees.
            azimuth: 0-360
            elevation: 0-90

            The low-level driver will enforce device limits, plus here
            we are enforcing geometry limits (mounted sonar in body-frame)!
        """

        # geometry transformation
        az = data.azimuth
        el = data.elevation

        # wrap angles between (-180,+180)
        while az > 180:
            az -= 360

        while az < -180:
            az += 360

        while el > 180:
            el -= 360

        while el < -180:
            el += 360

        # enforce geometry limits
        az = max(self.MIN_AZIMUTH, min(az, self.MAX_AZIMUTH))
        el = max(self.MIN_ELEVATION, min(el, self.MAX_ELEVATION))

        # geometry conversion
        #   from sonar body-frame to pan&tilt protocol
        az = (az + 180.0)
        el = (el + 180.0)

        # store desired orientation
        self.theta_des = az - self.offset_pan
        self.phi_des = el - self.offset_tilt

        # extra info
        rospy.loginfo('[PT]: az: {0:0.2f}, el: {1:0.2f} -- theta: {2:0.2f}, phi: {3:0.2f}'.format(data.azimuth, data.elevation, az, el))

        # raise a request flag
        self.new_request = True

    def run(self):
        try:
            while not rospy.is_shutdown():
                # check if still connected
                if self.status is 0:
                    try:
                        self.dev.connect()
                        self.status = 1
                    except SerialException as se:
                        rospy.logerr('device not found, waiting for device [%s] ...', self.serial_config['port'])
                        rospy.logdebug(se)
                        # wait before reconnect
                        self.status = 0
                        rospy.sleep(5)
                        continue

                # connected, poll device
                try:
                    theta = self.dev.get_pan_angle()
                    phi = self.dev.get_tilt_angle()
                except ProtocolException as pe:
                    # low-level driver will apply brakes in case of exceptions
                    self.status = 4
                    rospy.logerr(pe)
                except SerialException as se:
                    rospy.logfatal(se)
                    self.status = 0

                # update device angles (used for the joint state publisher)
                self.azimuth = (theta - 180) + self.offset_pan
                self.elevation = (phi - 180) + self.offset_tilt

                # send position feedback
                orientation = PanTiltOrientation()
                orientation.header.stamp = rospy.Time.now()
                orientation.azimuth = self.azimuth
                orientation.elevation = self.elevation
                self.pub.publish(orientation)

                # calculate how far is the device from the requested orientation
                theta_delta = abs(theta - self.theta_des)
                phi_delta = abs(phi - self.phi_des)

                # process incoming requests (precision is limited by hardware device)
                if self.theta_des != self.prev_theta or self.phi_des != self.prev_phi:
                    # store angles
                    self.prev_theta = self.theta_des
                    self.prev_phi = self.phi_des
                    self.status = 2

                    # NOTE: DO NOT USE A LOWER CONSTRAINT OTHERWISE THE DEVICE WILL BURN!
                    # if theta_delta > MIN_DELTA or phi_delta > MIN_DELTA:
                    if self.new_request:
                        # send commands
                        self.dev.set_pan_angle(self.theta_des)
                        self.dev.set_tilt_angle(self.phi_des)
                        self.new_request = True

                    # extra info
                    rospy.loginfo('[op]: theta: {0:0.2f}, phi: {1:0.2f}, theta_des: {2:0.2f}, phi_des: {3:0.2f}'.format(theta, phi, self.theta_des, self.phi_des))
                else:
                    self.status = 1

                # # apply brakes
                # if theta_delta <= MIN_DELTA and phi_delta <= MIN_DELTA:
                #     self.dev.set_brake(self.dev.id_pan, self.dev.PAN_BRAKE_DEFAULT)
                #     self.dev.set_brake(self.dev.id_tilt, self.dev.TILT_BRAKE_DEFAULT)
                #     rospy.logdebug('[PT]: pan&tilt brakes on ...')

        except Exception as e:
            rospy.logfatal(e)

            rospy.logfatal('Uncaught exception, dying!')
            traceback.print_exc()


        if self.status is not 0:
            self.dev.disconnect()


def main():
    rospy.init_node('pantilt_node')
    name = rospy.get_name()

    rospy.loginfo('%s initializing ...', name)

    # local flags
    verbose = False

    # local args (without ROS commands)
    args = rospy.myargv()

    if len(args) > 1:
        if args[1] == '-v':
            verbose = True

    # load parameters
    serial_port = rospy.get_param('~/serial_port', default='/vdev/tty_pantilt')

    try:
        op = int(rospy.get_param('~/offset_pan', default=OFFSET_PAN))
        ot = int(rospy.get_param('~/offset_tilt', default=OFFSET_TILT))
    except Exception:
        op = 0
        ot = 0

    # check input
    offset_pan = np.maximum(np.minimum(op, 45), -45)
    offset_tilt = np.maximum(np.minimum(ot, 10), -10)

    # shutdown handlers
    #rospy.on_shutdown(...)

    # initial logging
    rospy.loginfo('%s serial device: %s', name, serial_port)
    rospy.loginfo('%s pan offset: %s degrees', name, offset_pan)
    rospy.loginfo('%s titl offset: %s degrees', name, offset_tilt)

    # device configuration
    serial_config = {'port': serial_port}

    # run device interface
    device = PanTiltInterface(name, serial_config, offset_pan=offset_pan, offset_tilt=offset_tilt)
    device.run()    # blocking call

    rospy.loginfo('pantilt_node shutdown ...')

if __name__ == '__main__':
    main()
