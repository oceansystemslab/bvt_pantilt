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

import math
import time
import serial

from serial import Serial, SerialException


SERIAL_DEFAULT = {
    'port': '/vdev/tty_pantilt',
    'baudrate': 9600,
    'bytesize': serial.EIGHTBITS,
    'parity': serial.PARITY_NONE,
    'stopbits': serial.STOPBITS_ONE,
    'timeout': 1
}

TIMING = {
    'T1': 0.040,    # wait between char and echo
    'T2': 0.025,    # wait between echo and next char
    'T3': 0.050,    # minimum time between commands
}

class ProtocolException(Exception):
    pass


class PanTiltDriver(object):
    """BlueView Pan&Tilt Driver
        This client uses the RS485 Protocol for RemoteOceanicSociety Pan&Tilt unit.
    """

    def __init__(self, serial_config=None):
        # positioners ids
        self.id_pan = 'A'
        self.id_tilt = 'B'

        # default settings
        self.PAN_BRAKE_DEFAULT = 100
        self.TILT_BRAKE_DEFAULT = 110

        # initialize pan with fail-safe values
        self.pan_pos = 500
        self.pan_req_pos = 500
        self.pan_min = 500
        self.pan_max = 500
        self.pan_ccw = 500
        self.pan_cw = 500
        self.pan_brake = self.PAN_BRAKE_DEFAULT
        self.pan_acc = 0
        self.pan_vel = 0

        # initialize tilt with fail-safe values
        self.tilt_pos = 500
        self.tilt_req_pos = 500
        self.tilt_min = 500
        self.tilt_max = 500
        self.tilt_ccw = 500
        self.tilt_cw = 500
        self.tilt_brake = self.PAN_BRAKE_DEFAULT
        self.tilt_acc= 0
        self.tilt_vel = 0

        # serial connection
        self.ser = None
        self.ser_conf = SERIAL_DEFAULT

        if serial_config is not None:
            self.ser_conf.update(serial_config)


    def __init_pan(self):
        settings = self.get_factory_settings(self.id_pan)
        self.pan_min = settings['ccw_user']
        self.pan_max = settings['cw_user']
        self.pan_ccw = settings['ccw_factory']
        self.pan_cw = settings['cw_factory']

        self.pan_acc = self.get_acceleration(self.id_pan)
        self.pan_vel = self.get_velocity(self.id_pan)
        self.pan_pos = self.get_position(self.id_pan)

        self.set_brake(self.id_pan, self.PAN_BRAKE_DEFAULT)
        self.pan_brake = self.PAN_BRAKE_DEFAULT

        # store factory string
        self.pan_conf = self.get_factory_string(self.id_pan)

    def __init_tilt(self):
        settings = self.get_factory_settings(self.id_tilt)
        self.tilt_min = settings['ccw_user']
        self.tilt_max = settings['cw_user']
        self.tilt_ccw = settings['ccw_factory']
        self.tilt_cw = settings['cw_factory']

        self.tilt_acc = self.get_acceleration(self.id_tilt)
        self.tilt_vel = self.get_velocity(self.id_tilt)
        self.tilt_pos = self.get_position(self.id_tilt)

        self.set_brake(self.id_tilt, self.TILT_BRAKE_DEFAULT)
        self.tilt_brake = self.TILT_BRAKE_DEFAULT

        # store factory string
        self.tilt_conf = self.get_factory_string(self.id_tilt)


    def _send_command(self, cmd):
        """Send command 'cmd' using serial connection 'ser'
            'cmd' is ASCII encoded according to ros RS485 protocol
            'ser' is a pyserial.Serial connection object
        """
        for c in cmd:
            self.ser.write(c)
            time.sleep(TIMING['T1'])

            e = self.ser.read(1)
            time.sleep(TIMING['T2'])

            if e != c:
                raise ProtocolException('Wrong command readback: {0}'.format(cmd))


    def _send_command_feedback(self, cmd, count):
        """Send command 'cmd' using serial connection 'ser' and read feedback from node
            'cmd' is ASCII encoded according to ros RS485 protocol
            'ser' is a pyserial.Serial connection object
            'feedback' is the number of bytes to read
        """
        self._send_command(cmd)
        time.sleep(TIMING['T3'])
        feedback = self.ser.read(count)

        if len(feedback) < count:
            raise ProtocolException('Feedback error for: {0}'.format(cmd))

        return feedback


    def connect(self):
        if self.ser is None:
            try:
                # open serial connection
                self.ser = Serial(**self.ser_conf)

                # clear buffers
                self.ser.flushInput()
                self.ser.flushOutput()
            except SerialException as se:
                raise se
            else:
                self.__init_pan()
                self.__init_tilt()

    def disconnect(self):
        if self.ser is not None:
            if self.ser.isOpen():
                self.set_brake(self.id_pan, self.PAN_BRAKE_DEFAULT)
                self.set_brake(self.id_tilt, self.TILT_BRAKE_DEFAULT)

                self.ser.flush()
                self.ser.close()


    @property
    def connected(self):
        if self.ser is not None:
            return self.ser.isOpen()
        else:
            return False


    # unit actions
    def set_pan_angle(self, angle):
        alpha = max(0, min(angle, 360))
        req = math.ceil( ((alpha / 360.0) * (self.pan_cw - self.pan_ccw)) + self.pan_ccw )
        req = int(req)

        # enforce user limits
        self.pan_req_pos = max(self.pan_min, min(req, self.pan_max))

        # request position
        self.set_position(self.id_pan, self.pan_req_pos)


    def set_tilt_angle(self, angle):
        alpha = max(0, min(angle, 360))
        req = math.ceil( ((alpha / 360.0) * (self.tilt_cw - self.tilt_ccw)) + self.tilt_ccw )
        req = int(req)

        # enforce user limits
        self.tilt_req_pos = max(self.tilt_min, min(req, self.tilt_max))

        # request position
        self.set_position(self.id_tilt, self.tilt_req_pos)


    # protocol actions
    def set_position(self, node, pos):
        cmd = '{0}p{1:03d}'.format(node[0], pos)

        try:
            self._send_command(cmd)
        except ProtocolException as pe:
            # apply brakes and re-raise exception
            self.set_brake(self.id_pan, self.PAN_BRAKE_DEFAULT)
            self.set_brake(self.id_tilt, self.TILT_BRAKE_DEFAULT)
            raise pe

    def set_brake(self, node, brake):
        """Set node brake value using the specified value (0-128)
            es. As000 --> maximum brake (high current brake)
            es. As128 --> minimum brake (low current, be careful with heavy weights)
        """
        value = max(0, min(brake, 128))     # enforce 0-128 range

        cmd = '{0}s{1:03d}'.format(node[0], value)
        self._send_command(cmd)


    # unit requests
    def get_pan_angle(self):
        self.pan_pos = self.get_position(self.id_pan)
        return ((self.pan_pos - self.pan_ccw) * 360.0) / (self.pan_cw - self.pan_ccw)

    def get_tilt_angle(self):
        self.tilt_pos = self.get_position(self.id_tilt)
        return ((self.tilt_pos - self.tilt_ccw) * 360.0) / (self.tilt_cw - self.tilt_ccw)


    # protocol requests
    def get_factory_string(self, node):
        return self._send_command_feedback(node[0] + '?000', 33)

    def get_factory_settings(self, node):
        feedback = self._send_command_feedback(node[0] + '?000', 33)
        fields = feedback.split(',')

        return {
            'id': fields[0],
            'ccw_factory': int(fields[1]),
            'cw_factory': int(fields[2]),
            'ccw_user': int(fields[3]),
            'cw_user': int(fields[4]),
            'dash': fields[5],
            'feedback': fields[6],
            'rate': fields[7],
            'device': fields[8],
            'firmware': fields[9]
        }

    def get_acceleration(self, node):
        """Query for positioner acceleration setting, returns deg/s2 units.

            According to RS485 Protocol Specification (p. 64-65)
                0:  2 deg/s2
                1:  4 deg/s2
                2:  6 deg/s2
                3:  8 deg/s2
                4:  10 deg/s2
        """
        feedback = self._send_command_feedback(node[0] + '?003', 4)
        raw = int(feedback[1:])

        if raw < 0 or raw > 4:
            return -1
        else:
            return 2 + (raw * 2)

    def get_velocity(self, node):
        """Query for positioner velocity setting, returns deg/s units.

            According to RS485 Protocol Specification (p. 24)
                001:  0.5  deg/s
                002:  1.0  deg/s
                004:  2.0  deg/s
                008:  4.0  deg/s
                ...   ...   ...
                080:  40.0 deg/s
        """
        feedback = self._send_command_feedback(node[0] + '?004', 4)
        raw = int(feedback[1:])

        if raw < 1 or raw > 80:
            return -1
        else:
            return raw * 0.5

    def get_brake(self, node):
        feedback = self._send_command_feedback(node[0] + '?006', 4)
        return int(feedback[1:])

    def get_position(self, node):
        feedback = self._send_command_feedback(node[0] + 'f', 4)
        return int(feedback[1:])


    def __str__(self):
        if self.ser is None:
            return 'Pan&Tilt: Disconnected!'
        else:
            s = 'Pan&Tilt Unit:\n'
            s += '  pan_pos: {0}\n'.format(self.pan_pos)
            s += '  pan_vel: {0}\n'.format(self.pan_vel)
            s += '  pan_acc: {0}\n'.format(self.pan_acc)
            s += '  pan_conf: {0}\n'.format(self.pan_conf)
            s += '  tilt_pos: {0}\n'.format(self.tilt_pos)
            s += '  tilt_vel: {0}\n'.format(self.tilt_vel)
            s += '  tilt_acc: {0}\n'.format(self.tilt_acc)
            s += '  tilt_conf: {0}'.format(self.tilt_conf)
            return s
