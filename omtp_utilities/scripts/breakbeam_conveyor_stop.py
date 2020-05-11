#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Delft University of Technology
# TU Delft Robotics Institute.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from std_srvs.srv import Empty

from omtp_gazebo.msg import Proximity, ConveyorBeltState
from omtp_gazebo.srv import ConveyorBeltControl


def handler(msg, svc_conveyor_control):
  rospy.loginfo("State change in msg {}: object {}".format(msg.header.seq, 'detected' if msg.object_detected else 'not detected'))

  if msg.object_detected:
    svc_conveyor_control(ConveyorBeltState(power=0.0))
    rospy.loginfo('Stopped conveyor, waiting for next object ..')


if __name__ == "__main__":
  rospy.init_node('breakbeam_conveyor_stop')

  try:
    rospy.wait_for_service('/omtp/conveyor/control', timeout=5.0)
  except Exception as e:
    rospy.logerror("A required service is not available, exiting: {}".format(e))

  svc_conveyor_control = rospy.ServiceProxy('/omtp/conveyor/control', ConveyorBeltControl)

  rospy.loginfo('Configured services, observing break beam, waiting for objects ..')

  rospy.Subscriber('/break_beam_sensor_change', Proximity, handler, callback_args=svc_conveyor_control)

  rospy.spin()
