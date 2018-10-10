#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import time
from std_msgs.msg import String

from thormang3_foot_step_generator.msg import FootStepCommand

# global pub 
# pub = rospy.Publisher('/robotis/walking_demo/command', FootStepCommand, queue_size=10)


def init_exo():
    global g_foot_pub 
    global g_enable_ctrl_module_pub
    g_foot_pub = rospy.Publisher('/robotis/thormang3_foot_step_generator/walking_command', FootStepCommand, queue_size=10)
    g_enable_ctrl_module_pub = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size=10)

    #pub = rospy.Publisher('/robotis/walking_demo/command', String, queue_size=10)
    rospy.init_node('exoint', anonymous=True)

    time.sleep(1)
    g_enable_ctrl_module_pub.publish("walking_module")
    time.sleep(1)

    #rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()

def walk(t=1.5 , l=0.12 ,s=3):
    msg=FootStepCommand()
    msg.command = "forward";
    msg.step_num = s ;
    msg.step_time = t;
    msg.step_length = l;
    msg.side_step_length = 0.0;
    msg.step_angle_rad = 0.0;
    g_foot_pub.publish(msg)

def stop():
    msg=FootStepCommand()
    msg.command = "stop";
    msg.step_num = 10 ;
    msg.step_time = 1.0;
    msg.step_length = 0.1;
    msg.side_step_length = 0.0;
    msg.step_angle_rad = 0.0;
    g_foot_pub.publish(msg)


if __name__ == '__main__':
    try:
        init_exo()
        
        while(True):
            try:
                y=raw_input()
                if (y=='a'):
                    walk()
                    print("walk working...")
                elif (y=='b'):
                    stop()
                    print("stop working...")
                elif (y=='e'):
                    t=input("enter time:\n")
                    l=input("enter len:\n")
                    s=input("enter step:\n")
                    time.sleep(3)
                    walk(t,l,s)
                elif(y=='c'):
                    raise KeyboardInterrupt
                else:
                    print("noooooo")
            except KeyboardInterrupt:
                exit()
                raise


    except rospy.ROSInterruptException:
        pass
