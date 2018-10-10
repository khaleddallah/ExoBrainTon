/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * gazebo_grip_pub.cpp
 *
 *  Created on: Jul 11, 2015
 *      Author: sch
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>

ros::Publisher g_left_arm_grip_joint_pub;
ros::Publisher g_right_arm_grip_joint_pub;

void leftArmGripJointCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std_msgs::Float64 grip_joint_msg;

  grip_joint_msg.data = msg->data;

  g_left_arm_grip_joint_pub.publish(grip_joint_msg);
}

void rightArmGripJointCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std_msgs::Float64 grip_joint_msg;

  grip_joint_msg.data = msg->data;

  g_right_arm_grip_joint_pub.publish(grip_joint_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_gripper_publisher");
  ros::NodeHandle nh("~");

  g_left_arm_grip_joint_pub = nh.advertise<std_msgs::Float64>("/thormang3/l_arm_grip_1_position/command", 0);
  g_right_arm_grip_joint_pub = nh.advertise<std_msgs::Float64>("/thormang3/r_arm_grip_1_position/command", 0);

  ros::Subscriber l_arm_grip_joint_sub = nh.subscribe("/thormang3/l_arm_grip_position/command", 5, leftArmGripJointCallback);
  ros::Subscriber r_arm_grip_joint_sub = nh.subscribe("/thormang3/r_arm_grip_position/command", 5, rightArmGripJointCallback);

  ros::spin();

  return 0;
}
