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
 * main.cpp
 *
 *  Created on: 2016. 2. 18.
 *      Author: Jay Song
 */



#include "exo_walk/exo_walk.h"

ros::Subscriber g_demo_command_sub;

bool is_init_pose = false;

void demoCommandCallback(const std_msgs::String::ConstPtr& msg)
{

  ROS_INFO_STREAM("[Demo]  : receive [" << msg->data << "] msg " );

  if(msg->data == "ini_pose")
  {
    ROS_INFO("demo 1: go to initial pose");
    moveToInitPose();
    is_init_pose = true;
    ROS_INFO("[Demo]  : please wait 5 seconds");
  }
  else if ( msg->data == "set_mode")
  {
    ROS_INFO("demo 2: set walking control mode");
    setCtrlModule();
  }
  else if( msg->data == "forward" )
  {
    ROS_INFO("demo 4: forward walking");
    walkForward();
  }
  else if( msg->data == "backward" )
  {
    ROS_INFO("demo 5: backward walking");
    walkBackward();
  }
  else if( msg->data == "balance_on" )
  {
    ROS_INFO("demo 3: balance enable");
    setBalanceOn();
  }
  else if( msg->data == "balance_off" )
  {
    ROS_INFO("demo 3: balance disable");
    setBalanceOff();
  }
  else if( msg->data == "new" )
  {
    ROS_INFO("demo 9: walk_new");
    walk_new();
  }
  else if( msg->data == "stop")
  {
    ROS_INFO("demo 9: stop");
    stop();
  }
  else {
    ROS_ERROR("Invalid Command!!!");
  }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "exo_walk");

  ROS_INFO("ROBOTIS THORMANG3 Walking Simple Demo");

  initialize();

  ros::NodeHandle nh;
  g_demo_command_sub = nh.subscribe("/robotis/walking_demo/command", 10, demoCommandCallback);

  ros::spin();
  return 0;
}

