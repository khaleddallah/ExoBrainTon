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
 * thormang3_offset_tuner_server.cpp
 *
 *  Created on: 2016. 2. 15.
 *      Author: Jay Song
 */

#ifndef THORMANG3_OFFSET_TUNER_SERVER_H_
#define THORMANG3_OFFSET_TUNER_SERVER_H_

#include <map>
#include <fstream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "robotis_controller/robotis_controller.h"
#include "thormang3_base_module/base_module.h"
#include "thormang3_offset_tuner_msgs/JointOffsetData.h"
#include "thormang3_offset_tuner_msgs/JointTorqueOnOffArray.h"
#include "thormang3_offset_tuner_msgs/GetPresentJointOffsetData.h"

namespace thormang3
{

class JointOffsetData
{
public:
  double joint_offset_rad_;
  double joint_init_pos_rad_;
  int p_gain_;
  int i_gain_;
  int d_gain_;

  JointOffsetData()
  {
    joint_offset_rad_ = 0;
    joint_init_pos_rad_ = 0;
    p_gain_ = 32;
    i_gain_ = 0;
    d_gain_ = 0;
  }

  JointOffsetData(double joint_offset_rad, double joint_init_pose_rad)
  {
    this->joint_offset_rad_ = joint_offset_rad;
    this->joint_init_pos_rad_ = joint_init_pose_rad;
    p_gain_ = 32;
    i_gain_ = 0;
    d_gain_ = 0;
  }

  ~JointOffsetData()
  {
  }
};

class OffsetTunerServer: public robotis_framework::Singleton<OffsetTunerServer>
{
private:
  //RobotisController* controller_;

  robotis_framework::RobotisController* controller_;

  std::string init_file_;
  std::string robot_file_;
  std::string offset_file_;

  std::map<std::string, bool>             robot_torque_enable_data_;
  std::map<std::string, JointOffsetData*> robot_offset_data_;

  ros::Subscriber     send_tra_sub_;
  ros::Subscriber     joint_offset_data_sub_;
  ros::Subscriber     joint_torque_enable_sub_;
  ros::Subscriber     command_sub_;
  ros::ServiceServer  offset_data_server_;

  void setCtrlModule(std::string module);

public:
  OffsetTunerServer();
  ~OffsetTunerServer();

  bool initialize();
  void moveToInitPose();
  void stringMsgsCallBack(const std_msgs::String::ConstPtr& msg);
  void commandCallback(const std_msgs::String::ConstPtr& msg);
  void jointOffsetDataCallback(const thormang3_offset_tuner_msgs::JointOffsetData::ConstPtr &msg);
  void jointTorqueOnOffCallback(const thormang3_offset_tuner_msgs::JointTorqueOnOffArray::ConstPtr& msg);
  bool getPresentJointOffsetDataServiceCallback(thormang3_offset_tuner_msgs::GetPresentJointOffsetData::Request &req,
                                                thormang3_offset_tuner_msgs::GetPresentJointOffsetData::Response &res);

};

}

#endif /* THORMANG3_OFFSET_TUNER_SERVER_H_ */
