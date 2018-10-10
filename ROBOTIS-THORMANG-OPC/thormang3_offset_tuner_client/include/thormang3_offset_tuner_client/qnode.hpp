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

/* Author: Kayman Jung, sch */

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef thormang3_offset_tuner_client_QNODE_HPP_
#define thormang3_offset_tuner_client_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN

#include <string>
#include <QThread>
#include <QStringListModel>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

#include "thormang3_offset_tuner_msgs/JointOffsetData.h"
#include "thormang3_offset_tuner_msgs/JointOffsetPositionData.h"
#include "thormang3_offset_tuner_msgs/JointTorqueOnOff.h"
#include "thormang3_offset_tuner_msgs/JointTorqueOnOffArray.h"
#include "thormang3_offset_tuner_msgs/GetPresentJointOffsetData.h"

#endif  // Q_MOC_RUN

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace thormang3_offset_tuner_client
{

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode : public QThread
{
Q_OBJECT
 public:
  enum LogLevel
  {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();

  QStringListModel* loggingModel()
  {
    return &logging_model_;
  }
  void log(const LogLevel &level, const std::string &msg);

  void send_torque_enable_msg(thormang3_offset_tuner_msgs::JointTorqueOnOffArray msg);
  void send_joint_offset_data_msg(thormang3_offset_tuner_msgs::JointOffsetData msg);
  void send_command_msg(std_msgs::String msg);
  bool is_refresh()
  {
    return is_refresh_;
  }

  std::map<int, std::string> right_arm_offset_group;
  std::map<int, std::string> left_arm_offset_group;
  std::map<int, std::string> legs_offset_group;
  std::map<int, std::string> body_offset_group;

 public Q_SLOTS:
  void getPresentJointOffsetData();

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

  void update_present_joint_offset_data(thormang3_offset_tuner_msgs::JointOffsetPositionData msg);

 private:
  void ParseOffsetGroup(const std::string &path);

  int init_argc_;
  char** init_argv_;
  bool is_refresh_;
  QStringListModel logging_model_;

  ros::Publisher joint_offset_data_pub_;
  ros::Publisher torque_enable_pub_;
  ros::Publisher command_pub_;

  ros::ServiceClient get_present_joint_offset_data_client_;
};

}  // namespace thormang3_offset_tuner_client

#endif /* thormang3_offset_tuner_client_QNODE_HPP_ */
