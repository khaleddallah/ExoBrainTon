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
 * HeadControlModule.cpp
 *
 *  Created on: 2016. 1. 27.
 *      Author: kayman
 */

#include <stdio.h>
#include "thormang3_head_control_module/head_control_module.h"

using namespace thormang3;

HeadControlModule::HeadControlModule()
    : control_cycle_msec_(0),
      stop_process_(false),
      is_moving_(false),
      is_direct_control_(false),
      tra_count_(0),
      tra_size_(0),
      current_state_(None),
      original_position_lidar_(0.0),
      moving_time_(3.0),
      scan_range_(0.0),
      DEBUG(false)
{
  enable_ = false;
  module_name_ = "head_control_module";
  control_mode_ = robotis_framework::PositionControl;

  result_["head_y"] = new robotis_framework::DynamixelState();
  result_["head_p"] = new robotis_framework::DynamixelState();

  using_joint_name_["head_y"] = 0;
  using_joint_name_["head_p"] = 1;

  target_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  current_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  tra_gene_thread_ = 0;
}

HeadControlModule::~HeadControlModule()
{
  queue_thread_.join();
}

void HeadControlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  queue_thread_ = boost::thread(boost::bind(&HeadControlModule::queueThread, this));

  control_cycle_msec_ = control_cycle_msec;

  ros::NodeHandle ros_node;

  /* publish topics */
  moving_head_pub_ = ros_node.advertise<std_msgs::String>("/robotis/sensor/move_lidar", 0);
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 0);

  movement_done_pub_ = ros_node.advertise<std_msgs::String>("/robotis/movement_done", 1);
}

void HeadControlModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscribe topics */
  ros::Subscriber get_3d_lidar_sub = ros_node.subscribe("/robotis/head_control/move_lidar", 1,
                                                        &HeadControlModule::get3DLidarCallback, this);
  ros::Subscriber get_3d_lidar_range_sub = ros_node.subscribe("/robotis/head_control/move_lidar_with_range", 1,
                                                              &HeadControlModule::get3DLidarRangeCallback, this);
  ros::Subscriber set_head_joint_sub = ros_node.subscribe("/robotis/head_control/set_joint_states", 1,
                                                          &HeadControlModule::setHeadJointCallback, this);
  ros::Subscriber set_head_joint_time_sub = ros_node.subscribe("/robotis/head_control/set_joint_states_time", 1,
                                                               &HeadControlModule::setHeadJointTimeCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

void HeadControlModule::get3DLidarCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false || is_moving_ == true)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Fail to move Lidar");
    publishDoneMsg("scan_failed");
    return;
  }

  if (DEBUG)
    fprintf(stderr, "TOPIC CALLBACK : get_3d_lidar\n");

  if (current_state_ == None)
  {
    // turn off direct control and move head joint in order to make pointcloud
    is_direct_control_ = false;
    scan_range_ = 0.0;
    beforeMoveLidar(SCAN_START_ANGLE);

    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start head joint in order to make pointcloud");
  }
  else
  {
    ROS_INFO("Head is used.");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Fail to move Lidar");
  }
}

void HeadControlModule::get3DLidarRangeCallback(const std_msgs::Float64::ConstPtr &msg)
{
  if (enable_ == false || is_moving_ == true)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Fail to move Lidar");
    publishDoneMsg("scan_failed");
    return;
  }

  if (DEBUG)
    fprintf(stderr, "TOPIC CALLBACK : get_3d_lidar\n");

  if (current_state_ == None)
  {
    // turn off direct control and move head joint in order to make pointcloud
    is_direct_control_ = false;
    scan_range_ = msg->data;
    double start_angle = current_position_.coeffRef(0, using_joint_name_["head_p"]) - scan_range_;
    beforeMoveLidar(start_angle);

    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start head joint in order to make pointcloud");
  }
  else
  {
    ROS_INFO("Head is used.");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Fail to move Lidar");
  }
}

void HeadControlModule::setHeadJointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (enable_ == false)
  {
    ROS_INFO("Head module is not enable.");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Not Enable");
    publishDoneMsg("head_control_failed");
    return;
  }

  if (is_moving_ == true && is_direct_control_ == false)
  {
    ROS_INFO("Head is moving now.");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Head is busy.");
    publishDoneMsg("head_control_failed");
    return;
  }

  // moving time
  moving_time_ = 1.0;        // default : 1 sec

  // set target joint angle
  target_position_ = goal_position_;    // default

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    std::string joint_name = msg->name[ix];
    std::map<std::string, int>::iterator iter = using_joint_name_.find(joint_name);

    if (iter != using_joint_name_.end())
    {
      // set target position
      target_position_.coeffRef(0, iter->second) = msg->position[ix];

      // set time
      int calc_moving_time = fabs(goal_position_.coeff(0, iter->second) - target_position_.coeff(0, iter->second))
          / 0.45;
      if (calc_moving_time > moving_time_)
        moving_time_ = calc_moving_time;

      if (DEBUG)
      {
        std::cout << "joint : " << joint_name << ", Index : " << iter->second << ", Angle : " << msg->position[ix]
                  << ", Time : " << moving_time_ << std::endl;
      }
    }
  }

  // set init joint vel, accel
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  if (is_moving_ == true && is_direct_control_ == true)
  {
    goal_velocity_ = calc_joint_vel_tra_.block(tra_count_, 0, 1, result_.size());
    goal_acceleration_ = calc_joint_accel_tra_.block(tra_count_, 0, 1, result_.size());
  }

  // set mode
  is_direct_control_ = true;

  // generate trajectory
  tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneThread, this));
  delete tra_gene_thread_;
}

void HeadControlModule::setHeadJointTimeCallback(const thormang3_head_control_module_msgs::HeadJointPose::ConstPtr &msg)
{
  if (enable_ == false)
  {
    ROS_INFO("Head module is not enable.");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Not Enable");
    publishDoneMsg("head_control_failed");
    return;
  }

  if (is_moving_ == true && is_direct_control_ == false)
  {
    ROS_INFO("Head is moving now.");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Head is busy.");
    publishDoneMsg("head_control_failed");
    return;
  }

  // moving time
  moving_time_ = msg->mov_time;

  // set target joint angle
  target_position_ = goal_position_;  // default

  for (int ix = 0; ix < msg->angle.name.size(); ix++)
  {
    std::string joint_name = msg->angle.name[ix];
    std::map<std::string, int>::iterator iter = using_joint_name_.find(joint_name);

    if (iter != using_joint_name_.end())
    {
      // set target position
      target_position_.coeffRef(0, iter->second) = msg->angle.position[ix];

      // set time
      int calc_moving_time = fabs(goal_position_.coeff(0, iter->second) - target_position_.coeff(0, iter->second))
          / 0.45;
      if (moving_time_ == 0)
        moving_time_ = calc_moving_time;

      if (DEBUG)
      {
        std::cout << "joint : " << joint_name << ", Index : " << iter->second << ", Angle : " << msg->angle.position[ix]
                  << ", Time : " << moving_time_ << std::endl;
      }
    }
  }

  // set init joint vel, accel
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  if (is_moving_ == true && is_direct_control_ == true)
  {
    goal_velocity_ = calc_joint_vel_tra_.block(tra_count_, 0, 1, result_.size());
    goal_acceleration_ = calc_joint_accel_tra_.block(tra_count_, 0, 1, result_.size());
  }

  // set mode
  is_direct_control_ = true;

  // generate trajectory
  tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneThread, this));
  delete tra_gene_thread_;
}

void HeadControlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  tra_lock_.lock();

  // get joint data
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
      state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    int index = using_joint_name_[joint_name];

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    current_position_.coeffRef(0, index) = dxl->dxl_state_->present_position_;
    goal_position_.coeffRef(0, index) = dxl->dxl_state_->goal_position_;
  }

  // check to stop
  if (stop_process_ == true)
  {
    stopMoving();
  }
  else
  {
    // process
    if (tra_size_ != 0)
    {
      // start of steps
      if (tra_count_ == 0)
      {
        startMoving();
      }

      // end of steps
      if (tra_count_ >= tra_size_)
      {
        finishMoving();
      }
      else
      {
        goal_position_ = calc_joint_tra_.block(tra_count_, 0, 1, result_.size());
        tra_count_ += 1;
      }
    }
  }
  tra_lock_.unlock();

  // set joint data
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
      state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    int index = using_joint_name_[joint_name];

    result_[joint_name]->goal_position_ = goal_position_.coeff(0, index);
  }
}

void HeadControlModule::stop()
{
  tra_lock_.lock();

  if (is_moving_ == true)
    stop_process_ = true;

  tra_lock_.unlock();

  return;
}

bool HeadControlModule::isRunning()
{
  return is_moving_;
}

void HeadControlModule::startMoving()
{
  is_moving_ = true;

  // set current lidar mode
  if (is_direct_control_ == false)
  {
    current_state_ = (current_state_ + 1) % ModeCount;
    ROS_INFO_STREAM("state is changed : " << current_state_);

    if (current_state_ == StartMove)
      publishLidarMoveMsg("start");
  }
}

void HeadControlModule::finishMoving()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;

  // handle lidar state
  switch (current_state_)
  {
    case BeforeStart:
    {
      // generate start trajectory
      double target_angle =
          (scan_range_ == 0) ?
              SCAN_END_ANGLE : current_position_.coeffRef(0, using_joint_name_["head_p"]) + scan_range_ * 2;
      startMoveLidar(target_angle);
      break;
    }
    case StartMove:
      publishLidarMoveMsg("end");
      current_state_ = EndMove;

      // generate next trajectory
      afterMoveLidar();
      break;

    case AfterMove:
      current_state_ = None;
      is_direct_control_ = true;
      is_moving_ = false;
      scan_range_ = 0.0;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO,
                       "Finish head joint in order to make pointcloud");
      break;

    default:
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Head movement is finished.");
      is_moving_ = false;
      publishDoneMsg("head_control");

      break;
  }

  // is_direct_control_ = false;
  if (DEBUG)
    std::cout << "Trajectory End" << std::endl;
}

void HeadControlModule::stopMoving()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_moving_ = false;

  // handle lidar state
  switch (current_state_)
  {
    case StartMove:
      publishLidarMoveMsg("end");

    default:
      // stop moving
      current_state_ = None;
      is_direct_control_ = true;
      break;
  }

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Stop Module.");
  stop_process_ = false;
}

void HeadControlModule::beforeMoveLidar(double start_angle)
{
  // angle and moving time
  original_position_lidar_ = goal_position_.coeff(0, using_joint_name_["head_p"]);
  moving_time_ = fabs(current_position_.coeffRef(0, using_joint_name_["head_p"]) - start_angle) / (30 * DEGREE2RADIAN);
  double min_moving_time = 1.0;

  moving_time_ = (moving_time_ < min_moving_time) ? min_moving_time : moving_time_;

  // moving_time_ = 1.0;

  // set target joint angle : pitch
  target_position_ = goal_position_;
  target_position_.coeffRef(0, using_joint_name_["head_p"]) = start_angle;

  // set init joint vel, accel
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  // generate trajectory
  tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneThread, this));
  delete tra_gene_thread_;

  ROS_INFO("Go to Lidar start position");
}

void HeadControlModule::startMoveLidar(double target_angle)
{
  // angle and moving time
  moving_time_ = fabs(current_position_.coeffRef(0, using_joint_name_["head_p"]) - target_angle) / (10 * DEGREE2RADIAN);
  double max_moving_time = 8.0;

  moving_time_ = (moving_time_ < max_moving_time) ? moving_time_ : max_moving_time;

  // moving_time_ = 8.0;        // 8 secs

  // set target joint angle
  target_position_ = goal_position_;
  target_position_.coeffRef(0, using_joint_name_["head_p"]) = target_angle;

  // set init joint vel, accel
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  // generate trajectory
  //tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneThread, this));
  tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::lidarJointTraGeneThread, this));
  delete tra_gene_thread_;

  ROS_INFO("Go to Lidar end position");
}

void HeadControlModule::afterMoveLidar()
{
  // angle and moving time
  moving_time_ = 2.0;

  // set target joint angle : pitch
  target_position_ = goal_position_;
  target_position_.coeffRef(0, using_joint_name_["head_p"]) = original_position_lidar_;

  // set init joint vel, accel
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  // generate trajectory
  tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneThread, this));
  delete tra_gene_thread_;

  ROS_INFO("Go to Lidar before position");
}

void HeadControlModule::publishLidarMoveMsg(std::string msg_data)
{
  std_msgs::String msg;
  msg.data = msg_data;

  moving_head_pub_.publish(msg);

  if (msg_data == "end")
    publishDoneMsg("scan");
}

/*
 simple minimum jerk trajectory

 pos_start : position at initial state
 vel_start : velocity at initial state
 accel_start : acceleration at initial state

 pos_end : position at final state
 vel_end : velocity at final state
 accel_end : acceleration at final state

 smp_time : sampling time

 mov_time : movement time
 */
Eigen::MatrixXd HeadControlModule::calcMinimumJerkTraPVA(double pos_start, double vel_start, double accel_start,
                                                         double pos_end, double vel_end, double accel_end,
                                                         double smp_time, double mov_time)
{
  Eigen::MatrixXd poly_matrix(3, 3);
  Eigen::MatrixXd poly_vector(3, 1);

  poly_matrix << robotis_framework::powDI(mov_time, 3), robotis_framework::powDI(mov_time, 4), robotis_framework::powDI(
      mov_time, 5), 3 * robotis_framework::powDI(mov_time, 2), 4 * robotis_framework::powDI(mov_time, 3), 5
      * robotis_framework::powDI(mov_time, 4), 6 * mov_time, 12 * robotis_framework::powDI(mov_time, 2), 20
      * robotis_framework::powDI(mov_time, 3);

  poly_vector << pos_end - pos_start - vel_start * mov_time - accel_start * pow(mov_time, 2) / 2, vel_end - vel_start
      - accel_start * mov_time, accel_end - accel_start;

  Eigen::MatrixXd poly_coeff = poly_matrix.inverse() * poly_vector;

  int all_time_steps = round(mov_time / smp_time + 1);

  Eigen::MatrixXd time = Eigen::MatrixXd::Zero(all_time_steps, 1);
  Eigen::MatrixXd minimum_jer_tra = Eigen::MatrixXd::Zero(all_time_steps, 3);

  for (int step = 0; step < all_time_steps; step++)
    time.coeffRef(step, 0) = step * smp_time;

  for (int step = 0; step < all_time_steps; step++)
  {
    // position
    minimum_jer_tra.coeffRef(step, 0) = pos_start + vel_start * time.coeff(step, 0)
        + 0.5 * accel_start * robotis_framework::powDI(time.coeff(step, 0), 2)
        + poly_coeff.coeff(0, 0) * robotis_framework::powDI(time.coeff(step, 0), 3)
        + poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 4)
        + poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 5);
    // velocity
    minimum_jer_tra.coeffRef(step, 1) = vel_start + accel_start * time.coeff(step, 0)
        + 3 * poly_coeff.coeff(0, 0) * robotis_framework::powDI(time.coeff(step, 0), 2)
        + 4 * poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 3)
        + 5 * poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 4);
    // accel
    minimum_jer_tra.coeffRef(step, 2) = accel_start + 6 * poly_coeff.coeff(0, 0) * time.coeff(step, 0)
        + 12 * poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 2)
        + 20 * poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 3);
  }

  return minimum_jer_tra;
}

Eigen::MatrixXd HeadControlModule::calcLinearInterpolationTra(double pos_start, double pos_end, double smp_time,
                                                              double mov_time)
{
  double time_steps = mov_time / smp_time;
  int all_time_steps = round(time_steps + 1);
  double next_step = (pos_end - pos_start) / all_time_steps;

  Eigen::MatrixXd minimum_jerk_tra = Eigen::MatrixXd::Zero(all_time_steps, 1);

  for (int step = 0; step < all_time_steps; step++)
    minimum_jerk_tra.coeffRef(step, 0) = pos_start + next_step * (step + 1);

  return minimum_jerk_tra;
}

void HeadControlModule::jointTraGeneThread()
{
  tra_lock_.lock();

  double smp_time = control_cycle_msec_ * 0.001;  // ms -> s
  int all_time_steps = int(moving_time_ / smp_time) + 1;

  calc_joint_tra_.resize(all_time_steps, result_.size());
  calc_joint_vel_tra_.resize(all_time_steps, result_.size());
  calc_joint_accel_tra_.resize(all_time_steps, result_.size());

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
      state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    int index = using_joint_name_[joint_name];

    double ini_value = goal_position_.coeff(0, index);
    double ini_vel = goal_velocity_.coeff(0, index);
    double ini_accel = goal_acceleration_.coeff(0, index);

    double tar_value = target_position_.coeff(0, index);

    Eigen::MatrixXd tra = calcMinimumJerkTraPVA(ini_value, ini_vel, ini_accel, tar_value, 0.0, 0.0, smp_time,
                                                moving_time_);

    calc_joint_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 0, all_time_steps, 1);
    calc_joint_vel_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 1, all_time_steps, 1);
    calc_joint_accel_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 2, all_time_steps, 1);
  }

  tra_size_ = calc_joint_tra_.rows();
  tra_count_ = 0;

  if (DEBUG)
    ROS_INFO("[ready] make trajectory : %d, %d", tra_size_, tra_count_);

  // init value
  // moving_time_ = 0;

  tra_lock_.unlock();
}

void HeadControlModule::lidarJointTraGeneThread()
{
  tra_lock_.lock();

  double smp_time = control_cycle_msec_ * 0.001;  // ms -> s
  int all_time_steps = int(moving_time_ / smp_time) + 1;

  calc_joint_tra_.resize(all_time_steps, result_.size());

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
      state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    int index = using_joint_name_[joint_name];

    double ini_value = goal_position_.coeff(0, index);
    double tar_value = target_position_.coeff(0, index);

    Eigen::MatrixXd tra = calcLinearInterpolationTra(ini_value, tar_value, smp_time, moving_time_);

    calc_joint_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 0, all_time_steps, 1);
  }

  tra_size_ = calc_joint_tra_.rows();
  tra_count_ = 0;

  if (DEBUG)
    ROS_INFO("[ready] make trajectory : %d, %d", tra_size_, tra_count_);

  // init value
  // moving_time_ = 0;

  tra_lock_.unlock();
}

void HeadControlModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Head Control";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

void HeadControlModule::publishDoneMsg(const std::string done_msg)
{
  std_msgs::String movement_msg;
  movement_msg.data = done_msg;
  movement_done_pub_.publish(movement_msg);
}
