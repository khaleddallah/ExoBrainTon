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
 * thormang3_online_walking.cpp
 *
 *  Created on: 2016. 6. 10.
 *      Author: Jay Song
 */

#include <iostream>
#include <stdio.h>
#include "thormang3_walking_module/thormang3_online_walking.h"


using namespace thormang3;


static const double MMtoM = 0.001;
static const double MStoS = 0.001;

static const int NO_STEP_IDX = -1;

static const double TIME_UNIT = 0.008;

static const int IN_WALKING_STARTING = 0;
static const int IN_WALKING = 1;
static const int IN_WALKING_ENDING = 2;

static const int LEFT_FOOT_SWING  = 1;
static const int RIGHT_FOOT_SWING = 2;
static const int STANDING = 3;

static const int BalancingPhase0 = 0; // DSP : START
static const int BalancingPhase1 = 1; // DSP : R--O->L
static const int BalancingPhase2 = 2; // SSP : L_BALANCING1
static const int BalancingPhase3 = 3; // SSP : L_BALANCING2
static const int BalancingPhase4 = 4; // DSP : R--O<-L
static const int BalancingPhase5 = 5; // DSP : R<-O--L
static const int BalancingPhase6 = 6; // SSP : R_BALANCING1
static const int BalancingPhase7 = 7; // SSP : R_BALANCING2
static const int BalancingPhase8 = 8; // DSP : R->O--L
static const int BalancingPhase9 = 9; // DSP : END

static const int StepDataStatus1 = 1; //
static const int StepDataStatus2 = 2; //
static const int StepDataStatus3 = 3; //
static const int StepDataStatus4 = 4; //


THORMANG3OnlineWalking::THORMANG3OnlineWalking()
  {
  thormang3_kd_ = new KinematicsDynamics(WholeBody);

  present_right_foot_pose_.x = 0.0;    present_right_foot_pose_.y = -0.5*thormang3_kd_->leg_side_offset_m_;
  present_right_foot_pose_.z = -0.63*MMtoM;
  present_right_foot_pose_.roll = 0.0; present_right_foot_pose_.pitch = 0.0; present_right_foot_pose_.yaw = 0.0;

  present_left_foot_pose_.x = 0.0;    present_left_foot_pose_.y = 0.5*thormang3_kd_->leg_side_offset_m_;
  present_left_foot_pose_.z = -0.63*MMtoM;
  present_left_foot_pose_.roll = 0.0; present_left_foot_pose_.pitch = 0.0; present_left_foot_pose_.yaw = 0.0;

  present_body_pose_.x = 0.0;    present_body_pose_.y = 0.0;     present_body_pose_.z = 0.0;
  present_body_pose_.roll = 0.0; present_body_pose_.pitch = 0.0; present_body_pose_.yaw = 0;

  previous_step_right_foot_pose_  = present_right_foot_pose_;
  previous_step_left_foot_pose_   = present_left_foot_pose_;
  previous_step_body_pose_        = present_body_pose_;

  initial_right_foot_pose_ = previous_step_right_foot_pose_;
  initial_left_foot_pose_  = previous_step_left_foot_pose_;
  initial_body_pose_       = previous_step_body_pose_;

  mat_cob_to_rhip_ = robotis_framework::getTranslation4D(0.0,       thormang3_kd_->thormang3_link_data_[ID_R_LEG_START]->relative_position_.coeff(1, 0), 0.0);
  mat_rhip_to_cob_ = robotis_framework::getTranslation4D(0.0, -1.0*(thormang3_kd_->thormang3_link_data_[ID_R_LEG_START]->relative_position_.coeff(1, 0)), 0.0);
  mat_cob_to_lhip_ = robotis_framework::getTranslation4D(0.0,       thormang3_kd_->thormang3_link_data_[ID_L_LEG_START]->relative_position_.coeff(1, 0), 0.0);
  mat_lhip_to_cob_ = robotis_framework::getTranslation4D(0.0, -1.0*(thormang3_kd_->thormang3_link_data_[ID_L_LEG_START]->relative_position_.coeff(1, 0)), 0.0);

  mat_rfoot_to_rft_ = robotis_framework::getRotation4d(M_PI,0,0);
  mat_lfoot_to_lft_ = robotis_framework::getRotation4d(M_PI,0,0);
  rot_x_pi_3d_ = robotis_framework::getRotationX(M_PI);
  rot_z_pi_3d_ = robotis_framework::getRotationZ(M_PI);


  mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(present_body_pose_.x, present_body_pose_.y, present_body_pose_.z,
      present_body_pose_.roll, present_body_pose_.pitch, present_body_pose_.yaw);

  mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);

  mat_robot_to_cob_ = robotis_framework::getRotation4d(present_body_pose_.roll, present_body_pose_.pitch, 0);
  mat_cob_to_robot_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_);
  mat_g_to_robot_   = mat_g_to_cob_ * mat_cob_to_robot_;
  mat_robot_to_g_   = robotis_framework::getInverseTransformation(mat_g_to_robot_);

  mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(present_right_foot_pose_.x, present_right_foot_pose_.y, present_right_foot_pose_.z,
      present_right_foot_pose_.roll, present_right_foot_pose_.pitch, present_right_foot_pose_.yaw);
  mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(present_left_foot_pose_.x, present_left_foot_pose_.y, present_left_foot_pose_.z,
      present_left_foot_pose_.roll, present_left_foot_pose_.pitch, present_left_foot_pose_.yaw);

  mat_robot_to_rfoot_ = mat_robot_to_g_ * mat_g_to_rfoot_;
  mat_robot_to_lfoot_ = mat_robot_to_g_ * mat_g_to_lfoot_;

  rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_) * mat_robot_to_rfoot_);
  lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_) * mat_robot_to_lfoot_);

  thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
  thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);


  r_shoulder_dir_ = thormang3_kd_->thormang3_link_data_[ID_R_ARM_START+2*0]->joint_axis_.coeff(1, 0);
  r_elbow_dir_    = thormang3_kd_->thormang3_link_data_[ID_R_ARM_START+2*3]->joint_axis_.coeff(2, 0);
  l_shoulder_dir_ = thormang3_kd_->thormang3_link_data_[ID_L_ARM_START+2*0]->joint_axis_.coeff(1, 0);
  l_elbow_dir_    = thormang3_kd_->thormang3_link_data_[ID_L_ARM_START+2*3]->joint_axis_.coeff(2, 0);


  r_init_shoulder_angle_rad_ = r_init_elbow_angle_rad_ = r_shoulder_out_angle_rad_ = r_elbow_out_angle_rad_ = 0;
  l_init_shoulder_angle_rad_ = l_init_elbow_angle_rad_ = l_shoulder_out_angle_rad_ = l_elbow_out_angle_rad_ = 0;

  goal_waist_yaw_angle_rad_ = 0.0*M_PI;

  reference_step_data_for_addition_.position_data.moving_foot = STANDING;
  reference_step_data_for_addition_.position_data.elbow_swing_gain = 0.1;
  reference_step_data_for_addition_.position_data.shoulder_swing_gain = 0.05;
  reference_step_data_for_addition_.position_data.foot_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.waist_pitch_angle = 0.0;
  reference_step_data_for_addition_.position_data.waist_yaw_angle = goal_waist_yaw_angle_rad_;
  reference_step_data_for_addition_.position_data.body_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.body_pose = previous_step_body_pose_;
  reference_step_data_for_addition_.position_data.right_foot_pose = previous_step_right_foot_pose_;
  reference_step_data_for_addition_.position_data.left_foot_pose = previous_step_left_foot_pose_;
  reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
  reference_step_data_for_addition_.time_data.abs_step_time = 1.6;
  reference_step_data_for_addition_.time_data.dsp_ratio = 0.2;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_yaw = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_yaw = 0.0;

  present_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;
  previous_step_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;

  shouler_swing_gain_    = 0.05;
  elbow_swing_gain_     = 0.1;

  real_running = false; ctrl_running = false;
  current_step_data_status_ = StepDataStatus4;

  walking_time_ = 0; reference_time_ = 0;
  balancing_index_ = BalancingPhase0;

  preview_time_ = 1.6;
  preview_size_ = round(preview_time_/TIME_UNIT);

  //These parameters are for preview control
  k_s_ = 0;
  current_start_idx_for_ref_zmp_ = 0;
  ref_zmp_x_at_this_time_ = 0;
  ref_zmp_y_at_this_time_ = 0;
  sum_of_zmp_x_ = 0;
  sum_of_zmp_y_ = 0;
  sum_of_cx_ = 0;
  sum_of_cy_ = 0;

  // variables for balance
  hip_roll_feedforward_angle_rad_ = 0;

  current_right_fx_N_  = current_right_fy_N_  = current_right_fz_N_  = 0;
  current_right_tx_Nm_ = current_right_ty_Nm_ = current_right_tz_Nm_ = 0;
  current_left_fx_N_  = current_left_fy_N_  = current_left_fz_N_  = 0;
  current_left_tx_Nm_ = current_left_ty_Nm_ = current_left_tz_Nm_ = 0;

  current_imu_roll_rad_ = current_imu_pitch_rad_ = 0;
  current_gyro_roll_rad_per_sec_ = current_gyro_pitch_rad_per_sec_ = 0;

  total_mass_of_robot_ = thormang3_kd_->calcTotalMass(0);

  right_dsp_fz_N_ = -1.0*(total_mass_of_robot_)*9.8*0.5;
  right_ssp_fz_N_ = -1.0*(total_mass_of_robot_)*9.8;
  left_dsp_fz_N_  = -1.0*(total_mass_of_robot_)*9.8*0.5;
  left_ssp_fz_N_  = -1.0*(total_mass_of_robot_)*9.8;

  left_fz_trajectory_start_time_ = 0;
  left_fz_trajectory_end_time_  = 0;
  left_fz_trajectory_target_  = left_dsp_fz_N_;
  left_fz_trajectory_shift_   = left_dsp_fz_N_;


  balance_error_ = thormang3::BalanceControlError::NoError;
  quat_current_imu_.w() = cos(0.5*M_PI);
  quat_current_imu_.x() = sin(0.5*M_PI);
  quat_current_imu_.y() = 0;
  quat_current_imu_.z() = 0;
}

THORMANG3OnlineWalking::~THORMANG3OnlineWalking()
  {
}
bool THORMANG3OnlineWalking::setInitialPose(double r_foot_x, double r_foot_y, double r_foot_z, double r_foot_roll, double r_foot_pitch, double r_foot_yaw,
                                            double l_foot_x, double l_foot_y, double l_foot_z, double l_foot_roll, double l_foot_pitch, double l_foot_yaw,
                                            double center_of_body_x, double center_of_body_y, double center_of_body_z,
                                            double center_of_body_roll, double center_of_body_pitch, double center_of_body_yaw)
  {


  if(real_running || ctrl_running)
    return false;

  previous_step_right_foot_pose_.x     = r_foot_x;
  previous_step_right_foot_pose_.y     = r_foot_y;
  previous_step_right_foot_pose_.z     = r_foot_z;
  previous_step_right_foot_pose_.roll  = r_foot_roll;
  previous_step_right_foot_pose_.pitch = r_foot_pitch;
  previous_step_right_foot_pose_.yaw   = r_foot_yaw;

  previous_step_left_foot_pose_.x     = l_foot_x;
  previous_step_left_foot_pose_.y     = l_foot_y;
  previous_step_left_foot_pose_.z     = l_foot_z;
  previous_step_left_foot_pose_.roll  = l_foot_roll;
  previous_step_left_foot_pose_.pitch = l_foot_pitch;
  previous_step_left_foot_pose_.yaw   = l_foot_yaw;

  previous_step_body_pose_.x     = center_of_body_x;
  previous_step_body_pose_.y     = center_of_body_y;
  previous_step_body_pose_.z     = center_of_body_z;
  previous_step_body_pose_.roll  = center_of_body_roll;
  previous_step_body_pose_.pitch = center_of_body_pitch;
  previous_step_body_pose_.yaw   = center_of_body_yaw;

  initial_right_foot_pose_ = previous_step_right_foot_pose_;
  initial_left_foot_pose_  = previous_step_left_foot_pose_;
  initial_body_pose_       = previous_step_body_pose_;

  return true;
}

void THORMANG3OnlineWalking::setInitalWaistYawAngle(double waist_yaw_angle_rad)
  {
  goal_waist_yaw_angle_rad_ = waist_yaw_angle_rad;
}

void THORMANG3OnlineWalking::setInitialRightShoulderAngle(double shoulder_angle_rad)
  {
  r_init_shoulder_angle_rad_ = shoulder_angle_rad;
}

void THORMANG3OnlineWalking::setInitialLeftShoulderAngle(double shoulder_angle_rad)
  {
  l_init_shoulder_angle_rad_ = shoulder_angle_rad;
}

void THORMANG3OnlineWalking::setInitialRightElbowAngle(double elbow_angle_rad)
  {
  r_init_elbow_angle_rad_ = elbow_angle_rad;
}

void THORMANG3OnlineWalking::setInitialLeftElbowAngle(double elbow_angle_rad)
  {
  l_init_elbow_angle_rad_ = elbow_angle_rad;
}

void THORMANG3OnlineWalking::setCurrentIMUSensorOutput(double gyro_x, double gyro_y, double quat_x, double quat_y, double quat_z, double quat_w)
  {
  imu_data_mutex_lock_.lock();

  current_gyro_roll_rad_per_sec_  = gyro_x;
  current_gyro_pitch_rad_per_sec_ = gyro_y;

  quat_current_imu_ = Eigen::Quaterniond(quat_w, quat_x, quat_y, quat_z);

  mat_current_imu_ = (rot_x_pi_3d_ * quat_current_imu_.toRotationMatrix()) * rot_z_pi_3d_;

  current_imu_roll_rad_  = atan2( mat_current_imu_.coeff(2,1), mat_current_imu_.coeff(2,2));
  current_imu_pitch_rad_ = atan2(-mat_current_imu_.coeff(2,0), sqrt(robotis_framework::powDI(mat_current_imu_.coeff(2,1), 2) + robotis_framework::powDI(mat_current_imu_.coeff(2,2), 2)));

  imu_data_mutex_lock_.unlock();
}

void THORMANG3OnlineWalking::initialize()
  {
  if(real_running)
    return;

  step_data_mutex_lock_.lock();
  added_step_data_.clear();

  // initialize balance
  balance_ctrl_.initialize(TIME_UNIT*1000.0);
  balance_ctrl_.setGyroBalanceEnable(true);
  balance_ctrl_.setOrientationBalanceEnable(true);
  balance_ctrl_.setForceTorqueBalanceEnable(true);

  // load balance offset gain
  init_balance_offset_ = false;
  std::string balance_offset_path = ros::package::getPath("thormang3_walking_module") + "/config/balance_offset.yaml";
  parseBalanceOffsetData(balance_offset_path);

  //Initialize Time
  walking_time_ = 0; reference_time_ = 0;

  present_right_foot_pose_ = previous_step_right_foot_pose_;
  present_left_foot_pose_  = previous_step_left_foot_pose_;
  present_body_pose_       = previous_step_body_pose_;

  mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(previous_step_body_pose_.x, previous_step_body_pose_.y, previous_step_body_pose_.z,
      previous_step_body_pose_.roll, previous_step_body_pose_.pitch, previous_step_body_pose_.yaw);

  mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);

  mat_robot_to_cob_ = robotis_framework::getRotation4d(previous_step_body_pose_.roll, previous_step_body_pose_.pitch, 0);
  mat_cob_to_robot_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_);
  mat_g_to_robot_   = mat_g_to_cob_ * mat_cob_to_robot_;
  mat_robot_to_g_   = robotis_framework::getInverseTransformation(mat_g_to_robot_);


  mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(previous_step_right_foot_pose_.x, previous_step_right_foot_pose_.y, previous_step_right_foot_pose_.z,
      previous_step_right_foot_pose_.roll, previous_step_right_foot_pose_.pitch, previous_step_right_foot_pose_.yaw);
  mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(previous_step_left_foot_pose_.x, previous_step_left_foot_pose_.y, previous_step_left_foot_pose_.z,
      previous_step_left_foot_pose_.roll, previous_step_left_foot_pose_.pitch, previous_step_left_foot_pose_.yaw);

  mat_robot_to_rfoot_ = mat_robot_to_g_*mat_g_to_rfoot_;
  mat_robot_to_lfoot_ = mat_robot_to_g_*mat_g_to_lfoot_;

  balance_ctrl_.process(&balance_error_, &mat_robot_to_cob_modified_, &mat_robot_to_rf_modified_, &mat_robot_to_lf_modified_);
  mat_cob_to_robot_modified_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_modified_);

  rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_rf_modified_);
  lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_lf_modified_);

  if(thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw) == false)
  {
    printf("IK not Solved right EPR : %f %f %f %f %f %f\n", rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
    return;
  }

  if(thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw) == false)
  {
    printf("IK not Solved left EPL : %f %f %f %f %f %f\n", lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);
    return;
  }

  for(int angle_idx = 0; angle_idx < 6; angle_idx++)
  {
    out_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx];
    out_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx];
    curr_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx];
    curr_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx];
  }

  for(int feed_forward_idx = 0; feed_forward_idx < 12; feed_forward_idx++)
  {
    leg_angle_feed_back_[feed_forward_idx].p_gain_ = 0;
    leg_angle_feed_back_[feed_forward_idx].d_gain_ = 0;
  }

  reference_step_data_for_addition_.position_data.moving_foot = STANDING;
  reference_step_data_for_addition_.position_data.elbow_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.shoulder_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.foot_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.waist_pitch_angle = 0.0;
  reference_step_data_for_addition_.position_data.waist_yaw_angle = goal_waist_yaw_angle_rad_;
  reference_step_data_for_addition_.position_data.body_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.body_pose = previous_step_body_pose_;
  reference_step_data_for_addition_.position_data.right_foot_pose = previous_step_right_foot_pose_;
  reference_step_data_for_addition_.position_data.left_foot_pose = previous_step_left_foot_pose_;
  reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
  reference_step_data_for_addition_.time_data.abs_step_time = 0.0;
  reference_step_data_for_addition_.time_data.dsp_ratio = 0.2;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_yaw = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_yaw = 0.0;

  present_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;
  previous_step_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;

  current_step_data_status_ = StepDataStatus4;

  // Initialize Matrix for Preview Control
  double t = 0;
  if(TIME_UNIT < 1.0)
    t=TIME_UNIT;
  else
    t=TIME_UNIT/1000.0;

  preview_size_ = round(preview_time_/TIME_UNIT);

  step_idx_data_.resize(preview_size_);
  step_idx_data_.fill(NO_STEP_IDX);
  current_start_idx_for_ref_zmp_ = 0;
  reference_zmp_x_.resize(preview_size_, 1);
  reference_zmp_x_.fill(0.5*(present_right_foot_pose_.x + present_left_foot_pose_.x));
  reference_zmp_y_.resize(preview_size_, 1);
  reference_zmp_y_.fill(0.5*(present_right_foot_pose_.y + present_left_foot_pose_.y));

  A_.resize(3,3); b_.resize(3,1); c_.resize(1,3);
  A_ << 1,  t, t*t/2.0,
      0,  1,   t,
      0,  0,   1;
  b_(0,0) = t*t*t/6.0;
  b_(1,0) =   t*t/2.0;
  b_(2,0) =     t;

  c_(0,0) = 1; c_(0,1) = 0; c_(0,2) = -500.0*0.001/GRAVITY_ACCELERATION;

  k_s_ = 608.900142915471; //500.0
  //k_s_ = 583.938789769793; //600.0

  k_x_.resize(1,3);
  k_x_ << 35904.1790662895, 8609.63092261379, 112.710622775482; // 500
  //k_x_ << 37452.749706802, 9756.07453643388, 121.001738374095; // 600

  f_.resize(1, preview_size_);
  //500
  f_ << 608.9001429,  760.1988656,  921.629618,  1034.769891,   1089.313783,   1096.765451,  1074.295061,  1036.842257,   994.5569472,  953.0295724,
      914.5708045,  879.5752596,  847.5633512,  817.8237132,   789.7293917,   762.8419074,  736.9038288,  711.7889296,  687.4484508,  663.8698037,
      641.050971,   618.987811,   597.6697941,  577.0801943,   557.1979969,   537.9999833,  519.4623366,  501.5616312,  484.2753137,  467.581849,
      451.4606896,  435.8921765,  420.8574331,  406.3382777,   392.3171623,   378.7771307,  365.7017923,  353.0753019,  340.8823445,  329.1081203,
      317.7383294,  306.759157,   296.1572572,  285.919738,    276.0341458,   266.4884502,  257.2710302,  248.3706597,  239.7764944,  231.4780587,
      223.4652335,  215.7282439,  208.2576474,  201.0443231,   194.0794606,   187.3545495,  180.8613691,  174.5919788,  168.5387087,  162.6941501,
      157.0511468,  151.6027868,  146.3423936,  141.2635185,   136.3599326,   131.62562,    127.0547695,  122.6417688,  118.3811969,  114.2678179,
      110.2965751,  106.462584,   102.7611275,   99.18764938,   95.73774938,   92.40717757,  89.19182939,  86.08774068,  83.0910829,   80.19815849,
      77.40539648,  74.70934812,  72.10668274,  69.59418373,   67.16874468,   64.82736558,  62.56714924,  60.38529775,  58.27910913,  56.24597404,
      54.28337262,  52.38887145,  50.5601206,   48.79485075,   47.09087052,   45.44606371,  43.8583868,   42.32586646,  40.84659712,  39.4187387,
      38.04051434,  36.71020825,  35.42616363,  34.18678064,   32.99051448,   31.83587345,  30.72141721,  29.64575495,  28.60754377,  27.60548695,
      26.63833247,  25.70487138,  24.80393641,  23.93440049,   23.09517539,   22.28521038,  21.50349096,  20.74903761,  20.0209046,   19.3181788,
      18.63997859,  17.98545279,  17.35377958,  16.74416551,   16.15584454,   15.58807707,  15.04014906,  14.51137112,  14.00107771,  13.50862627,
      13.03339646,  12.57478939,  12.13222688,  11.70515076,   11.29302214,   10.89532081,  10.51154456,  10.14120854,   9.783844714,  9.439001248,
      9.106241955,  8.78514576,   8.475306179,  8.176330819,   7.887840885,   7.609470721,  7.340867346,  7.081690027,  6.83160985,   6.590309314,
      6.357481938,  6.132831881,  5.916073573,  5.706931359,   5.505139163,   5.310440149,  5.122586408,  4.941338646,  4.766465888,  4.597745188,
      4.434961356,  4.277906685,  4.126380694,  3.980189879,   3.839147471,   3.703073201,  3.571793078,  3.445139169,  3.322949391,  3.205067309,
      3.091341936,  2.981627549,  2.875783507,  2.773674068,   2.675168228,   2.58013955,   2.488466009,  2.400029838,  2.314717381,  2.232418947,
      2.153028678,  2.076444411,  2.002567552,  1.931302952,   1.862558786,   1.796246438,  1.732280393,  1.670578121,  1.611059983,  1.553649123,
      1.498271374,  1.444855165,  1.393331431,  1.343633522,   1.295697125,   1.249460178,  1.204862791,  1.161847176,  1.120357568,  1.080340156;

  //600
//  f_ << 583.9387898,  750.9225137,  918.1585887,  1025.28224,  1068.572334,  1066.00235,   1038.071691,  1000.083662,   960.8853819,  924.332612,
//        891.3091699,  861.3465762,  833.615981,   807.3964237,  782.2132154,  757.8151538,  734.0999123,  711.0427972,  688.6479404,  666.9221777,
//        645.8649499,  625.4670304,  605.7128357,  586.5833441,  568.0583614,  550.1178678,  532.7426248,  515.914327,   499.6155377,  483.8295609,
//        468.5403224,  453.7322864,  439.3904039,  425.5000849,  412.0471825,  399.0179825,  386.3991934,  374.1779373,  362.3417377,  350.8785086,
//        339.7765423,  329.0244973,  318.6113874,  308.5265697,  298.7597346,  289.3008946,  280.140375,   271.2688034,  262.6771012,  254.3564733,
//        246.2984005,  238.4946298,  230.9371668,  223.6182676,  216.5304305,  209.6663889,  203.019104,   196.5817571,  190.3477436,  184.3106657,
//        178.464326,   172.8027215,  167.3200373,  162.010641,   156.8690766,  151.8900593,  147.0684701,  142.3993506,  137.8778978,  133.4994595,
//        129.2595297,  125.1537435,  121.1778731,  117.3278237,  113.5996286,  109.9894461,  106.4935549,  103.1083507,   99.83034228,  96.6561483,
//         93.58249356,  90.6062058,   87.72421248,  84.93353763, 82.23129884,   79.61470434,  77.08105014,  74.62771731,  72.25216927,  69.95194925,
//         67.72467778,  65.56805026,  63.47983461,  61.45786901, 59.50005969,   57.6043788,   55.76886233,  53.99160812,  52.27077392,  50.60457551,
//         48.99128487,  47.42922844,  45.91678537,  44.4523859,  43.03450975,   41.66168457,  40.33248441,  39.04552829,  37.79947877,  36.5930406,
//         35.42495939,  34.29402031,  33.19904685,  32.13889964, 31.11247526,   30.11870513,  29.15655437,  28.22502079,  27.32313386,  26.44995365,
//         25.60456997,  24.78610134,  23.99369414,  23.22652172, 22.48378355,   21.76470439,  21.06853351,  20.39454392,  19.74203158,  19.11031475,
//         18.49873322,  17.90664768,  17.33343901,  16.77850772, 16.24127324,   15.7211734,   15.21766381,  14.73021731,  14.25832344,  13.80148787,
//         13.35923194,  12.93109216,  12.51661969,  12.11537993, 11.72695202,   11.35092848,  10.98691468,  10.63452855,  10.29340009,   9.963171053,
//          9.643494539,  9.334034646,  9.034466125,  8.74447404,  8.463753446,   8.19200907,   7.928955006,  7.674314421,  7.427819265,  7.189209996,
//          6.958235311,  6.734651883,  6.518224112,  6.308723877, 6.105930303,   5.909629529,  5.719614489,  5.535684693,  5.357646024,  5.18531053,
//          5.018496236,  4.857026947,  4.700732073,  4.549446444, 4.403010145,   4.261268345,  4.124071137,  3.991273383,  3.862734561,  3.738318623,
//          3.617893846,  3.501332704,  3.388511725,  3.27931137,  3.173615908,   3.071313287,  2.97229503,   2.87645611,   2.783694848,  2.693912803,
//          2.607014671,  2.522908184,  2.441504017,  2.362715689, 2.286459477,   2.212654328,  2.141221772,  2.072085843,  2.005172996,  1.940412033;

  u_x.resize(1,1);
  u_y.resize(1,1);

  x_lipm_.resize(3, 1);    y_lipm_.resize(3, 1);
  x_lipm_.fill(0.0);       y_lipm_.fill(0.0);

  step_data_mutex_lock_.unlock();

  r_shoulder_out_angle_rad_ = r_shoulder_dir_*(mat_robot_to_rfoot_.coeff(0, 3) - mat_robot_to_lfoot_.coeff(0, 3))*shouler_swing_gain_ + r_init_shoulder_angle_rad_;
  l_shoulder_out_angle_rad_ = l_shoulder_dir_*(mat_robot_to_lfoot_.coeff(0, 3) - mat_robot_to_rfoot_.coeff(0, 3))*shouler_swing_gain_ + l_init_shoulder_angle_rad_;
  r_elbow_out_angle_rad_ = r_elbow_dir_*(mat_robot_to_rfoot_.coeff(0, 3) - mat_robot_to_lfoot_.coeff(0, 3))*elbow_swing_gain_ + r_init_elbow_angle_rad_;
  l_elbow_out_angle_rad_ = l_elbow_dir_*(mat_robot_to_lfoot_.coeff(0, 3) - mat_robot_to_rfoot_.coeff(0, 3))*elbow_swing_gain_ + l_init_elbow_angle_rad_;

  left_fz_trajectory_start_time_ = 0;
  left_fz_trajectory_end_time_  = 0;
  left_fz_trajectory_target_  = left_dsp_fz_N_;
  left_fz_trajectory_shift_   = left_dsp_fz_N_;

}

void THORMANG3OnlineWalking::reInitialize()
  {
  if(real_running)
    return;

  step_data_mutex_lock_.lock();
  added_step_data_.clear();

  //Initialize Time
  walking_time_ = 0; reference_time_ = 0;

  previous_step_right_foot_pose_ = robotis_framework::getPose3DfromTransformMatrix(mat_robot_to_rfoot_);
  previous_step_left_foot_pose_  = robotis_framework::getPose3DfromTransformMatrix(mat_robot_to_lfoot_);
  previous_step_body_pose_       = robotis_framework::getPose3DfromTransformMatrix(mat_robot_to_cob_);

  present_right_foot_pose_ = previous_step_right_foot_pose_;
  present_left_foot_pose_  = previous_step_left_foot_pose_;
  present_body_pose_       = previous_step_body_pose_;

  mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(previous_step_body_pose_.x, previous_step_body_pose_.y, previous_step_body_pose_.z,
      previous_step_body_pose_.roll, previous_step_body_pose_.pitch, previous_step_body_pose_.yaw);

  mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);
  mat_g_to_robot_ = mat_g_to_cob_ * mat_cob_to_robot_;
  mat_robot_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_robot_);

  mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(previous_step_right_foot_pose_.x, previous_step_right_foot_pose_.y, previous_step_right_foot_pose_.z,
      previous_step_right_foot_pose_.roll, previous_step_right_foot_pose_.pitch, previous_step_right_foot_pose_.yaw);
  mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(previous_step_left_foot_pose_.x, previous_step_left_foot_pose_.y, previous_step_left_foot_pose_.z,
      previous_step_left_foot_pose_.roll, previous_step_left_foot_pose_.pitch, previous_step_left_foot_pose_.yaw);

  mat_robot_to_rfoot_ = mat_rhip_to_cob_*mat_cob_to_g_*mat_g_to_rfoot_;
  mat_robot_to_lfoot_ = mat_lhip_to_cob_*mat_cob_to_g_*mat_g_to_lfoot_;

  balance_ctrl_.process(&balance_error_, &mat_robot_to_cob_modified_, &mat_robot_to_rf_modified_, &mat_robot_to_lf_modified_);
  mat_cob_to_robot_modified_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_modified_);

  rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_rf_modified_);
  lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_lf_modified_);

  if(thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw) == false)
  {
    printf("IK not Solved rr EPR : %f %f %f %f %f %f\n", rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
    return;
  }

  if(thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw) == false)
  {
    printf("IK not Solved rl EPL : %f %f %f %f %f %f\n", lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);
    return;
  }

  for(int angle_idx = 0; angle_idx < 6; angle_idx++)
  {
    out_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx];
    out_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx];
  }

  reference_step_data_for_addition_.position_data.moving_foot = STANDING;
  reference_step_data_for_addition_.position_data.elbow_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.shoulder_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.foot_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.waist_pitch_angle = 0.0;
  reference_step_data_for_addition_.position_data.waist_yaw_angle = goal_waist_yaw_angle_rad_;
  reference_step_data_for_addition_.position_data.body_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.body_pose = previous_step_body_pose_;
  reference_step_data_for_addition_.position_data.right_foot_pose = previous_step_right_foot_pose_;
  reference_step_data_for_addition_.position_data.left_foot_pose = previous_step_left_foot_pose_;
  reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
  reference_step_data_for_addition_.time_data.abs_step_time = 0.0;
  reference_step_data_for_addition_.time_data.dsp_ratio = 0.2;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_yaw = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_yaw = 0.0;

  present_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;
  previous_step_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;

  current_step_data_status_ = StepDataStatus4;

  step_idx_data_.fill(NO_STEP_IDX);
  current_start_idx_for_ref_zmp_ = 0;
  reference_zmp_x_.fill(0.5*(present_right_foot_pose_.x + present_left_foot_pose_.x));
  reference_zmp_y_.fill(0.5*(present_right_foot_pose_.y + present_left_foot_pose_.y));
  sum_of_zmp_x_ = 0.0;
  sum_of_zmp_y_ = 0.0;

  sum_of_cx_ = 0.0;
  sum_of_cy_ = 0.0;
  x_lipm_.fill(0.0);        y_lipm_.fill(0.0);

  step_data_mutex_lock_.unlock();

  left_fz_trajectory_start_time_ = 0;
  left_fz_trajectory_end_time_  = 0;
  left_fz_trajectory_target_  = left_dsp_fz_N_;
  left_fz_trajectory_shift_   = left_dsp_fz_N_;
}

void THORMANG3OnlineWalking::start()
  {
  ctrl_running = true;
  real_running = true;
}

void THORMANG3OnlineWalking::stop()
  {
  ctrl_running = false;
}

bool THORMANG3OnlineWalking::isRunning()
  {
  return real_running;
}

bool THORMANG3OnlineWalking::addStepData(robotis_framework::StepData step_data)
  {
  step_data_mutex_lock_.lock();
  added_step_data_.push_back(step_data);

  calcStepIdxData();
  step_data_mutex_lock_.unlock();

  return true;
}

int THORMANG3OnlineWalking::getNumofRemainingUnreservedStepData()
  {
  int step_idx = step_idx_data_(preview_size_ - 1);
  int remain_step_num = 0;
  if(step_idx != NO_STEP_IDX)
  {
    remain_step_num = (added_step_data_.size() - 1 - step_idx);
  }
  else
  {
    remain_step_num = 0;
  }
  return remain_step_num;
}

void THORMANG3OnlineWalking::eraseLastStepData()
  {
  step_data_mutex_lock_.lock();
  if(getNumofRemainingUnreservedStepData() != 0)
  {
    added_step_data_.pop_back();
  }
  step_data_mutex_lock_.unlock();
}

void THORMANG3OnlineWalking::getReferenceStepDatafotAddition(robotis_framework::StepData *ref_step_data_for_addition)
  {
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.start_time_delay_ratio_yaw = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_x = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_y = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_z = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_roll = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_pitch = 0.0;
  reference_step_data_for_addition_.time_data.finish_time_advance_ratio_yaw = 0.0;
  (*ref_step_data_for_addition) = reference_step_data_for_addition_;
}

void THORMANG3OnlineWalking::calcStepIdxData()
  {
  unsigned int step_idx = 0, previous_step_idx = 0;
  unsigned int step_data_size = added_step_data_.size();
  if(added_step_data_.size() == 0)
  {
    step_idx_data_.fill(NO_STEP_IDX);
    current_step_data_status_ = StepDataStatus4;
    real_running = false;
  }
  else
  {
    if(walking_time_ >= added_step_data_[0].time_data.abs_step_time - 0.5*MStoS)
    {
      previous_step_waist_yaw_angle_rad_ = added_step_data_[0].position_data.waist_yaw_angle;
      previous_step_left_foot_pose_ = added_step_data_[0].position_data.left_foot_pose;
      previous_step_right_foot_pose_ = added_step_data_[0].position_data.right_foot_pose;
      previous_step_body_pose_ = added_step_data_[0].position_data.body_pose;
      previous_step_body_pose_.x = present_body_pose_.x;
      previous_step_body_pose_.y = present_body_pose_.y;
      reference_time_ = added_step_data_[0].time_data.abs_step_time;
      added_step_data_.erase(added_step_data_.begin());
      if(added_step_data_.size() == 0)
      {
        step_idx_data_.fill(NO_STEP_IDX);
        current_step_data_status_ = StepDataStatus4;
        real_running = false;
      }
      else
      {
        for(int idx = 0; idx < preview_size_; idx++)
        {
          //Get STepIDx
          if(walking_time_ + (idx+1)*TIME_UNIT > added_step_data_[step_data_size -1].time_data.abs_step_time)
            step_idx_data_(idx) = NO_STEP_IDX;
          else
          {
            for(step_idx = previous_step_idx; step_idx < step_data_size; step_idx++)
            {
              if(walking_time_ + (idx+1)*TIME_UNIT <= added_step_data_[step_idx].time_data.abs_step_time)
                break;
            }
            step_idx_data_(idx) = step_idx;
            previous_step_idx = step_idx;
          }
        }
      }
    }
    else
    {
      for(int idx = 0; idx < preview_size_; idx++)
      {
        //Get StepIdx
        if(walking_time_ + (idx+1)*TIME_UNIT > added_step_data_[step_data_size -1].time_data.abs_step_time)
          step_idx_data_(idx) = NO_STEP_IDX;
        else
        {
          for(step_idx = previous_step_idx; step_idx < step_data_size; step_idx++)
          {
            if(walking_time_ + (idx+1)*TIME_UNIT <= added_step_data_[step_idx].time_data.abs_step_time)
              break;
          }
          step_idx_data_(idx) = step_idx;
          previous_step_idx = step_idx;
        }
      }
    }
  }

  if(step_idx_data_(preview_size_ - 1) != NO_STEP_IDX)
  {
    if(getNumofRemainingUnreservedStepData() != 0)
    {
      current_step_data_status_ = StepDataStatus1;
      reference_step_data_for_addition_ = added_step_data_[step_idx_data_(preview_size_-1)];
    }
    else
    {
      current_step_data_status_ = StepDataStatus2;
      reference_step_data_for_addition_ = added_step_data_[step_idx_data_(preview_size_-1)];
    }
  }
  else
  {
    if(step_idx_data_(0) != NO_STEP_IDX)
    {
      reference_step_data_for_addition_ = added_step_data_[step_idx_data_(0)];
      reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
      reference_step_data_for_addition_.time_data.abs_step_time += preview_time_;

      current_step_data_status_ = StepDataStatus3;
    }
    else
    {
      reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
      reference_step_data_for_addition_.time_data.abs_step_time = walking_time_ + preview_time_;
      current_step_data_status_ = StepDataStatus4;
    }
  }
}

void THORMANG3OnlineWalking::calcRefZMP()
  {
  int ref_zmp_idx = 0;
  int step_idx = 0;
  if(walking_time_ == 0)
  {
    if((step_idx_data_(ref_zmp_idx) == NO_STEP_IDX)/* && (m_StepData.size() == 0)*/)
    {
      reference_zmp_x_.fill((present_left_foot_pose_.x + present_right_foot_pose_.x)*0.5);
      reference_zmp_y_.fill((present_left_foot_pose_.y + present_right_foot_pose_.y)*0.5);
      return;
    }

    for(ref_zmp_idx = 0; ref_zmp_idx < preview_size_;  ref_zmp_idx++)
    {
      step_idx = step_idx_data_(ref_zmp_idx);
      if(step_idx == NO_STEP_IDX)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = reference_zmp_x_(ref_zmp_idx - 1, 0);
        reference_zmp_y_(ref_zmp_idx, 0) = reference_zmp_y_(ref_zmp_idx - 1, 0);
      }
      else
      {
        if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING)
        {
          if( added_step_data_[step_idx].position_data.moving_foot == RIGHT_FOOT_SWING )
          {
            reference_zmp_x_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.left_foot_pose.x;
            reference_zmp_y_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.left_foot_pose.y;
          }
          else if( added_step_data_[step_idx].position_data.moving_foot == LEFT_FOOT_SWING )
          {
            reference_zmp_x_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.right_foot_pose.x;
            reference_zmp_y_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.right_foot_pose.y;
          }
          else if( added_step_data_[step_idx].position_data.moving_foot == STANDING )
          {
            reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
            reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
          }
          else
          {
            reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
            reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
          }
        }
        else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_STARTING)
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_ENDING)
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
      }
    }
    current_start_idx_for_ref_zmp_ = 0;
  }
  else
  {
    step_idx = step_idx_data_(preview_size_ - 1);

    if(current_start_idx_for_ref_zmp_ == 0)
      ref_zmp_idx = preview_size_ - 1;
    else
      ref_zmp_idx = current_start_idx_for_ref_zmp_ - 1;

    if(step_idx == NO_STEP_IDX)
    {
      reference_zmp_x_(ref_zmp_idx, 0) = 0.5*(reference_step_data_for_addition_.position_data.right_foot_pose.x + reference_step_data_for_addition_.position_data.left_foot_pose.x);
      reference_zmp_y_(ref_zmp_idx, 0) = 0.5*(reference_step_data_for_addition_.position_data.right_foot_pose.y + reference_step_data_for_addition_.position_data.left_foot_pose.y);
    }
    else
    {
      if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING)
      {
        if( added_step_data_[step_idx].position_data.moving_foot == RIGHT_FOOT_SWING )
        {
          reference_zmp_x_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.left_foot_pose.x;
          reference_zmp_y_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.left_foot_pose.y*0.2;
        }
        else if( added_step_data_[step_idx].position_data.moving_foot == LEFT_FOOT_SWING )
        {
          reference_zmp_x_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.right_foot_pose.x;
          reference_zmp_y_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.right_foot_pose.y*0.2;
        }
        else if( added_step_data_[step_idx].position_data.moving_foot == STANDING )
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
      }
      else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_STARTING)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
      else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_ENDING)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
      else
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
    }
  }
}

void THORMANG3OnlineWalking::calcDesiredPose()
  {
  ////Original LIPM
  //u_x = -K*x_LIPM + x_feed_forward_term;
  //x_LIPM = A*x_LIPM + b*u_x;
  //
  //u_y = -K*y_LIPM + y_feed_forward_term;
  //y_LIPM = A*y_LIPM + b*u_y;

  //Calc LIPM with Integral
  Eigen::MatrixXd  x_feed_forward_term2; //f_Preview*m_ZMP_Reference_X;
  Eigen::MatrixXd  y_feed_forward_term2; //f_Preview*m_ZMP_Reference_Y;

  x_feed_forward_term2.resize(1,1);
  x_feed_forward_term2.fill(0.0);
  y_feed_forward_term2.resize(1,1);
  y_feed_forward_term2.fill(0.0);

  for(int i = 0; i < preview_size_; i++)
  {
    if(current_start_idx_for_ref_zmp_ + i < preview_size_) {
      x_feed_forward_term2(0,0) += f_(i)*reference_zmp_x_(current_start_idx_for_ref_zmp_ + i, 0);
      y_feed_forward_term2(0,0) += f_(i)*reference_zmp_y_(current_start_idx_for_ref_zmp_ + i, 0);
    }
    else {
      x_feed_forward_term2(0,0) += f_(i)*reference_zmp_x_(current_start_idx_for_ref_zmp_ + i - preview_size_, 0);
      y_feed_forward_term2(0,0) += f_(i)*reference_zmp_y_(current_start_idx_for_ref_zmp_ + i - preview_size_, 0);
    }
  }

  sum_of_cx_ += c_(0,0)*x_lipm_(0,0) +  c_(0,1)*x_lipm_(1,0) +  c_(0,2)*x_lipm_(2,0);
  sum_of_cy_ += c_(0,0)*y_lipm_(0,0) +  c_(0,1)*y_lipm_(1,0) +  c_(0,2)*y_lipm_(2,0);

  u_x(0,0) = -k_s_*(sum_of_cx_ - sum_of_zmp_x_) - (k_x_(0,0)*x_lipm_(0,0) + k_x_(0,1)*x_lipm_(1,0) + k_x_(0,2)*x_lipm_(2,0)) + x_feed_forward_term2(0,0);
  u_y(0,0) = -k_s_*(sum_of_cy_ - sum_of_zmp_y_) - (k_x_(0,0)*y_lipm_(0,0) + k_x_(0,1)*y_lipm_(1,0) + k_x_(0,2)*y_lipm_(2,0)) + y_feed_forward_term2(0,0);
  x_lipm_ = A_*x_lipm_ + b_*u_x;
  y_lipm_ = A_*y_lipm_ + b_*u_y;


  ref_zmp_x_at_this_time_ = reference_zmp_x_.coeff(current_start_idx_for_ref_zmp_, 0);
  ref_zmp_y_at_this_time_ = reference_zmp_y_.coeff(current_start_idx_for_ref_zmp_, 0);

  sum_of_zmp_x_ += reference_zmp_x_.coeff(current_start_idx_for_ref_zmp_, 0);
  sum_of_zmp_y_ += reference_zmp_y_.coeff(current_start_idx_for_ref_zmp_, 0);

  present_body_pose_.x = x_lipm_.coeff(0,0);
  present_body_pose_.y = y_lipm_.coeff(0,0);

  reference_step_data_for_addition_.position_data.body_pose.x = x_lipm_(0,0);
  reference_step_data_for_addition_.position_data.body_pose.y = y_lipm_(0,0);

  current_start_idx_for_ref_zmp_++;
  if(current_start_idx_for_ref_zmp_ == (preview_size_))
    current_start_idx_for_ref_zmp_ = 0;
}

void THORMANG3OnlineWalking::process()
  {
  if(!ctrl_running)
  {
    return;
  }
  else
  {
    step_data_mutex_lock_.lock();

    calcStepIdxData();
    calcRefZMP();
    calcDesiredPose();

    double hip_roll_swap = 0;

    if((added_step_data_.size() != 0) && real_running)
    {
      double period_time, dsp_ratio, ssp_ratio, foot_move_period_time, ssp_time_start, ssp_time_end;
      period_time = added_step_data_[0].time_data.abs_step_time - reference_time_;
      dsp_ratio = added_step_data_[0].time_data.dsp_ratio;
      ssp_ratio = 1 - dsp_ratio;
      foot_move_period_time = ssp_ratio*period_time;

      ssp_time_start = dsp_ratio*period_time/2.0 + reference_time_;
      ssp_time_end = (1 + ssp_ratio)*period_time / 2.0 + reference_time_;

      double start_time_delay_ratio_x        = added_step_data_[0].time_data.start_time_delay_ratio_x;
      double start_time_delay_ratio_y        = added_step_data_[0].time_data.start_time_delay_ratio_y;
      double start_time_delay_ratio_z        = added_step_data_[0].time_data.start_time_delay_ratio_z;
      double start_time_delay_ratio_roll     = added_step_data_[0].time_data.start_time_delay_ratio_roll;
      double start_time_delay_ratio_pitch    = added_step_data_[0].time_data.start_time_delay_ratio_pitch;
      double start_time_delay_ratio_yaw      = added_step_data_[0].time_data.start_time_delay_ratio_yaw;
      double finish_time_advance_ratio_x     = added_step_data_[0].time_data.finish_time_advance_ratio_x;
      double finish_time_advance_ratio_y     = added_step_data_[0].time_data.finish_time_advance_ratio_y;
      double finish_time_advance_ratio_z     = added_step_data_[0].time_data.finish_time_advance_ratio_z;
      double finish_time_advance_ratio_roll  = added_step_data_[0].time_data.finish_time_advance_ratio_roll;
      double finish_time_advance_ratio_pitch = added_step_data_[0].time_data.finish_time_advance_ratio_pitch;
      double finish_time_advance_ratio_yaw   = added_step_data_[0].time_data.finish_time_advance_ratio_yaw;

      double hip_roll_swap_dir = 1.0;

      if( (walking_time_ - reference_time_) < TIME_UNIT)
      {
        waist_yaw_tra_.changeTrajectory(reference_time_, previous_step_waist_yaw_angle_rad_, 0, 0,
            added_step_data_[0].time_data.abs_step_time, added_step_data_[0].position_data.waist_yaw_angle, 0, 0);
        body_z_tra_.changeTrajectory(reference_time_, previous_step_body_pose_.z, 0, 0,
            added_step_data_[0].time_data.abs_step_time, added_step_data_[0].position_data.body_pose.z, 0, 0);
        body_roll_tra_.changeTrajectory(reference_time_, previous_step_body_pose_.roll, 0, 0,
            added_step_data_[0].time_data.abs_step_time, added_step_data_[0].position_data.body_pose.roll, 0, 0);
        body_pitch_tra_.changeTrajectory(reference_time_, previous_step_body_pose_.pitch, 0, 0,
            added_step_data_[0].time_data.abs_step_time, added_step_data_[0].position_data.body_pose.pitch, 0, 0);

        double bc_move_amp = added_step_data_[0].position_data.body_pose.yaw - previous_step_body_pose_.yaw;

        if(bc_move_amp >= M_PI)
          bc_move_amp -= 2.0*M_PI;
        else if(bc_move_amp <= -M_PI)
          bc_move_amp += 2.0*M_PI;

        body_yaw_tra_.changeTrajectory(reference_time_, previous_step_body_pose_.yaw, 0, 0,
            added_step_data_[0].time_data.abs_step_time, bc_move_amp + previous_step_body_pose_.yaw, 0, 0);

        body_z_swap_tra_.changeTrajectory(reference_time_,
            0, 0, 0,
            0.5*(added_step_data_[0].time_data.abs_step_time + reference_time_),
            added_step_data_[0].position_data.body_z_swap, 0, 0);

        if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
        {
          foot_x_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_x*foot_move_period_time,
              previous_step_right_foot_pose_.x, 0, 0,
              ssp_time_end - finish_time_advance_ratio_x*foot_move_period_time,
              added_step_data_[0].position_data.right_foot_pose.x, 0, 0);
          foot_y_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_y*foot_move_period_time,
              previous_step_right_foot_pose_.y, 0, 0,
              ssp_time_end - finish_time_advance_ratio_y*foot_move_period_time,
              added_step_data_[0].position_data.right_foot_pose.y, 0, 0);
          foot_z_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_z*foot_move_period_time,
              previous_step_right_foot_pose_.z, 0, 0,
              ssp_time_end - finish_time_advance_ratio_z*foot_move_period_time,
              added_step_data_[0].position_data.right_foot_pose.z, 0, 0);
          foot_roll_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_roll*foot_move_period_time,
              previous_step_right_foot_pose_.roll, 0, 0,
              ssp_time_end - finish_time_advance_ratio_roll*foot_move_period_time,
              added_step_data_[0].position_data.right_foot_pose.roll, 0, 0);
          foot_pitch_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_pitch*foot_move_period_time,
              previous_step_right_foot_pose_.pitch, 0, 0,
              ssp_time_end - finish_time_advance_ratio_pitch*foot_move_period_time,
              added_step_data_[0].position_data.right_foot_pose.pitch, 0, 0);

          double c_move_amp = added_step_data_[0].position_data.right_foot_pose.yaw - previous_step_right_foot_pose_.yaw;
          if(c_move_amp >= M_PI)
            c_move_amp -= 2.0*M_PI;
          else if(c_move_amp <= -M_PI)
            c_move_amp += 2.0*M_PI;

          foot_yaw_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_yaw*foot_move_period_time,
              previous_step_right_foot_pose_.yaw, 0, 0,
              ssp_time_end - finish_time_advance_ratio_yaw*foot_move_period_time,
              c_move_amp + previous_step_right_foot_pose_.yaw, 0, 0);

          foot_z_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
              0.5*(ssp_time_start + ssp_time_end), added_step_data_[0].position_data.foot_z_swap, 0, 0);

          hip_roll_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
              0.5*(ssp_time_start + ssp_time_end), hip_roll_feedforward_angle_rad_, 0, 0);
        }
        else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
        {
          foot_x_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_x*foot_move_period_time,
              previous_step_left_foot_pose_.x, 0, 0,
              ssp_time_end - finish_time_advance_ratio_x*foot_move_period_time,
              added_step_data_[0].position_data.left_foot_pose.x, 0, 0);
          foot_y_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_y*foot_move_period_time,
              previous_step_left_foot_pose_.y, 0, 0,
              ssp_time_end - finish_time_advance_ratio_y*foot_move_period_time,
              added_step_data_[0].position_data.left_foot_pose.y, 0, 0);
          foot_z_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_z*foot_move_period_time,
              previous_step_left_foot_pose_.z, 0, 0,
              ssp_time_end - finish_time_advance_ratio_z*foot_move_period_time,
              added_step_data_[0].position_data.left_foot_pose.z, 0, 0);
          foot_roll_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_roll*foot_move_period_time,
              previous_step_left_foot_pose_.roll, 0, 0,
              ssp_time_end - finish_time_advance_ratio_roll*foot_move_period_time,
              added_step_data_[0].position_data.left_foot_pose.roll, 0, 0);
          foot_pitch_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_pitch*foot_move_period_time,
              previous_step_left_foot_pose_.pitch, 0, 0,
              ssp_time_end - finish_time_advance_ratio_pitch*foot_move_period_time,
              added_step_data_[0].position_data.left_foot_pose.pitch, 0, 0);

          double c_move_amp = added_step_data_[0].position_data.left_foot_pose.yaw - previous_step_left_foot_pose_.yaw;
          if(c_move_amp >= M_PI)
            c_move_amp -= 2.0*M_PI;
          else if(c_move_amp <= -M_PI)
            c_move_amp += 2.0*M_PI;

          foot_yaw_tra_.changeTrajectory(ssp_time_start + start_time_delay_ratio_yaw*foot_move_period_time,
              previous_step_left_foot_pose_.yaw, 0, 0,
              ssp_time_end - finish_time_advance_ratio_yaw*foot_move_period_time,
              c_move_amp + previous_step_left_foot_pose_.yaw, 0, 0);

          foot_z_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
              0.5*(ssp_time_start + ssp_time_end), added_step_data_[0].position_data.foot_z_swap, 0, 0);

          hip_roll_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
              0.5*(ssp_time_start + ssp_time_end), hip_roll_feedforward_angle_rad_, 0, 0);

          hip_roll_swap_dir = 1.0;
        }
        else
        {
          foot_z_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
              ssp_time_end, 0, 0, 0);

          hip_roll_swap_tra_.changeTrajectory(ssp_time_start, 0, 0, 0,
              ssp_time_end, 0, 0, 0);

          hip_roll_swap_dir = 0.0;
        }
      }

      double z_swap = body_z_swap_tra_.getPosition(walking_time_);
      double wp_move = waist_yaw_tra_.getPosition(walking_time_);
      double bz_move = body_z_tra_.getPosition(walking_time_);
      double ba_move = body_roll_tra_.getPosition(walking_time_);
      double bb_move = body_pitch_tra_.getPosition(walking_time_);
      double bc_move = body_yaw_tra_.getPosition(walking_time_);

      present_waist_yaw_angle_rad_ = wp_move;
      present_body_pose_.z = bz_move + z_swap;
      present_body_pose_.roll = ba_move;
      present_body_pose_.pitch = bb_move;
      present_body_pose_.yaw = bc_move;

      //Feet
      double x_move, y_move, z_move, a_move, b_move, c_move, z_vibe;
      if( walking_time_ <= ssp_time_start)
      {
        x_move = foot_x_tra_.getPosition(ssp_time_start);
        y_move = foot_y_tra_.getPosition(ssp_time_start);
        z_move = foot_z_tra_.getPosition(ssp_time_start);
        a_move = foot_roll_tra_.getPosition(ssp_time_start);
        b_move = foot_pitch_tra_.getPosition(ssp_time_start);
        c_move = foot_yaw_tra_.getPosition(ssp_time_start);

        z_vibe = foot_z_swap_tra_.getPosition(ssp_time_start);
        hip_roll_swap = hip_roll_swap_tra_.getPosition(ssp_time_start);

        if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
          balancing_index_ = BalancingPhase1;
        else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
          balancing_index_ = BalancingPhase5;
        else
          balancing_index_ = BalancingPhase0;
      }
      else if( walking_time_ <= ssp_time_end)
      {
        x_move = foot_x_tra_.getPosition(walking_time_);
        y_move = foot_y_tra_.getPosition(walking_time_);
        z_move = foot_z_tra_.getPosition(walking_time_);
        a_move = foot_roll_tra_.getPosition(walking_time_);
        b_move = foot_pitch_tra_.getPosition(walking_time_);
        c_move = foot_yaw_tra_.getPosition(walking_time_);

        if(added_step_data_[0].position_data.moving_foot != STANDING)
        {
          if((walking_time_ >= 0.5*(ssp_time_start + ssp_time_end))
              && (walking_time_ < (0.5*(ssp_time_start + ssp_time_end) + TIME_UNIT)))
          {
            body_z_swap_tra_.changeTrajectory(0.5*(added_step_data_[0].time_data.abs_step_time + reference_time_),
                added_step_data_[0].position_data.body_z_swap, 0, 0,
                added_step_data_[0].time_data.abs_step_time,
                0, 0, 0);

            foot_z_swap_tra_.changeTrajectory(0.5*(ssp_time_start + ssp_time_end),
                added_step_data_[0].position_data.foot_z_swap, 0, 0,
                ssp_time_end, 0, 0, 0);

            hip_roll_swap_tra_.changeTrajectory(0.5*(ssp_time_start + ssp_time_end),
                hip_roll_feedforward_angle_rad_, 0, 0,
                ssp_time_end, 0, 0, 0);
          }
        }

        z_vibe = foot_z_swap_tra_.getPosition(walking_time_);
        hip_roll_swap = hip_roll_swap_tra_.getPosition(walking_time_);

        if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
        {
          if(walking_time_ <= (ssp_time_end + ssp_time_start)*0.5)
            balancing_index_ = BalancingPhase2;
          else
            balancing_index_ = BalancingPhase3;
        }
        else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
        {
          if(walking_time_ <= (ssp_time_end + ssp_time_start)*0.5)
            balancing_index_ = BalancingPhase6;
          else
            balancing_index_ = BalancingPhase7;
        }
        else
          balancing_index_ = BalancingPhase0;
      }
      else
      {
        x_move = foot_x_tra_.getPosition(ssp_time_end);
        y_move = foot_y_tra_.getPosition(ssp_time_end);
        z_move = foot_z_tra_.getPosition(ssp_time_end);
        a_move = foot_roll_tra_.getPosition(ssp_time_end);
        b_move = foot_pitch_tra_.getPosition(ssp_time_end);
        c_move = foot_yaw_tra_.getPosition(ssp_time_end);

        z_vibe = foot_z_swap_tra_.getPosition(ssp_time_end);
        hip_roll_swap = hip_roll_swap_tra_.getPosition(ssp_time_end);

        if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
          balancing_index_ = BalancingPhase4;
        else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
          balancing_index_ = BalancingPhase8;
        else
          balancing_index_ = BalancingPhase0;
      }


      if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
      {
        present_right_foot_pose_.x = x_move;
        present_right_foot_pose_.y = y_move;
        present_right_foot_pose_.z = z_move + z_vibe;
        present_right_foot_pose_.roll = a_move;
        present_right_foot_pose_.pitch = b_move;
        present_right_foot_pose_.yaw = c_move;

        present_left_foot_pose_ = added_step_data_[0].position_data.left_foot_pose;

        hip_roll_swap_dir = -1.0;
      }
      else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
      {
        present_right_foot_pose_ = added_step_data_[0].position_data.right_foot_pose;

        present_left_foot_pose_.x = x_move;
        present_left_foot_pose_.y = y_move;
        present_left_foot_pose_.z = z_move + z_vibe;
        present_left_foot_pose_.roll = a_move;
        present_left_foot_pose_.pitch = b_move;
        present_left_foot_pose_.yaw = c_move;

        hip_roll_swap_dir = 1.0;
      }
      else
      {
        present_right_foot_pose_ = added_step_data_[0].position_data.right_foot_pose;
        present_left_foot_pose_ = added_step_data_[0].position_data.left_foot_pose;

        hip_roll_swap_dir = 0.0;
      }

      hip_roll_swap = hip_roll_swap_dir * hip_roll_swap;

      shouler_swing_gain_ = added_step_data_[0].position_data.shoulder_swing_gain;
      elbow_swing_gain_ = added_step_data_[0].position_data.elbow_swing_gain;

      walking_time_ += TIME_UNIT;

      if(walking_time_ > added_step_data_[added_step_data_.size() - 1].time_data.abs_step_time - 0.5*0.001)
      {
        real_running = false;
        calcStepIdxData();
        step_data_mutex_lock_.unlock();
        reInitialize();
      }

      if(balancing_index_ == BalancingPhase0 || balancing_index_ == BalancingPhase9)
      {
        left_fz_trajectory_start_time_ = walking_time_;
        left_fz_trajectory_end_time_   = walking_time_;
        left_fz_trajectory_target_  = left_dsp_fz_N_;
        left_fz_trajectory_shift_   = left_dsp_fz_N_;
      }
      else if(balancing_index_ == BalancingPhase1 )
      {
        left_fz_trajectory_end_time_ = ssp_time_start;
        left_fz_trajectory_target_ = left_ssp_fz_N_;
      }
      else if(balancing_index_ == BalancingPhase4 )
      {
        left_fz_trajectory_start_time_ = ssp_time_end;
        left_fz_trajectory_shift_ = left_ssp_fz_N_;
        if(added_step_data_.size() > 1)
        {
          if(added_step_data_[1].position_data.moving_foot == STANDING)
          {
            left_fz_trajectory_target_ = left_dsp_fz_N_;
            left_fz_trajectory_end_time_ = added_step_data_[0].time_data.abs_step_time;
          }
          else if(added_step_data_[1].position_data.moving_foot == LEFT_FOOT_SWING)
          {
            left_fz_trajectory_target_ = 0.0;
            left_fz_trajectory_end_time_ = (added_step_data_[1].time_data.abs_step_time - added_step_data_[0].time_data.abs_step_time)*0.5*added_step_data_[1].time_data.dsp_ratio + added_step_data_[0].time_data.abs_step_time;
          }
          else
          {
            left_fz_trajectory_target_ = left_ssp_fz_N_;
            left_fz_trajectory_end_time_ = (added_step_data_[1].time_data.abs_step_time - added_step_data_[0].time_data.abs_step_time)*0.5*added_step_data_[1].time_data.dsp_ratio + added_step_data_[0].time_data.abs_step_time;
          }
        }
        else {
          left_fz_trajectory_target_ = left_dsp_fz_N_;
          left_fz_trajectory_end_time_ = added_step_data_[0].time_data.abs_step_time;
        }
      }
      else if(balancing_index_ == BalancingPhase5 )
      {
        left_fz_trajectory_end_time_ = ssp_time_start;
        left_fz_trajectory_target_ = 0.0;
      }
      else if(balancing_index_ == BalancingPhase8)
      {
        left_fz_trajectory_start_time_ = ssp_time_end;
        left_fz_trajectory_shift_ = 0.0;
        if(added_step_data_.size() > 1)
        {
          if(added_step_data_[1].position_data.moving_foot == STANDING)
          {
            left_fz_trajectory_target_ = left_dsp_fz_N_;
            left_fz_trajectory_end_time_ = added_step_data_[0].time_data.abs_step_time;
          }
          else if(added_step_data_[1].position_data.moving_foot == LEFT_FOOT_SWING)
          {
            left_fz_trajectory_target_ = 0.0;
            left_fz_trajectory_end_time_ = (added_step_data_[1].time_data.abs_step_time - added_step_data_[0].time_data.abs_step_time)*0.5*added_step_data_[1].time_data.dsp_ratio + added_step_data_[0].time_data.abs_step_time;
          }
          else
          {
            left_fz_trajectory_target_ = left_ssp_fz_N_;
            left_fz_trajectory_end_time_ = (added_step_data_[1].time_data.abs_step_time - added_step_data_[0].time_data.abs_step_time)*0.5*added_step_data_[1].time_data.dsp_ratio + added_step_data_[0].time_data.abs_step_time;
          }
        }
        else
        {
          left_fz_trajectory_target_ = left_dsp_fz_N_;
          left_fz_trajectory_end_time_ = added_step_data_[0].time_data.abs_step_time;
        }
      }
      else
      {
        left_fz_trajectory_start_time_ = walking_time_;
        left_fz_trajectory_end_time_   = walking_time_;
      }
    }

    step_data_mutex_lock_.unlock();

    mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(present_body_pose_.x, present_body_pose_.y, present_body_pose_.z,
        present_body_pose_.roll, present_body_pose_.pitch, present_body_pose_.yaw);

    mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(present_right_foot_pose_.x, present_right_foot_pose_.y, present_right_foot_pose_.z,
        present_right_foot_pose_.roll, present_right_foot_pose_.pitch, present_right_foot_pose_.yaw);

    mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(present_left_foot_pose_.x, present_left_foot_pose_.y, present_left_foot_pose_.z,
        present_left_foot_pose_.roll, present_left_foot_pose_.pitch, present_left_foot_pose_.yaw);

    mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);

    mat_robot_to_cob_ = robotis_framework::getRotation4d(present_body_pose_.roll, present_body_pose_.pitch, 0);
    mat_cob_to_robot_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_);
    mat_g_to_robot_   = mat_g_to_cob_ * mat_cob_to_robot_;
    mat_robot_to_g_   = robotis_framework::getInverseTransformation(mat_g_to_robot_);

    mat_robot_to_rfoot_ = mat_robot_to_g_*mat_g_to_rfoot_;
    mat_robot_to_lfoot_ = mat_robot_to_g_*mat_g_to_lfoot_;


    //Stabilizer Start
    //Balancing Algorithm
    double target_fz_N  = 0;

    double right_leg_fx_N  = current_right_fx_N_;
    double right_leg_fy_N  = current_right_fy_N_;
    double right_leg_fz_N  = current_right_fz_N_;
    double right_leg_Tx_Nm = current_right_tx_Nm_;
    double right_leg_Ty_Nm = current_right_ty_Nm_;
    double right_leg_Tz_Nm = current_right_tz_Nm_;

    double left_leg_fx_N  = current_left_fx_N_;
    double left_leg_fy_N  = current_left_fy_N_;
    double left_leg_fz_N  = current_left_fz_N_;
    double left_leg_Tx_Nm = current_left_tx_Nm_;
    double left_leg_Ty_Nm = current_left_ty_Nm_;
    double left_leg_Tz_Nm = current_left_tz_Nm_;

    Eigen::MatrixXd  mat_right_force, mat_right_torque;
    mat_right_force.resize(4,1);    mat_right_force.fill(0);
    mat_right_torque.resize(4,1);   mat_right_torque.fill(0);
    mat_right_force(0,0) = right_leg_fx_N;
    mat_right_force(1,0) = right_leg_fy_N;
    mat_right_force(2,0) = right_leg_fz_N;
    mat_right_torque(0,0) = right_leg_Tx_Nm;
    mat_right_torque(1,0) = right_leg_Ty_Nm;
    mat_right_torque(2,0) = right_leg_Tz_Nm;

    Eigen::MatrixXd  mat_left_force, mat_left_torque;
    mat_left_force.resize(4,1);     mat_left_force.fill(0);
    mat_left_torque.resize(4,1);    mat_left_torque.fill(0);
    mat_left_force(0,0) = left_leg_fx_N;
    mat_left_force(1,0) = left_leg_fy_N;
    mat_left_force(2,0) = left_leg_fz_N;
    mat_left_torque(0,0) = left_leg_Tx_Nm;
    mat_left_torque(1,0) = left_leg_Ty_Nm;
    mat_left_torque(2,0) = left_leg_Tz_Nm;

    mat_right_force  = mat_robot_to_rfoot_*mat_rfoot_to_rft_*mat_right_force;
    mat_right_torque = mat_robot_to_rfoot_*mat_rfoot_to_rft_*mat_right_torque;

    mat_left_force  = mat_robot_to_lfoot_*mat_lfoot_to_lft_*mat_left_force;
    mat_left_torque = mat_robot_to_lfoot_*mat_lfoot_to_lft_*mat_left_torque;

    imu_data_mutex_lock_.lock();
    double gyro_roll_rad_per_sec  = current_gyro_roll_rad_per_sec_;
    double gyro_pitch_rad_per_sec = current_gyro_pitch_rad_per_sec_;

    double iu_roll_rad  = current_imu_roll_rad_;
    double iu_pitch_rad = current_imu_pitch_rad_;
    imu_data_mutex_lock_.unlock();

    balance_ctrl_.setCurrentGyroSensorOutput(gyro_roll_rad_per_sec, gyro_pitch_rad_per_sec);
    balance_ctrl_.setCurrentOrientationSensorOutput(iu_roll_rad, iu_pitch_rad);
    balance_ctrl_.setCurrentFootForceTorqueSensorOutput(mat_right_force.coeff(0,0),  mat_right_force.coeff(1,0),  mat_right_force.coeff(2,0),
                                                        mat_right_torque.coeff(0,0), mat_right_torque.coeff(1,0), mat_right_torque.coeff(2,0),
                                                        mat_left_force.coeff(0,0),   mat_left_force.coeff(1,0),   mat_left_force.coeff(2,0),
                                                        mat_left_torque.coeff(0,0),  mat_left_torque.coeff(1,0),  mat_left_torque.coeff(2,0));


    double r_target_fx_N = 0;
    double l_target_fx_N = 0;
    double r_target_fy_N = 0;
    double l_target_fy_N = 0;
    double r_target_fz_N = right_dsp_fz_N_;
    double l_target_fz_N = left_dsp_fz_N_;

    Eigen::MatrixXd mat_g_to_acc, mat_robot_to_acc;
    mat_g_to_acc.resize(4, 1);
    mat_g_to_acc.fill(0);
    mat_g_to_acc.coeffRef(0,0) = x_lipm_.coeff(2,0);
    mat_g_to_acc.coeffRef(1,0) = y_lipm_.coeff(2,0);
    mat_robot_to_acc = mat_robot_to_g_ * mat_g_to_acc;


    switch(balancing_index_)
    {
    case BalancingPhase0:
      //fprintf(stderr, "DSP : START\n");
      r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_dsp_fz_N_;
      l_target_fz_N = left_dsp_fz_N_;
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      break;
    case BalancingPhase1:
      //fprintf(stderr, "DSP : R--O->L\n");
      r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_dsp_fz_N_;
      l_target_fz_N = left_dsp_fz_N_;
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      break;
    case BalancingPhase2:
      //fprintf(stderr, "SSP : L_BALANCING1\n");
      r_target_fx_N = 0;
      r_target_fy_N = 0;
      r_target_fz_N = 0;

      l_target_fx_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      l_target_fy_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      l_target_fz_N = left_ssp_fz_N_;
      target_fz_N = left_ssp_fz_N_;
      break;
    case BalancingPhase3:
      //fprintf(stderr, "SSP : L_BALANCING2\n");
      r_target_fx_N = 0;
      r_target_fy_N = 0;
      r_target_fz_N = 0;

      l_target_fx_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      l_target_fy_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      l_target_fz_N = left_ssp_fz_N_;
      target_fz_N = left_ssp_fz_N_;
      break;
    case BalancingPhase4:
      //fprintf(stderr, "DSP : R--O<-L\n");
      r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_dsp_fz_N_;
      l_target_fz_N = left_dsp_fz_N_;
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      break;
    case BalancingPhase5:
      //fprintf(stderr, "DSP : R<-O--L\n");
      r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_dsp_fz_N_;
      l_target_fz_N = left_dsp_fz_N_;
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      break;
    case BalancingPhase6:
      //fprintf(stderr, "SSP : R_BALANCING1\n");
      r_target_fx_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_ssp_fz_N_;

      l_target_fx_N = 0;
      l_target_fy_N = 0;
      l_target_fz_N = 0;
      target_fz_N = -right_ssp_fz_N_;
      break;
    case BalancingPhase7:
      //fprintf(stderr, "SSP : R_BALANCING2\n");
      r_target_fx_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = -1.0*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_ssp_fz_N_;

      l_target_fx_N = 0;
      l_target_fy_N = 0;
      l_target_fz_N = 0;
      target_fz_N =  -right_ssp_fz_N_;
      break;
    case BalancingPhase8:
      //fprintf(stderr, "DSP : R->O--L");
      r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_dsp_fz_N_;
      l_target_fz_N = left_dsp_fz_N_;
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      break;
    case BalancingPhase9:
      //fprintf(stderr, "DSP : END");
      r_target_fx_N = l_target_fx_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(0,0);
      r_target_fy_N = l_target_fy_N = -0.5*total_mass_of_robot_*mat_robot_to_acc.coeff(1,0);
      r_target_fz_N = right_dsp_fz_N_;
      l_target_fz_N = left_dsp_fz_N_;
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      break;
    default:
      break;
    }


    bool IsDSP = false;
    if( (balancing_index_ == BalancingPhase0) ||
        (balancing_index_ == BalancingPhase1) ||
        (balancing_index_ == BalancingPhase4) ||
        (balancing_index_ == BalancingPhase5) ||
        (balancing_index_ == BalancingPhase8) ||
        (balancing_index_ == BalancingPhase9) )
    {
      IsDSP = true;
    }
    else
      IsDSP = false;

    if(IsDSP)
    {
      if( (balancing_index_ == BalancingPhase0) || (balancing_index_ == BalancingPhase9) )
      {
        r_target_fz_N = right_dsp_fz_N_;
        l_target_fz_N = left_dsp_fz_N_;
      }
      else
      {
        l_target_fz_N = wsigmoid(walking_time_ - TIME_UNIT, left_fz_trajectory_end_time_ -  left_fz_trajectory_start_time_, left_fz_trajectory_start_time_, left_fz_trajectory_target_ - left_fz_trajectory_shift_, left_fz_trajectory_shift_, 1.0, 1.0);
        r_target_fz_N = left_ssp_fz_N_ - l_target_fz_N;
      }
    }
    else
    {
      if( (balancing_index_ == BalancingPhase2) || (balancing_index_ == BalancingPhase3) )
      {
        r_target_fz_N = 0;
        l_target_fz_N = left_ssp_fz_N_;
      }
      else
      {
        r_target_fz_N = right_ssp_fz_N_;
        l_target_fz_N = 0;
      }
    }

    setBalanceOffset();

    balance_ctrl_.setDesiredCOBGyro(0,0);
//    balance_ctrl_.setDesiredCOBOrientation(present_body_pose_.roll, present_body_pose_.pitch);
    balance_ctrl_.setDesiredCOBOrientation(present_body_pose_.roll  + des_balance_offset_.coeff(0,0),
                                           present_body_pose_.pitch + des_balance_offset_.coeff(1,0));

    balance_ctrl_.setDesiredFootForceTorque(r_target_fx_N*1.0, r_target_fy_N*1.0, r_target_fz_N, 0, 0, 0,
                                            l_target_fx_N*1.0, l_target_fy_N*1.0, l_target_fz_N, 0, 0, 0);
    balance_ctrl_.setDesiredPose(mat_robot_to_cob_, mat_robot_to_rfoot_, mat_robot_to_lfoot_);

    balance_ctrl_.process(&balance_error_, &mat_robot_to_cob_modified_, &mat_robot_to_rf_modified_, &mat_robot_to_lf_modified_);
    mat_cob_to_robot_modified_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_modified_);
    tata_mat_robot_to_cob_modified_=mat_robot_to_rf_modified_;
    tf::matrixEigenToMsg(tata_mat_robot_to_cob_modified_,map_info );

    //Stabilizer End

    rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_rf_modified_);
    lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_lf_modified_);

    if((rhip_to_rfoot_pose_.yaw > 30.0*M_PI/180.0) || (rhip_to_rfoot_pose_.yaw < -30.0*M_PI/180.0) )
    {
      return;
    }

    if((lhip_to_lfoot_pose_.yaw < -30.0*M_PI/180.0) || (lhip_to_lfoot_pose_.yaw > 30.0*M_PI/180.0) )
    {
      return;
    }

    if(thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw) == false)
    {
      printf("IK not Solved pr EPR : %f %f %f %f %f %f\n", rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
      return;
    }

    if(thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw) == false)
    {
      printf("IK not Solved pl EPL : %f %f %f %f %f %f\n", lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);
      return;
    }


    r_shoulder_out_angle_rad_ = r_shoulder_dir_*(mat_robot_to_rfoot_.coeff(0, 3) - mat_robot_to_lfoot_.coeff(0, 3))*shouler_swing_gain_ + r_init_shoulder_angle_rad_;
    l_shoulder_out_angle_rad_ = l_shoulder_dir_*(mat_robot_to_lfoot_.coeff(0, 3) - mat_robot_to_rfoot_.coeff(0, 3))*shouler_swing_gain_ + l_init_shoulder_angle_rad_;
    r_elbow_out_angle_rad_ = r_elbow_dir_*(mat_robot_to_rfoot_.coeff(0, 3) - mat_robot_to_lfoot_.coeff(0, 3))*elbow_swing_gain_ + r_init_elbow_angle_rad_;
    l_elbow_out_angle_rad_ = l_elbow_dir_*(mat_robot_to_lfoot_.coeff(0, 3) - mat_robot_to_rfoot_.coeff(0, 3))*elbow_swing_gain_ + l_init_elbow_angle_rad_;


    if(added_step_data_.size() != 0)
    {
      if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
        r_leg_out_angle_rad_[1] = r_leg_out_angle_rad_[1] + hip_roll_swap;
      else if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
        r_leg_out_angle_rad_[1] = r_leg_out_angle_rad_[1] - 0.35*hip_roll_swap;
    }

    if(added_step_data_.size() != 0)
    {
      if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
        l_leg_out_angle_rad_[1] = l_leg_out_angle_rad_[1] + hip_roll_swap;
      else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
        l_leg_out_angle_rad_[1] = l_leg_out_angle_rad_[1] - 0.35*hip_roll_swap;
    }

    for(int angle_idx = 0; angle_idx < 6; angle_idx++)
    {
      leg_angle_feed_back_[angle_idx+0].desired_ = r_leg_out_angle_rad_[angle_idx];
      leg_angle_feed_back_[angle_idx+6].desired_ = l_leg_out_angle_rad_[angle_idx];
      out_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx] + leg_angle_feed_back_[angle_idx+0].getFeedBack(curr_angle_rad_[angle_idx]);
      out_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx] + leg_angle_feed_back_[angle_idx+6].getFeedBack(curr_angle_rad_[angle_idx+6]);
//      out_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx];
//      out_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx];
    }
  }
}


double THORMANG3OnlineWalking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
  {
  return mag * sin(2 * M_PI / period * time - period_shift) + mag_shift;
}

double THORMANG3OnlineWalking::wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio)
  {
  double value = mag_shift, Amplitude = 0.0, sigmoid_distor_gain = 1.0, t = 0.0;
  if ((sigmoid_ratio >= 1.0) && (sigmoid_ratio < 2.0))
  {
    if( time >= time_shift+period*(2-sigmoid_ratio)) {
      value = mag_shift + mag;
    }
    else
    {
      t = 2.0*M_PI*(time - time_shift)/(period*(2-sigmoid_ratio));
      sigmoid_distor_gain = distortion_ratio + (1-distortion_ratio)*(time-(time_shift+period*(1-sigmoid_ratio)))/(period*(2-sigmoid_ratio));
      Amplitude = mag/(2.0*M_PI);
      value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
    }
  }
  else if( (sigmoid_ratio >= 0.0) && (sigmoid_ratio < 1.0))
  {
    if( time <= time_shift+period*(1-sigmoid_ratio))
      value = mag_shift;
    else {
      t = 2.0*M_PI*(time - time_shift-period*(1-sigmoid_ratio))/(period*sigmoid_ratio);
      sigmoid_distor_gain = distortion_ratio + (1-distortion_ratio)*(time-(time_shift+period*(1-sigmoid_ratio)))/(period*sigmoid_ratio);
      Amplitude = mag/(2.0*M_PI);
      value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
    }
  }
  else if(( sigmoid_ratio >= 2.0) && ( sigmoid_ratio < 3.0))
  {
    double nsigmoid_ratio = sigmoid_ratio - 2.0;
    if(time <= time_shift + period*(1.0-nsigmoid_ratio)*0.5)
      value = mag_shift;
    else if(time >= time_shift + period*(1.0+nsigmoid_ratio)*0.5)
      value = mag + mag_shift;
    else {
      t = 2.0*M_PI*(time - (time_shift+period*(1.0-nsigmoid_ratio)*0.5))/(period*nsigmoid_ratio);
      sigmoid_distor_gain = distortion_ratio + (1.0-distortion_ratio)*(time-(time_shift+period*(1.0-nsigmoid_ratio)*0.5))/(period*nsigmoid_ratio);
      Amplitude = mag/(2.0*M_PI);
      value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
    }
  }
  else
    value = mag_shift;

  return value;
}

void THORMANG3OnlineWalking::parseBalanceOffsetData(const std::string &path)
  {
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  r_leg_to_body_roll_gain_ = doc["r_leg_to_body_roll_gain"].as<double>();
  l_leg_to_body_roll_gain_ = doc["l_leg_to_body_roll_gain"].as<double>();

  r_leg_to_body_pitch_gain_ = doc["r_leg_to_body_pitch_gain"].as<double>();
  l_leg_to_body_pitch_gain_ = doc["l_leg_to_body_pitch_gain"].as<double>();

  ROS_INFO("r_leg_to_body_roll_gain_ : %f", r_leg_to_body_roll_gain_);
  ROS_INFO("l_leg_to_body_roll_gain_ : %f", l_leg_to_body_roll_gain_);
  ROS_INFO("r_leg_to_body_pitch_gain_ : %f", r_leg_to_body_pitch_gain_);
  ROS_INFO("l_leg_to_body_pitch_gain_ : %f", l_leg_to_body_pitch_gain_);
}

void THORMANG3OnlineWalking::initBalanceOffset()
  {
  if (init_balance_offset_ == false)
  {
    if((added_step_data_.size() != 0) && real_running)
    {
      double period_time = added_step_data_[0].time_data.abs_step_time - reference_time_;
      double dsp_ratio = added_step_data_[0].time_data.dsp_ratio;

//      ROS_INFO("period_time = %f", period_time);
//      ROS_INFO("dsp_ratio = %f", dsp_ratio);

      // feedforward trajectory
      std::vector<double_t> zero_vector;
      zero_vector.resize(1,0.0);

      std::vector<double_t> via_pos;
      via_pos.resize(3, 0.0);
      via_pos[0] = 1.0 * DEGREE2RADIAN;

      double init_time = 0.0;
      double fin_time = period_time;
      double via_time = 0.5 * (init_time + fin_time);

      feed_forward_tra_ =
          new robotis_framework::MinimumJerkViaPoint(init_time, fin_time, via_time, dsp_ratio,
                                                     zero_vector, zero_vector, zero_vector,
                                                     zero_vector, zero_vector, zero_vector,
                                                     via_pos, zero_vector, zero_vector);

      mov_time_ = fin_time;
      mov_size_ = (int) (mov_time_ / TIME_UNIT) + 1;

      init_balance_offset_ = true;
    }
  }
}

void THORMANG3OnlineWalking::setBalanceOffset()
  {
  initBalanceOffset();

  bool is_DSP = false;
  bool is_l_balancing, is_r_balancing;
  if( (balancing_index_ == BalancingPhase0) ||
      (balancing_index_ == BalancingPhase1) ||
      (balancing_index_ == BalancingPhase4) ||
      (balancing_index_ == BalancingPhase5) ||
      (balancing_index_ == BalancingPhase8) ||
      (balancing_index_ == BalancingPhase9) )
  {
    is_DSP = true;
    is_l_balancing = false;
    is_r_balancing = false;
  }
  else
    is_DSP = false;

  if (balancing_index_ == 2 || balancing_index_ == 3)
  {
    is_l_balancing = true;
    is_r_balancing = false;
  }

  if (balancing_index_ == 6 || balancing_index_ == 7)
  {
    is_l_balancing = false;
    is_r_balancing = true;
  }

  des_balance_offset_ = Eigen::MatrixXd::Zero(2,1);

  if (init_balance_offset_)
  {
    double cur_time = (double) mov_step_ * TIME_UNIT;
//    ROS_INFO("cur_time : %f / %f", cur_time , mov_time_);

    if (is_DSP == false)
    {
//      ROS_INFO("SSP");
      std::vector<double_t> value = feed_forward_tra_->getPosition(cur_time);

      if (is_l_balancing)
      {
        des_balance_offset_.coeffRef(0,0) = r_leg_to_body_roll_gain_ * value[0];
        des_balance_offset_.coeffRef(1,0) = r_leg_to_body_pitch_gain_ * value[0];
//        ROS_INFO("L_BALANCING");
      }

      if (is_r_balancing)
      {
        des_balance_offset_.coeffRef(0,0) = l_leg_to_body_roll_gain_ * value[0];
        des_balance_offset_.coeffRef(1,0) = l_leg_to_body_pitch_gain_ * value[0];
//        ROS_INFO("R_BALANCING");
      }
    }

    if (mov_step_ == mov_size_-1)
    {
      mov_step_ = 0;
      init_balance_offset_ = false;
    }
    else
      mov_step_++;

//    ROS_INFO("des_balance_offset_ roll: %f, pitch: %f", des_balance_offset_.coeff(0,0), des_balance_offset_.coeff(1,0));
  }
}
