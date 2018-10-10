#ifndef _ROS_thormang3_walking_module_msgs_BalanceParam_h
#define _ROS_thormang3_walking_module_msgs_BalanceParam_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_walking_module_msgs
{

  class BalanceParam : public ros::Msg
  {
    public:
      typedef float _cob_x_offset_m_type;
      _cob_x_offset_m_type cob_x_offset_m;
      typedef float _cob_y_offset_m_type;
      _cob_y_offset_m_type cob_y_offset_m;
      typedef float _hip_roll_swap_angle_rad_type;
      _hip_roll_swap_angle_rad_type hip_roll_swap_angle_rad;
      typedef float _foot_roll_gyro_p_gain_type;
      _foot_roll_gyro_p_gain_type foot_roll_gyro_p_gain;
      typedef float _foot_roll_gyro_d_gain_type;
      _foot_roll_gyro_d_gain_type foot_roll_gyro_d_gain;
      typedef float _foot_pitch_gyro_p_gain_type;
      _foot_pitch_gyro_p_gain_type foot_pitch_gyro_p_gain;
      typedef float _foot_pitch_gyro_d_gain_type;
      _foot_pitch_gyro_d_gain_type foot_pitch_gyro_d_gain;
      typedef float _foot_roll_angle_p_gain_type;
      _foot_roll_angle_p_gain_type foot_roll_angle_p_gain;
      typedef float _foot_roll_angle_d_gain_type;
      _foot_roll_angle_d_gain_type foot_roll_angle_d_gain;
      typedef float _foot_pitch_angle_p_gain_type;
      _foot_pitch_angle_p_gain_type foot_pitch_angle_p_gain;
      typedef float _foot_pitch_angle_d_gain_type;
      _foot_pitch_angle_d_gain_type foot_pitch_angle_d_gain;
      typedef float _foot_x_force_p_gain_type;
      _foot_x_force_p_gain_type foot_x_force_p_gain;
      typedef float _foot_x_force_d_gain_type;
      _foot_x_force_d_gain_type foot_x_force_d_gain;
      typedef float _foot_y_force_p_gain_type;
      _foot_y_force_p_gain_type foot_y_force_p_gain;
      typedef float _foot_y_force_d_gain_type;
      _foot_y_force_d_gain_type foot_y_force_d_gain;
      typedef float _foot_z_force_p_gain_type;
      _foot_z_force_p_gain_type foot_z_force_p_gain;
      typedef float _foot_z_force_d_gain_type;
      _foot_z_force_d_gain_type foot_z_force_d_gain;
      typedef float _foot_roll_torque_p_gain_type;
      _foot_roll_torque_p_gain_type foot_roll_torque_p_gain;
      typedef float _foot_roll_torque_d_gain_type;
      _foot_roll_torque_d_gain_type foot_roll_torque_d_gain;
      typedef float _foot_pitch_torque_p_gain_type;
      _foot_pitch_torque_p_gain_type foot_pitch_torque_p_gain;
      typedef float _foot_pitch_torque_d_gain_type;
      _foot_pitch_torque_d_gain_type foot_pitch_torque_d_gain;
      typedef float _roll_gyro_cut_off_frequency_type;
      _roll_gyro_cut_off_frequency_type roll_gyro_cut_off_frequency;
      typedef float _pitch_gyro_cut_off_frequency_type;
      _pitch_gyro_cut_off_frequency_type pitch_gyro_cut_off_frequency;
      typedef float _roll_angle_cut_off_frequency_type;
      _roll_angle_cut_off_frequency_type roll_angle_cut_off_frequency;
      typedef float _pitch_angle_cut_off_frequency_type;
      _pitch_angle_cut_off_frequency_type pitch_angle_cut_off_frequency;
      typedef float _foot_x_force_cut_off_frequency_type;
      _foot_x_force_cut_off_frequency_type foot_x_force_cut_off_frequency;
      typedef float _foot_y_force_cut_off_frequency_type;
      _foot_y_force_cut_off_frequency_type foot_y_force_cut_off_frequency;
      typedef float _foot_z_force_cut_off_frequency_type;
      _foot_z_force_cut_off_frequency_type foot_z_force_cut_off_frequency;
      typedef float _foot_roll_torque_cut_off_frequency_type;
      _foot_roll_torque_cut_off_frequency_type foot_roll_torque_cut_off_frequency;
      typedef float _foot_pitch_torque_cut_off_frequency_type;
      _foot_pitch_torque_cut_off_frequency_type foot_pitch_torque_cut_off_frequency;

    BalanceParam():
      cob_x_offset_m(0),
      cob_y_offset_m(0),
      hip_roll_swap_angle_rad(0),
      foot_roll_gyro_p_gain(0),
      foot_roll_gyro_d_gain(0),
      foot_pitch_gyro_p_gain(0),
      foot_pitch_gyro_d_gain(0),
      foot_roll_angle_p_gain(0),
      foot_roll_angle_d_gain(0),
      foot_pitch_angle_p_gain(0),
      foot_pitch_angle_d_gain(0),
      foot_x_force_p_gain(0),
      foot_x_force_d_gain(0),
      foot_y_force_p_gain(0),
      foot_y_force_d_gain(0),
      foot_z_force_p_gain(0),
      foot_z_force_d_gain(0),
      foot_roll_torque_p_gain(0),
      foot_roll_torque_d_gain(0),
      foot_pitch_torque_p_gain(0),
      foot_pitch_torque_d_gain(0),
      roll_gyro_cut_off_frequency(0),
      pitch_gyro_cut_off_frequency(0),
      roll_angle_cut_off_frequency(0),
      pitch_angle_cut_off_frequency(0),
      foot_x_force_cut_off_frequency(0),
      foot_y_force_cut_off_frequency(0),
      foot_z_force_cut_off_frequency(0),
      foot_roll_torque_cut_off_frequency(0),
      foot_pitch_torque_cut_off_frequency(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_cob_x_offset_m;
      u_cob_x_offset_m.real = this->cob_x_offset_m;
      *(outbuffer + offset + 0) = (u_cob_x_offset_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cob_x_offset_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cob_x_offset_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cob_x_offset_m.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cob_x_offset_m);
      union {
        float real;
        uint32_t base;
      } u_cob_y_offset_m;
      u_cob_y_offset_m.real = this->cob_y_offset_m;
      *(outbuffer + offset + 0) = (u_cob_y_offset_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cob_y_offset_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cob_y_offset_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cob_y_offset_m.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cob_y_offset_m);
      union {
        float real;
        uint32_t base;
      } u_hip_roll_swap_angle_rad;
      u_hip_roll_swap_angle_rad.real = this->hip_roll_swap_angle_rad;
      *(outbuffer + offset + 0) = (u_hip_roll_swap_angle_rad.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hip_roll_swap_angle_rad.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_hip_roll_swap_angle_rad.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_hip_roll_swap_angle_rad.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->hip_roll_swap_angle_rad);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_gyro_p_gain;
      u_foot_roll_gyro_p_gain.real = this->foot_roll_gyro_p_gain;
      *(outbuffer + offset + 0) = (u_foot_roll_gyro_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_roll_gyro_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_roll_gyro_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_roll_gyro_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_roll_gyro_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_gyro_d_gain;
      u_foot_roll_gyro_d_gain.real = this->foot_roll_gyro_d_gain;
      *(outbuffer + offset + 0) = (u_foot_roll_gyro_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_roll_gyro_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_roll_gyro_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_roll_gyro_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_roll_gyro_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_gyro_p_gain;
      u_foot_pitch_gyro_p_gain.real = this->foot_pitch_gyro_p_gain;
      *(outbuffer + offset + 0) = (u_foot_pitch_gyro_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_pitch_gyro_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_pitch_gyro_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_pitch_gyro_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_pitch_gyro_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_gyro_d_gain;
      u_foot_pitch_gyro_d_gain.real = this->foot_pitch_gyro_d_gain;
      *(outbuffer + offset + 0) = (u_foot_pitch_gyro_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_pitch_gyro_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_pitch_gyro_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_pitch_gyro_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_pitch_gyro_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_angle_p_gain;
      u_foot_roll_angle_p_gain.real = this->foot_roll_angle_p_gain;
      *(outbuffer + offset + 0) = (u_foot_roll_angle_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_roll_angle_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_roll_angle_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_roll_angle_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_roll_angle_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_angle_d_gain;
      u_foot_roll_angle_d_gain.real = this->foot_roll_angle_d_gain;
      *(outbuffer + offset + 0) = (u_foot_roll_angle_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_roll_angle_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_roll_angle_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_roll_angle_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_roll_angle_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_angle_p_gain;
      u_foot_pitch_angle_p_gain.real = this->foot_pitch_angle_p_gain;
      *(outbuffer + offset + 0) = (u_foot_pitch_angle_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_pitch_angle_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_pitch_angle_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_pitch_angle_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_pitch_angle_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_angle_d_gain;
      u_foot_pitch_angle_d_gain.real = this->foot_pitch_angle_d_gain;
      *(outbuffer + offset + 0) = (u_foot_pitch_angle_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_pitch_angle_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_pitch_angle_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_pitch_angle_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_pitch_angle_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_x_force_p_gain;
      u_foot_x_force_p_gain.real = this->foot_x_force_p_gain;
      *(outbuffer + offset + 0) = (u_foot_x_force_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_x_force_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_x_force_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_x_force_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_x_force_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_x_force_d_gain;
      u_foot_x_force_d_gain.real = this->foot_x_force_d_gain;
      *(outbuffer + offset + 0) = (u_foot_x_force_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_x_force_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_x_force_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_x_force_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_x_force_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_y_force_p_gain;
      u_foot_y_force_p_gain.real = this->foot_y_force_p_gain;
      *(outbuffer + offset + 0) = (u_foot_y_force_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_y_force_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_y_force_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_y_force_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_y_force_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_y_force_d_gain;
      u_foot_y_force_d_gain.real = this->foot_y_force_d_gain;
      *(outbuffer + offset + 0) = (u_foot_y_force_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_y_force_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_y_force_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_y_force_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_y_force_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_z_force_p_gain;
      u_foot_z_force_p_gain.real = this->foot_z_force_p_gain;
      *(outbuffer + offset + 0) = (u_foot_z_force_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_z_force_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_z_force_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_z_force_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_z_force_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_z_force_d_gain;
      u_foot_z_force_d_gain.real = this->foot_z_force_d_gain;
      *(outbuffer + offset + 0) = (u_foot_z_force_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_z_force_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_z_force_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_z_force_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_z_force_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_torque_p_gain;
      u_foot_roll_torque_p_gain.real = this->foot_roll_torque_p_gain;
      *(outbuffer + offset + 0) = (u_foot_roll_torque_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_roll_torque_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_roll_torque_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_roll_torque_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_roll_torque_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_torque_d_gain;
      u_foot_roll_torque_d_gain.real = this->foot_roll_torque_d_gain;
      *(outbuffer + offset + 0) = (u_foot_roll_torque_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_roll_torque_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_roll_torque_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_roll_torque_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_roll_torque_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_torque_p_gain;
      u_foot_pitch_torque_p_gain.real = this->foot_pitch_torque_p_gain;
      *(outbuffer + offset + 0) = (u_foot_pitch_torque_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_pitch_torque_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_pitch_torque_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_pitch_torque_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_pitch_torque_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_torque_d_gain;
      u_foot_pitch_torque_d_gain.real = this->foot_pitch_torque_d_gain;
      *(outbuffer + offset + 0) = (u_foot_pitch_torque_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_pitch_torque_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_pitch_torque_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_pitch_torque_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_pitch_torque_d_gain);
      union {
        float real;
        uint32_t base;
      } u_roll_gyro_cut_off_frequency;
      u_roll_gyro_cut_off_frequency.real = this->roll_gyro_cut_off_frequency;
      *(outbuffer + offset + 0) = (u_roll_gyro_cut_off_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_gyro_cut_off_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_gyro_cut_off_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_gyro_cut_off_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_gyro_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_pitch_gyro_cut_off_frequency;
      u_pitch_gyro_cut_off_frequency.real = this->pitch_gyro_cut_off_frequency;
      *(outbuffer + offset + 0) = (u_pitch_gyro_cut_off_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch_gyro_cut_off_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch_gyro_cut_off_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch_gyro_cut_off_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch_gyro_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_roll_angle_cut_off_frequency;
      u_roll_angle_cut_off_frequency.real = this->roll_angle_cut_off_frequency;
      *(outbuffer + offset + 0) = (u_roll_angle_cut_off_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_angle_cut_off_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_angle_cut_off_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_angle_cut_off_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_angle_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_pitch_angle_cut_off_frequency;
      u_pitch_angle_cut_off_frequency.real = this->pitch_angle_cut_off_frequency;
      *(outbuffer + offset + 0) = (u_pitch_angle_cut_off_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch_angle_cut_off_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch_angle_cut_off_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch_angle_cut_off_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch_angle_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_foot_x_force_cut_off_frequency;
      u_foot_x_force_cut_off_frequency.real = this->foot_x_force_cut_off_frequency;
      *(outbuffer + offset + 0) = (u_foot_x_force_cut_off_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_x_force_cut_off_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_x_force_cut_off_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_x_force_cut_off_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_x_force_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_foot_y_force_cut_off_frequency;
      u_foot_y_force_cut_off_frequency.real = this->foot_y_force_cut_off_frequency;
      *(outbuffer + offset + 0) = (u_foot_y_force_cut_off_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_y_force_cut_off_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_y_force_cut_off_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_y_force_cut_off_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_y_force_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_foot_z_force_cut_off_frequency;
      u_foot_z_force_cut_off_frequency.real = this->foot_z_force_cut_off_frequency;
      *(outbuffer + offset + 0) = (u_foot_z_force_cut_off_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_z_force_cut_off_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_z_force_cut_off_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_z_force_cut_off_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_z_force_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_torque_cut_off_frequency;
      u_foot_roll_torque_cut_off_frequency.real = this->foot_roll_torque_cut_off_frequency;
      *(outbuffer + offset + 0) = (u_foot_roll_torque_cut_off_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_roll_torque_cut_off_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_roll_torque_cut_off_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_roll_torque_cut_off_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_roll_torque_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_torque_cut_off_frequency;
      u_foot_pitch_torque_cut_off_frequency.real = this->foot_pitch_torque_cut_off_frequency;
      *(outbuffer + offset + 0) = (u_foot_pitch_torque_cut_off_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_pitch_torque_cut_off_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_pitch_torque_cut_off_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_pitch_torque_cut_off_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_pitch_torque_cut_off_frequency);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_cob_x_offset_m;
      u_cob_x_offset_m.base = 0;
      u_cob_x_offset_m.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cob_x_offset_m.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cob_x_offset_m.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cob_x_offset_m.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cob_x_offset_m = u_cob_x_offset_m.real;
      offset += sizeof(this->cob_x_offset_m);
      union {
        float real;
        uint32_t base;
      } u_cob_y_offset_m;
      u_cob_y_offset_m.base = 0;
      u_cob_y_offset_m.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cob_y_offset_m.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cob_y_offset_m.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cob_y_offset_m.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cob_y_offset_m = u_cob_y_offset_m.real;
      offset += sizeof(this->cob_y_offset_m);
      union {
        float real;
        uint32_t base;
      } u_hip_roll_swap_angle_rad;
      u_hip_roll_swap_angle_rad.base = 0;
      u_hip_roll_swap_angle_rad.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hip_roll_swap_angle_rad.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_hip_roll_swap_angle_rad.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_hip_roll_swap_angle_rad.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->hip_roll_swap_angle_rad = u_hip_roll_swap_angle_rad.real;
      offset += sizeof(this->hip_roll_swap_angle_rad);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_gyro_p_gain;
      u_foot_roll_gyro_p_gain.base = 0;
      u_foot_roll_gyro_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_roll_gyro_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_roll_gyro_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_roll_gyro_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_roll_gyro_p_gain = u_foot_roll_gyro_p_gain.real;
      offset += sizeof(this->foot_roll_gyro_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_gyro_d_gain;
      u_foot_roll_gyro_d_gain.base = 0;
      u_foot_roll_gyro_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_roll_gyro_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_roll_gyro_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_roll_gyro_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_roll_gyro_d_gain = u_foot_roll_gyro_d_gain.real;
      offset += sizeof(this->foot_roll_gyro_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_gyro_p_gain;
      u_foot_pitch_gyro_p_gain.base = 0;
      u_foot_pitch_gyro_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_pitch_gyro_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_pitch_gyro_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_pitch_gyro_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_pitch_gyro_p_gain = u_foot_pitch_gyro_p_gain.real;
      offset += sizeof(this->foot_pitch_gyro_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_gyro_d_gain;
      u_foot_pitch_gyro_d_gain.base = 0;
      u_foot_pitch_gyro_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_pitch_gyro_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_pitch_gyro_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_pitch_gyro_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_pitch_gyro_d_gain = u_foot_pitch_gyro_d_gain.real;
      offset += sizeof(this->foot_pitch_gyro_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_angle_p_gain;
      u_foot_roll_angle_p_gain.base = 0;
      u_foot_roll_angle_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_roll_angle_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_roll_angle_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_roll_angle_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_roll_angle_p_gain = u_foot_roll_angle_p_gain.real;
      offset += sizeof(this->foot_roll_angle_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_angle_d_gain;
      u_foot_roll_angle_d_gain.base = 0;
      u_foot_roll_angle_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_roll_angle_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_roll_angle_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_roll_angle_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_roll_angle_d_gain = u_foot_roll_angle_d_gain.real;
      offset += sizeof(this->foot_roll_angle_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_angle_p_gain;
      u_foot_pitch_angle_p_gain.base = 0;
      u_foot_pitch_angle_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_pitch_angle_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_pitch_angle_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_pitch_angle_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_pitch_angle_p_gain = u_foot_pitch_angle_p_gain.real;
      offset += sizeof(this->foot_pitch_angle_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_angle_d_gain;
      u_foot_pitch_angle_d_gain.base = 0;
      u_foot_pitch_angle_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_pitch_angle_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_pitch_angle_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_pitch_angle_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_pitch_angle_d_gain = u_foot_pitch_angle_d_gain.real;
      offset += sizeof(this->foot_pitch_angle_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_x_force_p_gain;
      u_foot_x_force_p_gain.base = 0;
      u_foot_x_force_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_x_force_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_x_force_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_x_force_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_x_force_p_gain = u_foot_x_force_p_gain.real;
      offset += sizeof(this->foot_x_force_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_x_force_d_gain;
      u_foot_x_force_d_gain.base = 0;
      u_foot_x_force_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_x_force_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_x_force_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_x_force_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_x_force_d_gain = u_foot_x_force_d_gain.real;
      offset += sizeof(this->foot_x_force_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_y_force_p_gain;
      u_foot_y_force_p_gain.base = 0;
      u_foot_y_force_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_y_force_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_y_force_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_y_force_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_y_force_p_gain = u_foot_y_force_p_gain.real;
      offset += sizeof(this->foot_y_force_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_y_force_d_gain;
      u_foot_y_force_d_gain.base = 0;
      u_foot_y_force_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_y_force_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_y_force_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_y_force_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_y_force_d_gain = u_foot_y_force_d_gain.real;
      offset += sizeof(this->foot_y_force_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_z_force_p_gain;
      u_foot_z_force_p_gain.base = 0;
      u_foot_z_force_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_z_force_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_z_force_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_z_force_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_z_force_p_gain = u_foot_z_force_p_gain.real;
      offset += sizeof(this->foot_z_force_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_z_force_d_gain;
      u_foot_z_force_d_gain.base = 0;
      u_foot_z_force_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_z_force_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_z_force_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_z_force_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_z_force_d_gain = u_foot_z_force_d_gain.real;
      offset += sizeof(this->foot_z_force_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_torque_p_gain;
      u_foot_roll_torque_p_gain.base = 0;
      u_foot_roll_torque_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_roll_torque_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_roll_torque_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_roll_torque_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_roll_torque_p_gain = u_foot_roll_torque_p_gain.real;
      offset += sizeof(this->foot_roll_torque_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_torque_d_gain;
      u_foot_roll_torque_d_gain.base = 0;
      u_foot_roll_torque_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_roll_torque_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_roll_torque_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_roll_torque_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_roll_torque_d_gain = u_foot_roll_torque_d_gain.real;
      offset += sizeof(this->foot_roll_torque_d_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_torque_p_gain;
      u_foot_pitch_torque_p_gain.base = 0;
      u_foot_pitch_torque_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_pitch_torque_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_pitch_torque_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_pitch_torque_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_pitch_torque_p_gain = u_foot_pitch_torque_p_gain.real;
      offset += sizeof(this->foot_pitch_torque_p_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_torque_d_gain;
      u_foot_pitch_torque_d_gain.base = 0;
      u_foot_pitch_torque_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_pitch_torque_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_pitch_torque_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_pitch_torque_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_pitch_torque_d_gain = u_foot_pitch_torque_d_gain.real;
      offset += sizeof(this->foot_pitch_torque_d_gain);
      union {
        float real;
        uint32_t base;
      } u_roll_gyro_cut_off_frequency;
      u_roll_gyro_cut_off_frequency.base = 0;
      u_roll_gyro_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_gyro_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_gyro_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_gyro_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_gyro_cut_off_frequency = u_roll_gyro_cut_off_frequency.real;
      offset += sizeof(this->roll_gyro_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_pitch_gyro_cut_off_frequency;
      u_pitch_gyro_cut_off_frequency.base = 0;
      u_pitch_gyro_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch_gyro_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch_gyro_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch_gyro_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch_gyro_cut_off_frequency = u_pitch_gyro_cut_off_frequency.real;
      offset += sizeof(this->pitch_gyro_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_roll_angle_cut_off_frequency;
      u_roll_angle_cut_off_frequency.base = 0;
      u_roll_angle_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_angle_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_angle_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_angle_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_angle_cut_off_frequency = u_roll_angle_cut_off_frequency.real;
      offset += sizeof(this->roll_angle_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_pitch_angle_cut_off_frequency;
      u_pitch_angle_cut_off_frequency.base = 0;
      u_pitch_angle_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch_angle_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch_angle_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch_angle_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch_angle_cut_off_frequency = u_pitch_angle_cut_off_frequency.real;
      offset += sizeof(this->pitch_angle_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_foot_x_force_cut_off_frequency;
      u_foot_x_force_cut_off_frequency.base = 0;
      u_foot_x_force_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_x_force_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_x_force_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_x_force_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_x_force_cut_off_frequency = u_foot_x_force_cut_off_frequency.real;
      offset += sizeof(this->foot_x_force_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_foot_y_force_cut_off_frequency;
      u_foot_y_force_cut_off_frequency.base = 0;
      u_foot_y_force_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_y_force_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_y_force_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_y_force_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_y_force_cut_off_frequency = u_foot_y_force_cut_off_frequency.real;
      offset += sizeof(this->foot_y_force_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_foot_z_force_cut_off_frequency;
      u_foot_z_force_cut_off_frequency.base = 0;
      u_foot_z_force_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_z_force_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_z_force_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_z_force_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_z_force_cut_off_frequency = u_foot_z_force_cut_off_frequency.real;
      offset += sizeof(this->foot_z_force_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_torque_cut_off_frequency;
      u_foot_roll_torque_cut_off_frequency.base = 0;
      u_foot_roll_torque_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_roll_torque_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_roll_torque_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_roll_torque_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_roll_torque_cut_off_frequency = u_foot_roll_torque_cut_off_frequency.real;
      offset += sizeof(this->foot_roll_torque_cut_off_frequency);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_torque_cut_off_frequency;
      u_foot_pitch_torque_cut_off_frequency.base = 0;
      u_foot_pitch_torque_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_pitch_torque_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_pitch_torque_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_pitch_torque_cut_off_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_pitch_torque_cut_off_frequency = u_foot_pitch_torque_cut_off_frequency.real;
      offset += sizeof(this->foot_pitch_torque_cut_off_frequency);
     return offset;
    }

    const char * getType(){ return "thormang3_walking_module_msgs/BalanceParam"; };
    const char * getMD5(){ return "dd76e7dd73be703607cfe08ec343d74f"; };

  };

}
#endif