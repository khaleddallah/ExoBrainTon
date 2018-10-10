#ifndef _ROS_thormang3_walking_module_msgs_DampingBalanceParam_h
#define _ROS_thormang3_walking_module_msgs_DampingBalanceParam_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_walking_module_msgs
{

  class DampingBalanceParam : public ros::Msg
  {
    public:
      typedef float _cob_x_offset_m_type;
      _cob_x_offset_m_type cob_x_offset_m;
      typedef float _cob_y_offset_m_type;
      _cob_y_offset_m_type cob_y_offset_m;
      typedef float _hip_roll_swap_angle_rad_type;
      _hip_roll_swap_angle_rad_type hip_roll_swap_angle_rad;
      typedef float _gyro_gain_type;
      _gyro_gain_type gyro_gain;
      typedef float _foot_roll_angle_gain_type;
      _foot_roll_angle_gain_type foot_roll_angle_gain;
      typedef float _foot_pitch_angle_gain_type;
      _foot_pitch_angle_gain_type foot_pitch_angle_gain;
      typedef float _foot_x_force_gain_type;
      _foot_x_force_gain_type foot_x_force_gain;
      typedef float _foot_y_force_gain_type;
      _foot_y_force_gain_type foot_y_force_gain;
      typedef float _foot_z_force_gain_type;
      _foot_z_force_gain_type foot_z_force_gain;
      typedef float _foot_roll_torque_gain_type;
      _foot_roll_torque_gain_type foot_roll_torque_gain;
      typedef float _foot_pitch_torque_gain_type;
      _foot_pitch_torque_gain_type foot_pitch_torque_gain;
      typedef float _foot_roll_angle_time_constant_type;
      _foot_roll_angle_time_constant_type foot_roll_angle_time_constant;
      typedef float _foot_pitch_angle_time_constant_type;
      _foot_pitch_angle_time_constant_type foot_pitch_angle_time_constant;
      typedef float _foot_x_force_time_constant_type;
      _foot_x_force_time_constant_type foot_x_force_time_constant;
      typedef float _foot_y_force_time_constant_type;
      _foot_y_force_time_constant_type foot_y_force_time_constant;
      typedef float _foot_z_force_time_constant_type;
      _foot_z_force_time_constant_type foot_z_force_time_constant;
      typedef float _foot_roll_torque_time_constant_type;
      _foot_roll_torque_time_constant_type foot_roll_torque_time_constant;
      typedef float _foot_pitch_torque_time_constant_type;
      _foot_pitch_torque_time_constant_type foot_pitch_torque_time_constant;

    DampingBalanceParam():
      cob_x_offset_m(0),
      cob_y_offset_m(0),
      hip_roll_swap_angle_rad(0),
      gyro_gain(0),
      foot_roll_angle_gain(0),
      foot_pitch_angle_gain(0),
      foot_x_force_gain(0),
      foot_y_force_gain(0),
      foot_z_force_gain(0),
      foot_roll_torque_gain(0),
      foot_pitch_torque_gain(0),
      foot_roll_angle_time_constant(0),
      foot_pitch_angle_time_constant(0),
      foot_x_force_time_constant(0),
      foot_y_force_time_constant(0),
      foot_z_force_time_constant(0),
      foot_roll_torque_time_constant(0),
      foot_pitch_torque_time_constant(0)
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
      } u_gyro_gain;
      u_gyro_gain.real = this->gyro_gain;
      *(outbuffer + offset + 0) = (u_gyro_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyro_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyro_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyro_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyro_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_angle_gain;
      u_foot_roll_angle_gain.real = this->foot_roll_angle_gain;
      *(outbuffer + offset + 0) = (u_foot_roll_angle_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_roll_angle_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_roll_angle_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_roll_angle_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_roll_angle_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_angle_gain;
      u_foot_pitch_angle_gain.real = this->foot_pitch_angle_gain;
      *(outbuffer + offset + 0) = (u_foot_pitch_angle_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_pitch_angle_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_pitch_angle_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_pitch_angle_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_pitch_angle_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_x_force_gain;
      u_foot_x_force_gain.real = this->foot_x_force_gain;
      *(outbuffer + offset + 0) = (u_foot_x_force_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_x_force_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_x_force_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_x_force_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_x_force_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_y_force_gain;
      u_foot_y_force_gain.real = this->foot_y_force_gain;
      *(outbuffer + offset + 0) = (u_foot_y_force_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_y_force_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_y_force_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_y_force_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_y_force_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_z_force_gain;
      u_foot_z_force_gain.real = this->foot_z_force_gain;
      *(outbuffer + offset + 0) = (u_foot_z_force_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_z_force_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_z_force_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_z_force_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_z_force_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_torque_gain;
      u_foot_roll_torque_gain.real = this->foot_roll_torque_gain;
      *(outbuffer + offset + 0) = (u_foot_roll_torque_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_roll_torque_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_roll_torque_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_roll_torque_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_roll_torque_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_torque_gain;
      u_foot_pitch_torque_gain.real = this->foot_pitch_torque_gain;
      *(outbuffer + offset + 0) = (u_foot_pitch_torque_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_pitch_torque_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_pitch_torque_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_pitch_torque_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_pitch_torque_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_angle_time_constant;
      u_foot_roll_angle_time_constant.real = this->foot_roll_angle_time_constant;
      *(outbuffer + offset + 0) = (u_foot_roll_angle_time_constant.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_roll_angle_time_constant.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_roll_angle_time_constant.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_roll_angle_time_constant.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_roll_angle_time_constant);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_angle_time_constant;
      u_foot_pitch_angle_time_constant.real = this->foot_pitch_angle_time_constant;
      *(outbuffer + offset + 0) = (u_foot_pitch_angle_time_constant.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_pitch_angle_time_constant.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_pitch_angle_time_constant.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_pitch_angle_time_constant.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_pitch_angle_time_constant);
      union {
        float real;
        uint32_t base;
      } u_foot_x_force_time_constant;
      u_foot_x_force_time_constant.real = this->foot_x_force_time_constant;
      *(outbuffer + offset + 0) = (u_foot_x_force_time_constant.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_x_force_time_constant.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_x_force_time_constant.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_x_force_time_constant.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_x_force_time_constant);
      union {
        float real;
        uint32_t base;
      } u_foot_y_force_time_constant;
      u_foot_y_force_time_constant.real = this->foot_y_force_time_constant;
      *(outbuffer + offset + 0) = (u_foot_y_force_time_constant.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_y_force_time_constant.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_y_force_time_constant.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_y_force_time_constant.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_y_force_time_constant);
      union {
        float real;
        uint32_t base;
      } u_foot_z_force_time_constant;
      u_foot_z_force_time_constant.real = this->foot_z_force_time_constant;
      *(outbuffer + offset + 0) = (u_foot_z_force_time_constant.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_z_force_time_constant.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_z_force_time_constant.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_z_force_time_constant.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_z_force_time_constant);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_torque_time_constant;
      u_foot_roll_torque_time_constant.real = this->foot_roll_torque_time_constant;
      *(outbuffer + offset + 0) = (u_foot_roll_torque_time_constant.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_roll_torque_time_constant.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_roll_torque_time_constant.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_roll_torque_time_constant.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_roll_torque_time_constant);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_torque_time_constant;
      u_foot_pitch_torque_time_constant.real = this->foot_pitch_torque_time_constant;
      *(outbuffer + offset + 0) = (u_foot_pitch_torque_time_constant.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_pitch_torque_time_constant.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_pitch_torque_time_constant.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_pitch_torque_time_constant.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_pitch_torque_time_constant);
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
      } u_gyro_gain;
      u_gyro_gain.base = 0;
      u_gyro_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyro_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyro_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyro_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyro_gain = u_gyro_gain.real;
      offset += sizeof(this->gyro_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_angle_gain;
      u_foot_roll_angle_gain.base = 0;
      u_foot_roll_angle_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_roll_angle_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_roll_angle_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_roll_angle_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_roll_angle_gain = u_foot_roll_angle_gain.real;
      offset += sizeof(this->foot_roll_angle_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_angle_gain;
      u_foot_pitch_angle_gain.base = 0;
      u_foot_pitch_angle_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_pitch_angle_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_pitch_angle_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_pitch_angle_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_pitch_angle_gain = u_foot_pitch_angle_gain.real;
      offset += sizeof(this->foot_pitch_angle_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_x_force_gain;
      u_foot_x_force_gain.base = 0;
      u_foot_x_force_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_x_force_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_x_force_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_x_force_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_x_force_gain = u_foot_x_force_gain.real;
      offset += sizeof(this->foot_x_force_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_y_force_gain;
      u_foot_y_force_gain.base = 0;
      u_foot_y_force_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_y_force_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_y_force_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_y_force_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_y_force_gain = u_foot_y_force_gain.real;
      offset += sizeof(this->foot_y_force_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_z_force_gain;
      u_foot_z_force_gain.base = 0;
      u_foot_z_force_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_z_force_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_z_force_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_z_force_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_z_force_gain = u_foot_z_force_gain.real;
      offset += sizeof(this->foot_z_force_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_torque_gain;
      u_foot_roll_torque_gain.base = 0;
      u_foot_roll_torque_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_roll_torque_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_roll_torque_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_roll_torque_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_roll_torque_gain = u_foot_roll_torque_gain.real;
      offset += sizeof(this->foot_roll_torque_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_torque_gain;
      u_foot_pitch_torque_gain.base = 0;
      u_foot_pitch_torque_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_pitch_torque_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_pitch_torque_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_pitch_torque_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_pitch_torque_gain = u_foot_pitch_torque_gain.real;
      offset += sizeof(this->foot_pitch_torque_gain);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_angle_time_constant;
      u_foot_roll_angle_time_constant.base = 0;
      u_foot_roll_angle_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_roll_angle_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_roll_angle_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_roll_angle_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_roll_angle_time_constant = u_foot_roll_angle_time_constant.real;
      offset += sizeof(this->foot_roll_angle_time_constant);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_angle_time_constant;
      u_foot_pitch_angle_time_constant.base = 0;
      u_foot_pitch_angle_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_pitch_angle_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_pitch_angle_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_pitch_angle_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_pitch_angle_time_constant = u_foot_pitch_angle_time_constant.real;
      offset += sizeof(this->foot_pitch_angle_time_constant);
      union {
        float real;
        uint32_t base;
      } u_foot_x_force_time_constant;
      u_foot_x_force_time_constant.base = 0;
      u_foot_x_force_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_x_force_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_x_force_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_x_force_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_x_force_time_constant = u_foot_x_force_time_constant.real;
      offset += sizeof(this->foot_x_force_time_constant);
      union {
        float real;
        uint32_t base;
      } u_foot_y_force_time_constant;
      u_foot_y_force_time_constant.base = 0;
      u_foot_y_force_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_y_force_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_y_force_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_y_force_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_y_force_time_constant = u_foot_y_force_time_constant.real;
      offset += sizeof(this->foot_y_force_time_constant);
      union {
        float real;
        uint32_t base;
      } u_foot_z_force_time_constant;
      u_foot_z_force_time_constant.base = 0;
      u_foot_z_force_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_z_force_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_z_force_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_z_force_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_z_force_time_constant = u_foot_z_force_time_constant.real;
      offset += sizeof(this->foot_z_force_time_constant);
      union {
        float real;
        uint32_t base;
      } u_foot_roll_torque_time_constant;
      u_foot_roll_torque_time_constant.base = 0;
      u_foot_roll_torque_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_roll_torque_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_roll_torque_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_roll_torque_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_roll_torque_time_constant = u_foot_roll_torque_time_constant.real;
      offset += sizeof(this->foot_roll_torque_time_constant);
      union {
        float real;
        uint32_t base;
      } u_foot_pitch_torque_time_constant;
      u_foot_pitch_torque_time_constant.base = 0;
      u_foot_pitch_torque_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_pitch_torque_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_pitch_torque_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_pitch_torque_time_constant.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_pitch_torque_time_constant = u_foot_pitch_torque_time_constant.real;
      offset += sizeof(this->foot_pitch_torque_time_constant);
     return offset;
    }

    const char * getType(){ return "thormang3_walking_module_msgs/DampingBalanceParam"; };
    const char * getMD5(){ return "8c8b6e2acc152a1c5a51242d901feb03"; };

  };

}
#endif