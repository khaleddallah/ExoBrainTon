#ifndef _ROS_thormang3_walking_module_msgs_JointFeedBackGain_h
#define _ROS_thormang3_walking_module_msgs_JointFeedBackGain_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_walking_module_msgs
{

  class JointFeedBackGain : public ros::Msg
  {
    public:
      typedef float _r_leg_hip_y_p_gain_type;
      _r_leg_hip_y_p_gain_type r_leg_hip_y_p_gain;
      typedef float _r_leg_hip_r_p_gain_type;
      _r_leg_hip_r_p_gain_type r_leg_hip_r_p_gain;
      typedef float _r_leg_hip_p_p_gain_type;
      _r_leg_hip_p_p_gain_type r_leg_hip_p_p_gain;
      typedef float _r_leg_kn_p_p_gain_type;
      _r_leg_kn_p_p_gain_type r_leg_kn_p_p_gain;
      typedef float _r_leg_an_p_p_gain_type;
      _r_leg_an_p_p_gain_type r_leg_an_p_p_gain;
      typedef float _r_leg_an_r_p_gain_type;
      _r_leg_an_r_p_gain_type r_leg_an_r_p_gain;
      typedef float _l_leg_hip_y_p_gain_type;
      _l_leg_hip_y_p_gain_type l_leg_hip_y_p_gain;
      typedef float _l_leg_hip_r_p_gain_type;
      _l_leg_hip_r_p_gain_type l_leg_hip_r_p_gain;
      typedef float _l_leg_hip_p_p_gain_type;
      _l_leg_hip_p_p_gain_type l_leg_hip_p_p_gain;
      typedef float _l_leg_kn_p_p_gain_type;
      _l_leg_kn_p_p_gain_type l_leg_kn_p_p_gain;
      typedef float _l_leg_an_p_p_gain_type;
      _l_leg_an_p_p_gain_type l_leg_an_p_p_gain;
      typedef float _l_leg_an_r_p_gain_type;
      _l_leg_an_r_p_gain_type l_leg_an_r_p_gain;
      typedef float _r_leg_hip_y_d_gain_type;
      _r_leg_hip_y_d_gain_type r_leg_hip_y_d_gain;
      typedef float _r_leg_hip_r_d_gain_type;
      _r_leg_hip_r_d_gain_type r_leg_hip_r_d_gain;
      typedef float _r_leg_hip_p_d_gain_type;
      _r_leg_hip_p_d_gain_type r_leg_hip_p_d_gain;
      typedef float _r_leg_kn_p_d_gain_type;
      _r_leg_kn_p_d_gain_type r_leg_kn_p_d_gain;
      typedef float _r_leg_an_p_d_gain_type;
      _r_leg_an_p_d_gain_type r_leg_an_p_d_gain;
      typedef float _r_leg_an_r_d_gain_type;
      _r_leg_an_r_d_gain_type r_leg_an_r_d_gain;
      typedef float _l_leg_hip_y_d_gain_type;
      _l_leg_hip_y_d_gain_type l_leg_hip_y_d_gain;
      typedef float _l_leg_hip_r_d_gain_type;
      _l_leg_hip_r_d_gain_type l_leg_hip_r_d_gain;
      typedef float _l_leg_hip_p_d_gain_type;
      _l_leg_hip_p_d_gain_type l_leg_hip_p_d_gain;
      typedef float _l_leg_kn_p_d_gain_type;
      _l_leg_kn_p_d_gain_type l_leg_kn_p_d_gain;
      typedef float _l_leg_an_p_d_gain_type;
      _l_leg_an_p_d_gain_type l_leg_an_p_d_gain;
      typedef float _l_leg_an_r_d_gain_type;
      _l_leg_an_r_d_gain_type l_leg_an_r_d_gain;

    JointFeedBackGain():
      r_leg_hip_y_p_gain(0),
      r_leg_hip_r_p_gain(0),
      r_leg_hip_p_p_gain(0),
      r_leg_kn_p_p_gain(0),
      r_leg_an_p_p_gain(0),
      r_leg_an_r_p_gain(0),
      l_leg_hip_y_p_gain(0),
      l_leg_hip_r_p_gain(0),
      l_leg_hip_p_p_gain(0),
      l_leg_kn_p_p_gain(0),
      l_leg_an_p_p_gain(0),
      l_leg_an_r_p_gain(0),
      r_leg_hip_y_d_gain(0),
      r_leg_hip_r_d_gain(0),
      r_leg_hip_p_d_gain(0),
      r_leg_kn_p_d_gain(0),
      r_leg_an_p_d_gain(0),
      r_leg_an_r_d_gain(0),
      l_leg_hip_y_d_gain(0),
      l_leg_hip_r_d_gain(0),
      l_leg_hip_p_d_gain(0),
      l_leg_kn_p_d_gain(0),
      l_leg_an_p_d_gain(0),
      l_leg_an_r_d_gain(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_y_p_gain;
      u_r_leg_hip_y_p_gain.real = this->r_leg_hip_y_p_gain;
      *(outbuffer + offset + 0) = (u_r_leg_hip_y_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_hip_y_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_hip_y_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_hip_y_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_hip_y_p_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_r_p_gain;
      u_r_leg_hip_r_p_gain.real = this->r_leg_hip_r_p_gain;
      *(outbuffer + offset + 0) = (u_r_leg_hip_r_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_hip_r_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_hip_r_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_hip_r_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_hip_r_p_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_p_p_gain;
      u_r_leg_hip_p_p_gain.real = this->r_leg_hip_p_p_gain;
      *(outbuffer + offset + 0) = (u_r_leg_hip_p_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_hip_p_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_hip_p_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_hip_p_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_hip_p_p_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_kn_p_p_gain;
      u_r_leg_kn_p_p_gain.real = this->r_leg_kn_p_p_gain;
      *(outbuffer + offset + 0) = (u_r_leg_kn_p_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_kn_p_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_kn_p_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_kn_p_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_kn_p_p_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_an_p_p_gain;
      u_r_leg_an_p_p_gain.real = this->r_leg_an_p_p_gain;
      *(outbuffer + offset + 0) = (u_r_leg_an_p_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_an_p_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_an_p_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_an_p_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_an_p_p_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_an_r_p_gain;
      u_r_leg_an_r_p_gain.real = this->r_leg_an_r_p_gain;
      *(outbuffer + offset + 0) = (u_r_leg_an_r_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_an_r_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_an_r_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_an_r_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_an_r_p_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_y_p_gain;
      u_l_leg_hip_y_p_gain.real = this->l_leg_hip_y_p_gain;
      *(outbuffer + offset + 0) = (u_l_leg_hip_y_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_hip_y_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_hip_y_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_hip_y_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_hip_y_p_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_r_p_gain;
      u_l_leg_hip_r_p_gain.real = this->l_leg_hip_r_p_gain;
      *(outbuffer + offset + 0) = (u_l_leg_hip_r_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_hip_r_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_hip_r_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_hip_r_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_hip_r_p_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_p_p_gain;
      u_l_leg_hip_p_p_gain.real = this->l_leg_hip_p_p_gain;
      *(outbuffer + offset + 0) = (u_l_leg_hip_p_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_hip_p_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_hip_p_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_hip_p_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_hip_p_p_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_kn_p_p_gain;
      u_l_leg_kn_p_p_gain.real = this->l_leg_kn_p_p_gain;
      *(outbuffer + offset + 0) = (u_l_leg_kn_p_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_kn_p_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_kn_p_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_kn_p_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_kn_p_p_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_an_p_p_gain;
      u_l_leg_an_p_p_gain.real = this->l_leg_an_p_p_gain;
      *(outbuffer + offset + 0) = (u_l_leg_an_p_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_an_p_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_an_p_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_an_p_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_an_p_p_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_an_r_p_gain;
      u_l_leg_an_r_p_gain.real = this->l_leg_an_r_p_gain;
      *(outbuffer + offset + 0) = (u_l_leg_an_r_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_an_r_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_an_r_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_an_r_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_an_r_p_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_y_d_gain;
      u_r_leg_hip_y_d_gain.real = this->r_leg_hip_y_d_gain;
      *(outbuffer + offset + 0) = (u_r_leg_hip_y_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_hip_y_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_hip_y_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_hip_y_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_hip_y_d_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_r_d_gain;
      u_r_leg_hip_r_d_gain.real = this->r_leg_hip_r_d_gain;
      *(outbuffer + offset + 0) = (u_r_leg_hip_r_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_hip_r_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_hip_r_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_hip_r_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_hip_r_d_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_p_d_gain;
      u_r_leg_hip_p_d_gain.real = this->r_leg_hip_p_d_gain;
      *(outbuffer + offset + 0) = (u_r_leg_hip_p_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_hip_p_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_hip_p_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_hip_p_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_hip_p_d_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_kn_p_d_gain;
      u_r_leg_kn_p_d_gain.real = this->r_leg_kn_p_d_gain;
      *(outbuffer + offset + 0) = (u_r_leg_kn_p_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_kn_p_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_kn_p_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_kn_p_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_kn_p_d_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_an_p_d_gain;
      u_r_leg_an_p_d_gain.real = this->r_leg_an_p_d_gain;
      *(outbuffer + offset + 0) = (u_r_leg_an_p_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_an_p_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_an_p_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_an_p_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_an_p_d_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_an_r_d_gain;
      u_r_leg_an_r_d_gain.real = this->r_leg_an_r_d_gain;
      *(outbuffer + offset + 0) = (u_r_leg_an_r_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_an_r_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_an_r_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_an_r_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_an_r_d_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_y_d_gain;
      u_l_leg_hip_y_d_gain.real = this->l_leg_hip_y_d_gain;
      *(outbuffer + offset + 0) = (u_l_leg_hip_y_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_hip_y_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_hip_y_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_hip_y_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_hip_y_d_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_r_d_gain;
      u_l_leg_hip_r_d_gain.real = this->l_leg_hip_r_d_gain;
      *(outbuffer + offset + 0) = (u_l_leg_hip_r_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_hip_r_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_hip_r_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_hip_r_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_hip_r_d_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_p_d_gain;
      u_l_leg_hip_p_d_gain.real = this->l_leg_hip_p_d_gain;
      *(outbuffer + offset + 0) = (u_l_leg_hip_p_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_hip_p_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_hip_p_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_hip_p_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_hip_p_d_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_kn_p_d_gain;
      u_l_leg_kn_p_d_gain.real = this->l_leg_kn_p_d_gain;
      *(outbuffer + offset + 0) = (u_l_leg_kn_p_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_kn_p_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_kn_p_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_kn_p_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_kn_p_d_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_an_p_d_gain;
      u_l_leg_an_p_d_gain.real = this->l_leg_an_p_d_gain;
      *(outbuffer + offset + 0) = (u_l_leg_an_p_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_an_p_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_an_p_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_an_p_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_an_p_d_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_an_r_d_gain;
      u_l_leg_an_r_d_gain.real = this->l_leg_an_r_d_gain;
      *(outbuffer + offset + 0) = (u_l_leg_an_r_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_an_r_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_an_r_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_an_r_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_an_r_d_gain);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_y_p_gain;
      u_r_leg_hip_y_p_gain.base = 0;
      u_r_leg_hip_y_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_hip_y_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_hip_y_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_hip_y_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_hip_y_p_gain = u_r_leg_hip_y_p_gain.real;
      offset += sizeof(this->r_leg_hip_y_p_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_r_p_gain;
      u_r_leg_hip_r_p_gain.base = 0;
      u_r_leg_hip_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_hip_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_hip_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_hip_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_hip_r_p_gain = u_r_leg_hip_r_p_gain.real;
      offset += sizeof(this->r_leg_hip_r_p_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_p_p_gain;
      u_r_leg_hip_p_p_gain.base = 0;
      u_r_leg_hip_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_hip_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_hip_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_hip_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_hip_p_p_gain = u_r_leg_hip_p_p_gain.real;
      offset += sizeof(this->r_leg_hip_p_p_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_kn_p_p_gain;
      u_r_leg_kn_p_p_gain.base = 0;
      u_r_leg_kn_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_kn_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_kn_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_kn_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_kn_p_p_gain = u_r_leg_kn_p_p_gain.real;
      offset += sizeof(this->r_leg_kn_p_p_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_an_p_p_gain;
      u_r_leg_an_p_p_gain.base = 0;
      u_r_leg_an_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_an_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_an_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_an_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_an_p_p_gain = u_r_leg_an_p_p_gain.real;
      offset += sizeof(this->r_leg_an_p_p_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_an_r_p_gain;
      u_r_leg_an_r_p_gain.base = 0;
      u_r_leg_an_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_an_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_an_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_an_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_an_r_p_gain = u_r_leg_an_r_p_gain.real;
      offset += sizeof(this->r_leg_an_r_p_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_y_p_gain;
      u_l_leg_hip_y_p_gain.base = 0;
      u_l_leg_hip_y_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_hip_y_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_hip_y_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_hip_y_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_hip_y_p_gain = u_l_leg_hip_y_p_gain.real;
      offset += sizeof(this->l_leg_hip_y_p_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_r_p_gain;
      u_l_leg_hip_r_p_gain.base = 0;
      u_l_leg_hip_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_hip_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_hip_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_hip_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_hip_r_p_gain = u_l_leg_hip_r_p_gain.real;
      offset += sizeof(this->l_leg_hip_r_p_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_p_p_gain;
      u_l_leg_hip_p_p_gain.base = 0;
      u_l_leg_hip_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_hip_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_hip_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_hip_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_hip_p_p_gain = u_l_leg_hip_p_p_gain.real;
      offset += sizeof(this->l_leg_hip_p_p_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_kn_p_p_gain;
      u_l_leg_kn_p_p_gain.base = 0;
      u_l_leg_kn_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_kn_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_kn_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_kn_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_kn_p_p_gain = u_l_leg_kn_p_p_gain.real;
      offset += sizeof(this->l_leg_kn_p_p_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_an_p_p_gain;
      u_l_leg_an_p_p_gain.base = 0;
      u_l_leg_an_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_an_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_an_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_an_p_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_an_p_p_gain = u_l_leg_an_p_p_gain.real;
      offset += sizeof(this->l_leg_an_p_p_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_an_r_p_gain;
      u_l_leg_an_r_p_gain.base = 0;
      u_l_leg_an_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_an_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_an_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_an_r_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_an_r_p_gain = u_l_leg_an_r_p_gain.real;
      offset += sizeof(this->l_leg_an_r_p_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_y_d_gain;
      u_r_leg_hip_y_d_gain.base = 0;
      u_r_leg_hip_y_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_hip_y_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_hip_y_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_hip_y_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_hip_y_d_gain = u_r_leg_hip_y_d_gain.real;
      offset += sizeof(this->r_leg_hip_y_d_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_r_d_gain;
      u_r_leg_hip_r_d_gain.base = 0;
      u_r_leg_hip_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_hip_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_hip_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_hip_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_hip_r_d_gain = u_r_leg_hip_r_d_gain.real;
      offset += sizeof(this->r_leg_hip_r_d_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_p_d_gain;
      u_r_leg_hip_p_d_gain.base = 0;
      u_r_leg_hip_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_hip_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_hip_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_hip_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_hip_p_d_gain = u_r_leg_hip_p_d_gain.real;
      offset += sizeof(this->r_leg_hip_p_d_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_kn_p_d_gain;
      u_r_leg_kn_p_d_gain.base = 0;
      u_r_leg_kn_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_kn_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_kn_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_kn_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_kn_p_d_gain = u_r_leg_kn_p_d_gain.real;
      offset += sizeof(this->r_leg_kn_p_d_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_an_p_d_gain;
      u_r_leg_an_p_d_gain.base = 0;
      u_r_leg_an_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_an_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_an_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_an_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_an_p_d_gain = u_r_leg_an_p_d_gain.real;
      offset += sizeof(this->r_leg_an_p_d_gain);
      union {
        float real;
        uint32_t base;
      } u_r_leg_an_r_d_gain;
      u_r_leg_an_r_d_gain.base = 0;
      u_r_leg_an_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_an_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_an_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_an_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_an_r_d_gain = u_r_leg_an_r_d_gain.real;
      offset += sizeof(this->r_leg_an_r_d_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_y_d_gain;
      u_l_leg_hip_y_d_gain.base = 0;
      u_l_leg_hip_y_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_hip_y_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_hip_y_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_hip_y_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_hip_y_d_gain = u_l_leg_hip_y_d_gain.real;
      offset += sizeof(this->l_leg_hip_y_d_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_r_d_gain;
      u_l_leg_hip_r_d_gain.base = 0;
      u_l_leg_hip_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_hip_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_hip_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_hip_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_hip_r_d_gain = u_l_leg_hip_r_d_gain.real;
      offset += sizeof(this->l_leg_hip_r_d_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_p_d_gain;
      u_l_leg_hip_p_d_gain.base = 0;
      u_l_leg_hip_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_hip_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_hip_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_hip_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_hip_p_d_gain = u_l_leg_hip_p_d_gain.real;
      offset += sizeof(this->l_leg_hip_p_d_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_kn_p_d_gain;
      u_l_leg_kn_p_d_gain.base = 0;
      u_l_leg_kn_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_kn_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_kn_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_kn_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_kn_p_d_gain = u_l_leg_kn_p_d_gain.real;
      offset += sizeof(this->l_leg_kn_p_d_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_an_p_d_gain;
      u_l_leg_an_p_d_gain.base = 0;
      u_l_leg_an_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_an_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_an_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_an_p_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_an_p_d_gain = u_l_leg_an_p_d_gain.real;
      offset += sizeof(this->l_leg_an_p_d_gain);
      union {
        float real;
        uint32_t base;
      } u_l_leg_an_r_d_gain;
      u_l_leg_an_r_d_gain.base = 0;
      u_l_leg_an_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_an_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_an_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_an_r_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_an_r_d_gain = u_l_leg_an_r_d_gain.real;
      offset += sizeof(this->l_leg_an_r_d_gain);
     return offset;
    }

    const char * getType(){ return "thormang3_walking_module_msgs/JointFeedBackGain"; };
    const char * getMD5(){ return "5622bf7abe0902384fe48f76fe3336ee"; };

  };

}
#endif