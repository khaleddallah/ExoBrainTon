#ifndef _ROS_thormang3_walking_module_msgs_WalkingJointStatesStamped_h
#define _ROS_thormang3_walking_module_msgs_WalkingJointStatesStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace thormang3_walking_module_msgs
{

  class WalkingJointStatesStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _r_goal_hip_y_type;
      _r_goal_hip_y_type r_goal_hip_y;
      typedef float _r_goal_hip_r_type;
      _r_goal_hip_r_type r_goal_hip_r;
      typedef float _r_goal_hip_p_type;
      _r_goal_hip_p_type r_goal_hip_p;
      typedef float _r_goal_kn_p_type;
      _r_goal_kn_p_type r_goal_kn_p;
      typedef float _r_goal_an_p_type;
      _r_goal_an_p_type r_goal_an_p;
      typedef float _r_goal_an_r_type;
      _r_goal_an_r_type r_goal_an_r;
      typedef float _l_goal_hip_y_type;
      _l_goal_hip_y_type l_goal_hip_y;
      typedef float _l_goal_hip_r_type;
      _l_goal_hip_r_type l_goal_hip_r;
      typedef float _l_goal_hip_p_type;
      _l_goal_hip_p_type l_goal_hip_p;
      typedef float _l_goal_kn_p_type;
      _l_goal_kn_p_type l_goal_kn_p;
      typedef float _l_goal_an_p_type;
      _l_goal_an_p_type l_goal_an_p;
      typedef float _l_goal_an_r_type;
      _l_goal_an_r_type l_goal_an_r;
      typedef float _r_present_hip_y_type;
      _r_present_hip_y_type r_present_hip_y;
      typedef float _r_present_hip_r_type;
      _r_present_hip_r_type r_present_hip_r;
      typedef float _r_present_hip_p_type;
      _r_present_hip_p_type r_present_hip_p;
      typedef float _r_present_kn_p_type;
      _r_present_kn_p_type r_present_kn_p;
      typedef float _r_present_an_p_type;
      _r_present_an_p_type r_present_an_p;
      typedef float _r_present_an_r_type;
      _r_present_an_r_type r_present_an_r;
      typedef float _l_present_hip_y_type;
      _l_present_hip_y_type l_present_hip_y;
      typedef float _l_present_hip_r_type;
      _l_present_hip_r_type l_present_hip_r;
      typedef float _l_present_hip_p_type;
      _l_present_hip_p_type l_present_hip_p;
      typedef float _l_present_kn_p_type;
      _l_present_kn_p_type l_present_kn_p;
      typedef float _l_present_an_p_type;
      _l_present_an_p_type l_present_an_p;
      typedef float _l_present_an_r_type;
      _l_present_an_r_type l_present_an_r;

    WalkingJointStatesStamped():
      header(),
      r_goal_hip_y(0),
      r_goal_hip_r(0),
      r_goal_hip_p(0),
      r_goal_kn_p(0),
      r_goal_an_p(0),
      r_goal_an_r(0),
      l_goal_hip_y(0),
      l_goal_hip_r(0),
      l_goal_hip_p(0),
      l_goal_kn_p(0),
      l_goal_an_p(0),
      l_goal_an_r(0),
      r_present_hip_y(0),
      r_present_hip_r(0),
      r_present_hip_p(0),
      r_present_kn_p(0),
      r_present_an_p(0),
      r_present_an_r(0),
      l_present_hip_y(0),
      l_present_hip_r(0),
      l_present_hip_p(0),
      l_present_kn_p(0),
      l_present_an_p(0),
      l_present_an_r(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_r_goal_hip_y;
      u_r_goal_hip_y.real = this->r_goal_hip_y;
      *(outbuffer + offset + 0) = (u_r_goal_hip_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_goal_hip_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_goal_hip_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_goal_hip_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_goal_hip_y);
      union {
        float real;
        uint32_t base;
      } u_r_goal_hip_r;
      u_r_goal_hip_r.real = this->r_goal_hip_r;
      *(outbuffer + offset + 0) = (u_r_goal_hip_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_goal_hip_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_goal_hip_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_goal_hip_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_goal_hip_r);
      union {
        float real;
        uint32_t base;
      } u_r_goal_hip_p;
      u_r_goal_hip_p.real = this->r_goal_hip_p;
      *(outbuffer + offset + 0) = (u_r_goal_hip_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_goal_hip_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_goal_hip_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_goal_hip_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_goal_hip_p);
      union {
        float real;
        uint32_t base;
      } u_r_goal_kn_p;
      u_r_goal_kn_p.real = this->r_goal_kn_p;
      *(outbuffer + offset + 0) = (u_r_goal_kn_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_goal_kn_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_goal_kn_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_goal_kn_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_goal_kn_p);
      union {
        float real;
        uint32_t base;
      } u_r_goal_an_p;
      u_r_goal_an_p.real = this->r_goal_an_p;
      *(outbuffer + offset + 0) = (u_r_goal_an_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_goal_an_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_goal_an_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_goal_an_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_goal_an_p);
      union {
        float real;
        uint32_t base;
      } u_r_goal_an_r;
      u_r_goal_an_r.real = this->r_goal_an_r;
      *(outbuffer + offset + 0) = (u_r_goal_an_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_goal_an_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_goal_an_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_goal_an_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_goal_an_r);
      union {
        float real;
        uint32_t base;
      } u_l_goal_hip_y;
      u_l_goal_hip_y.real = this->l_goal_hip_y;
      *(outbuffer + offset + 0) = (u_l_goal_hip_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_goal_hip_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_goal_hip_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_goal_hip_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_goal_hip_y);
      union {
        float real;
        uint32_t base;
      } u_l_goal_hip_r;
      u_l_goal_hip_r.real = this->l_goal_hip_r;
      *(outbuffer + offset + 0) = (u_l_goal_hip_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_goal_hip_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_goal_hip_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_goal_hip_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_goal_hip_r);
      union {
        float real;
        uint32_t base;
      } u_l_goal_hip_p;
      u_l_goal_hip_p.real = this->l_goal_hip_p;
      *(outbuffer + offset + 0) = (u_l_goal_hip_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_goal_hip_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_goal_hip_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_goal_hip_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_goal_hip_p);
      union {
        float real;
        uint32_t base;
      } u_l_goal_kn_p;
      u_l_goal_kn_p.real = this->l_goal_kn_p;
      *(outbuffer + offset + 0) = (u_l_goal_kn_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_goal_kn_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_goal_kn_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_goal_kn_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_goal_kn_p);
      union {
        float real;
        uint32_t base;
      } u_l_goal_an_p;
      u_l_goal_an_p.real = this->l_goal_an_p;
      *(outbuffer + offset + 0) = (u_l_goal_an_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_goal_an_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_goal_an_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_goal_an_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_goal_an_p);
      union {
        float real;
        uint32_t base;
      } u_l_goal_an_r;
      u_l_goal_an_r.real = this->l_goal_an_r;
      *(outbuffer + offset + 0) = (u_l_goal_an_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_goal_an_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_goal_an_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_goal_an_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_goal_an_r);
      union {
        float real;
        uint32_t base;
      } u_r_present_hip_y;
      u_r_present_hip_y.real = this->r_present_hip_y;
      *(outbuffer + offset + 0) = (u_r_present_hip_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_present_hip_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_present_hip_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_present_hip_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_present_hip_y);
      union {
        float real;
        uint32_t base;
      } u_r_present_hip_r;
      u_r_present_hip_r.real = this->r_present_hip_r;
      *(outbuffer + offset + 0) = (u_r_present_hip_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_present_hip_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_present_hip_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_present_hip_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_present_hip_r);
      union {
        float real;
        uint32_t base;
      } u_r_present_hip_p;
      u_r_present_hip_p.real = this->r_present_hip_p;
      *(outbuffer + offset + 0) = (u_r_present_hip_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_present_hip_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_present_hip_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_present_hip_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_present_hip_p);
      union {
        float real;
        uint32_t base;
      } u_r_present_kn_p;
      u_r_present_kn_p.real = this->r_present_kn_p;
      *(outbuffer + offset + 0) = (u_r_present_kn_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_present_kn_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_present_kn_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_present_kn_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_present_kn_p);
      union {
        float real;
        uint32_t base;
      } u_r_present_an_p;
      u_r_present_an_p.real = this->r_present_an_p;
      *(outbuffer + offset + 0) = (u_r_present_an_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_present_an_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_present_an_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_present_an_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_present_an_p);
      union {
        float real;
        uint32_t base;
      } u_r_present_an_r;
      u_r_present_an_r.real = this->r_present_an_r;
      *(outbuffer + offset + 0) = (u_r_present_an_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_present_an_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_present_an_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_present_an_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_present_an_r);
      union {
        float real;
        uint32_t base;
      } u_l_present_hip_y;
      u_l_present_hip_y.real = this->l_present_hip_y;
      *(outbuffer + offset + 0) = (u_l_present_hip_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_present_hip_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_present_hip_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_present_hip_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_present_hip_y);
      union {
        float real;
        uint32_t base;
      } u_l_present_hip_r;
      u_l_present_hip_r.real = this->l_present_hip_r;
      *(outbuffer + offset + 0) = (u_l_present_hip_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_present_hip_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_present_hip_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_present_hip_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_present_hip_r);
      union {
        float real;
        uint32_t base;
      } u_l_present_hip_p;
      u_l_present_hip_p.real = this->l_present_hip_p;
      *(outbuffer + offset + 0) = (u_l_present_hip_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_present_hip_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_present_hip_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_present_hip_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_present_hip_p);
      union {
        float real;
        uint32_t base;
      } u_l_present_kn_p;
      u_l_present_kn_p.real = this->l_present_kn_p;
      *(outbuffer + offset + 0) = (u_l_present_kn_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_present_kn_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_present_kn_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_present_kn_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_present_kn_p);
      union {
        float real;
        uint32_t base;
      } u_l_present_an_p;
      u_l_present_an_p.real = this->l_present_an_p;
      *(outbuffer + offset + 0) = (u_l_present_an_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_present_an_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_present_an_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_present_an_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_present_an_p);
      union {
        float real;
        uint32_t base;
      } u_l_present_an_r;
      u_l_present_an_r.real = this->l_present_an_r;
      *(outbuffer + offset + 0) = (u_l_present_an_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_present_an_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_present_an_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_present_an_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_present_an_r);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_r_goal_hip_y;
      u_r_goal_hip_y.base = 0;
      u_r_goal_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_goal_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_goal_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_goal_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_goal_hip_y = u_r_goal_hip_y.real;
      offset += sizeof(this->r_goal_hip_y);
      union {
        float real;
        uint32_t base;
      } u_r_goal_hip_r;
      u_r_goal_hip_r.base = 0;
      u_r_goal_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_goal_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_goal_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_goal_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_goal_hip_r = u_r_goal_hip_r.real;
      offset += sizeof(this->r_goal_hip_r);
      union {
        float real;
        uint32_t base;
      } u_r_goal_hip_p;
      u_r_goal_hip_p.base = 0;
      u_r_goal_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_goal_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_goal_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_goal_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_goal_hip_p = u_r_goal_hip_p.real;
      offset += sizeof(this->r_goal_hip_p);
      union {
        float real;
        uint32_t base;
      } u_r_goal_kn_p;
      u_r_goal_kn_p.base = 0;
      u_r_goal_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_goal_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_goal_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_goal_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_goal_kn_p = u_r_goal_kn_p.real;
      offset += sizeof(this->r_goal_kn_p);
      union {
        float real;
        uint32_t base;
      } u_r_goal_an_p;
      u_r_goal_an_p.base = 0;
      u_r_goal_an_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_goal_an_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_goal_an_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_goal_an_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_goal_an_p = u_r_goal_an_p.real;
      offset += sizeof(this->r_goal_an_p);
      union {
        float real;
        uint32_t base;
      } u_r_goal_an_r;
      u_r_goal_an_r.base = 0;
      u_r_goal_an_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_goal_an_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_goal_an_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_goal_an_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_goal_an_r = u_r_goal_an_r.real;
      offset += sizeof(this->r_goal_an_r);
      union {
        float real;
        uint32_t base;
      } u_l_goal_hip_y;
      u_l_goal_hip_y.base = 0;
      u_l_goal_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_goal_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_goal_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_goal_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_goal_hip_y = u_l_goal_hip_y.real;
      offset += sizeof(this->l_goal_hip_y);
      union {
        float real;
        uint32_t base;
      } u_l_goal_hip_r;
      u_l_goal_hip_r.base = 0;
      u_l_goal_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_goal_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_goal_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_goal_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_goal_hip_r = u_l_goal_hip_r.real;
      offset += sizeof(this->l_goal_hip_r);
      union {
        float real;
        uint32_t base;
      } u_l_goal_hip_p;
      u_l_goal_hip_p.base = 0;
      u_l_goal_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_goal_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_goal_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_goal_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_goal_hip_p = u_l_goal_hip_p.real;
      offset += sizeof(this->l_goal_hip_p);
      union {
        float real;
        uint32_t base;
      } u_l_goal_kn_p;
      u_l_goal_kn_p.base = 0;
      u_l_goal_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_goal_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_goal_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_goal_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_goal_kn_p = u_l_goal_kn_p.real;
      offset += sizeof(this->l_goal_kn_p);
      union {
        float real;
        uint32_t base;
      } u_l_goal_an_p;
      u_l_goal_an_p.base = 0;
      u_l_goal_an_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_goal_an_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_goal_an_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_goal_an_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_goal_an_p = u_l_goal_an_p.real;
      offset += sizeof(this->l_goal_an_p);
      union {
        float real;
        uint32_t base;
      } u_l_goal_an_r;
      u_l_goal_an_r.base = 0;
      u_l_goal_an_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_goal_an_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_goal_an_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_goal_an_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_goal_an_r = u_l_goal_an_r.real;
      offset += sizeof(this->l_goal_an_r);
      union {
        float real;
        uint32_t base;
      } u_r_present_hip_y;
      u_r_present_hip_y.base = 0;
      u_r_present_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_present_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_present_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_present_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_present_hip_y = u_r_present_hip_y.real;
      offset += sizeof(this->r_present_hip_y);
      union {
        float real;
        uint32_t base;
      } u_r_present_hip_r;
      u_r_present_hip_r.base = 0;
      u_r_present_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_present_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_present_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_present_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_present_hip_r = u_r_present_hip_r.real;
      offset += sizeof(this->r_present_hip_r);
      union {
        float real;
        uint32_t base;
      } u_r_present_hip_p;
      u_r_present_hip_p.base = 0;
      u_r_present_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_present_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_present_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_present_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_present_hip_p = u_r_present_hip_p.real;
      offset += sizeof(this->r_present_hip_p);
      union {
        float real;
        uint32_t base;
      } u_r_present_kn_p;
      u_r_present_kn_p.base = 0;
      u_r_present_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_present_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_present_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_present_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_present_kn_p = u_r_present_kn_p.real;
      offset += sizeof(this->r_present_kn_p);
      union {
        float real;
        uint32_t base;
      } u_r_present_an_p;
      u_r_present_an_p.base = 0;
      u_r_present_an_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_present_an_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_present_an_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_present_an_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_present_an_p = u_r_present_an_p.real;
      offset += sizeof(this->r_present_an_p);
      union {
        float real;
        uint32_t base;
      } u_r_present_an_r;
      u_r_present_an_r.base = 0;
      u_r_present_an_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_present_an_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_present_an_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_present_an_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_present_an_r = u_r_present_an_r.real;
      offset += sizeof(this->r_present_an_r);
      union {
        float real;
        uint32_t base;
      } u_l_present_hip_y;
      u_l_present_hip_y.base = 0;
      u_l_present_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_present_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_present_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_present_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_present_hip_y = u_l_present_hip_y.real;
      offset += sizeof(this->l_present_hip_y);
      union {
        float real;
        uint32_t base;
      } u_l_present_hip_r;
      u_l_present_hip_r.base = 0;
      u_l_present_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_present_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_present_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_present_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_present_hip_r = u_l_present_hip_r.real;
      offset += sizeof(this->l_present_hip_r);
      union {
        float real;
        uint32_t base;
      } u_l_present_hip_p;
      u_l_present_hip_p.base = 0;
      u_l_present_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_present_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_present_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_present_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_present_hip_p = u_l_present_hip_p.real;
      offset += sizeof(this->l_present_hip_p);
      union {
        float real;
        uint32_t base;
      } u_l_present_kn_p;
      u_l_present_kn_p.base = 0;
      u_l_present_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_present_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_present_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_present_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_present_kn_p = u_l_present_kn_p.real;
      offset += sizeof(this->l_present_kn_p);
      union {
        float real;
        uint32_t base;
      } u_l_present_an_p;
      u_l_present_an_p.base = 0;
      u_l_present_an_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_present_an_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_present_an_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_present_an_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_present_an_p = u_l_present_an_p.real;
      offset += sizeof(this->l_present_an_p);
      union {
        float real;
        uint32_t base;
      } u_l_present_an_r;
      u_l_present_an_r.base = 0;
      u_l_present_an_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_present_an_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_present_an_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_present_an_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_present_an_r = u_l_present_an_r.real;
      offset += sizeof(this->l_present_an_r);
     return offset;
    }

    const char * getType(){ return "thormang3_walking_module_msgs/WalkingJointStatesStamped"; };
    const char * getMD5(){ return "9405084c6a71cf9fde2eabd3642df1d4"; };

  };

}
#endif