#ifndef _ROS_thormang3_walking_module_msgs_ResultExo_h
#define _ROS_thormang3_walking_module_msgs_ResultExo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_walking_module_msgs
{

  class ResultExo : public ros::Msg
  {
    public:
      typedef float _r_leg_hip_y_type;
      _r_leg_hip_y_type r_leg_hip_y;
      typedef float _r_leg_hip_r_type;
      _r_leg_hip_r_type r_leg_hip_r;
      typedef float _r_leg_hip_p_type;
      _r_leg_hip_p_type r_leg_hip_p;
      typedef float _r_leg_kn_p_type;
      _r_leg_kn_p_type r_leg_kn_p;
      typedef float _r_leg_an_p_type;
      _r_leg_an_p_type r_leg_an_p;
      typedef float _r_leg_an_r_type;
      _r_leg_an_r_type r_leg_an_r;
      typedef float _l_leg_hip_y_type;
      _l_leg_hip_y_type l_leg_hip_y;
      typedef float _l_leg_hip_r_type;
      _l_leg_hip_r_type l_leg_hip_r;
      typedef float _l_leg_hip_p_type;
      _l_leg_hip_p_type l_leg_hip_p;
      typedef float _l_leg_kn_p_type;
      _l_leg_kn_p_type l_leg_kn_p;
      typedef float _l_leg_an_p_type;
      _l_leg_an_p_type l_leg_an_p;
      typedef float _l_leg_an_r_type;
      _l_leg_an_r_type l_leg_an_r;

    ResultExo():
      r_leg_hip_y(0),
      r_leg_hip_r(0),
      r_leg_hip_p(0),
      r_leg_kn_p(0),
      r_leg_an_p(0),
      r_leg_an_r(0),
      l_leg_hip_y(0),
      l_leg_hip_r(0),
      l_leg_hip_p(0),
      l_leg_kn_p(0),
      l_leg_an_p(0),
      l_leg_an_r(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_y;
      u_r_leg_hip_y.real = this->r_leg_hip_y;
      *(outbuffer + offset + 0) = (u_r_leg_hip_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_hip_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_hip_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_hip_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_hip_y);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_r;
      u_r_leg_hip_r.real = this->r_leg_hip_r;
      *(outbuffer + offset + 0) = (u_r_leg_hip_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_hip_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_hip_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_hip_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_hip_r);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_p;
      u_r_leg_hip_p.real = this->r_leg_hip_p;
      *(outbuffer + offset + 0) = (u_r_leg_hip_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_hip_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_hip_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_hip_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_hip_p);
      union {
        float real;
        uint32_t base;
      } u_r_leg_kn_p;
      u_r_leg_kn_p.real = this->r_leg_kn_p;
      *(outbuffer + offset + 0) = (u_r_leg_kn_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_kn_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_kn_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_kn_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_kn_p);
      union {
        float real;
        uint32_t base;
      } u_r_leg_an_p;
      u_r_leg_an_p.real = this->r_leg_an_p;
      *(outbuffer + offset + 0) = (u_r_leg_an_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_an_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_an_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_an_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_an_p);
      union {
        float real;
        uint32_t base;
      } u_r_leg_an_r;
      u_r_leg_an_r.real = this->r_leg_an_r;
      *(outbuffer + offset + 0) = (u_r_leg_an_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_leg_an_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_leg_an_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_leg_an_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_leg_an_r);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_y;
      u_l_leg_hip_y.real = this->l_leg_hip_y;
      *(outbuffer + offset + 0) = (u_l_leg_hip_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_hip_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_hip_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_hip_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_hip_y);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_r;
      u_l_leg_hip_r.real = this->l_leg_hip_r;
      *(outbuffer + offset + 0) = (u_l_leg_hip_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_hip_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_hip_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_hip_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_hip_r);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_p;
      u_l_leg_hip_p.real = this->l_leg_hip_p;
      *(outbuffer + offset + 0) = (u_l_leg_hip_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_hip_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_hip_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_hip_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_hip_p);
      union {
        float real;
        uint32_t base;
      } u_l_leg_kn_p;
      u_l_leg_kn_p.real = this->l_leg_kn_p;
      *(outbuffer + offset + 0) = (u_l_leg_kn_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_kn_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_kn_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_kn_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_kn_p);
      union {
        float real;
        uint32_t base;
      } u_l_leg_an_p;
      u_l_leg_an_p.real = this->l_leg_an_p;
      *(outbuffer + offset + 0) = (u_l_leg_an_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_an_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_an_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_an_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_an_p);
      union {
        float real;
        uint32_t base;
      } u_l_leg_an_r;
      u_l_leg_an_r.real = this->l_leg_an_r;
      *(outbuffer + offset + 0) = (u_l_leg_an_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_leg_an_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_leg_an_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_leg_an_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_leg_an_r);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_y;
      u_r_leg_hip_y.base = 0;
      u_r_leg_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_hip_y = u_r_leg_hip_y.real;
      offset += sizeof(this->r_leg_hip_y);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_r;
      u_r_leg_hip_r.base = 0;
      u_r_leg_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_hip_r = u_r_leg_hip_r.real;
      offset += sizeof(this->r_leg_hip_r);
      union {
        float real;
        uint32_t base;
      } u_r_leg_hip_p;
      u_r_leg_hip_p.base = 0;
      u_r_leg_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_hip_p = u_r_leg_hip_p.real;
      offset += sizeof(this->r_leg_hip_p);
      union {
        float real;
        uint32_t base;
      } u_r_leg_kn_p;
      u_r_leg_kn_p.base = 0;
      u_r_leg_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_kn_p = u_r_leg_kn_p.real;
      offset += sizeof(this->r_leg_kn_p);
      union {
        float real;
        uint32_t base;
      } u_r_leg_an_p;
      u_r_leg_an_p.base = 0;
      u_r_leg_an_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_an_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_an_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_an_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_an_p = u_r_leg_an_p.real;
      offset += sizeof(this->r_leg_an_p);
      union {
        float real;
        uint32_t base;
      } u_r_leg_an_r;
      u_r_leg_an_r.base = 0;
      u_r_leg_an_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_leg_an_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_leg_an_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_leg_an_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_leg_an_r = u_r_leg_an_r.real;
      offset += sizeof(this->r_leg_an_r);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_y;
      u_l_leg_hip_y.base = 0;
      u_l_leg_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_hip_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_hip_y = u_l_leg_hip_y.real;
      offset += sizeof(this->l_leg_hip_y);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_r;
      u_l_leg_hip_r.base = 0;
      u_l_leg_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_hip_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_hip_r = u_l_leg_hip_r.real;
      offset += sizeof(this->l_leg_hip_r);
      union {
        float real;
        uint32_t base;
      } u_l_leg_hip_p;
      u_l_leg_hip_p.base = 0;
      u_l_leg_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_hip_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_hip_p = u_l_leg_hip_p.real;
      offset += sizeof(this->l_leg_hip_p);
      union {
        float real;
        uint32_t base;
      } u_l_leg_kn_p;
      u_l_leg_kn_p.base = 0;
      u_l_leg_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_kn_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_kn_p = u_l_leg_kn_p.real;
      offset += sizeof(this->l_leg_kn_p);
      union {
        float real;
        uint32_t base;
      } u_l_leg_an_p;
      u_l_leg_an_p.base = 0;
      u_l_leg_an_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_an_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_an_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_an_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_an_p = u_l_leg_an_p.real;
      offset += sizeof(this->l_leg_an_p);
      union {
        float real;
        uint32_t base;
      } u_l_leg_an_r;
      u_l_leg_an_r.base = 0;
      u_l_leg_an_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_leg_an_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_leg_an_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_leg_an_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_leg_an_r = u_l_leg_an_r.real;
      offset += sizeof(this->l_leg_an_r);
     return offset;
    }

    const char * getType(){ return "thormang3_walking_module_msgs/ResultExo"; };
    const char * getMD5(){ return "040f0744414461004e42f108975983ea"; };

  };

}
#endif