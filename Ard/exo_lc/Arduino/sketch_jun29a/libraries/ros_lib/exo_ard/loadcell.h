#ifndef _ROS_exo_ard_loadcell_h
#define _ROS_exo_ard_loadcell_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace exo_ard
{

  class loadcell : public ros::Msg
  {
    public:
      typedef float _ld1_type;
      _ld1_type ld1;
      typedef float _ld2_type;
      _ld2_type ld2;
      typedef float _ld3_type;
      _ld3_type ld3;
      typedef float _ld4_type;
      _ld4_type ld4;

    loadcell():
      ld1(0),
      ld2(0),
      ld3(0),
      ld4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_ld1;
      u_ld1.real = this->ld1;
      *(outbuffer + offset + 0) = (u_ld1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ld1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ld1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ld1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ld1);
      union {
        float real;
        uint32_t base;
      } u_ld2;
      u_ld2.real = this->ld2;
      *(outbuffer + offset + 0) = (u_ld2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ld2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ld2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ld2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ld2);
      union {
        float real;
        uint32_t base;
      } u_ld3;
      u_ld3.real = this->ld3;
      *(outbuffer + offset + 0) = (u_ld3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ld3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ld3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ld3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ld3);
      union {
        float real;
        uint32_t base;
      } u_ld4;
      u_ld4.real = this->ld4;
      *(outbuffer + offset + 0) = (u_ld4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ld4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ld4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ld4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ld4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_ld1;
      u_ld1.base = 0;
      u_ld1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ld1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ld1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ld1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ld1 = u_ld1.real;
      offset += sizeof(this->ld1);
      union {
        float real;
        uint32_t base;
      } u_ld2;
      u_ld2.base = 0;
      u_ld2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ld2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ld2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ld2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ld2 = u_ld2.real;
      offset += sizeof(this->ld2);
      union {
        float real;
        uint32_t base;
      } u_ld3;
      u_ld3.base = 0;
      u_ld3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ld3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ld3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ld3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ld3 = u_ld3.real;
      offset += sizeof(this->ld3);
      union {
        float real;
        uint32_t base;
      } u_ld4;
      u_ld4.base = 0;
      u_ld4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ld4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ld4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ld4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ld4 = u_ld4.real;
      offset += sizeof(this->ld4);
     return offset;
    }

    const char * getType(){ return "exo_ard/loadcell"; };
    const char * getMD5(){ return "a0f52c784aa6f225b8db36501a001bf7"; };

  };

}
#endif