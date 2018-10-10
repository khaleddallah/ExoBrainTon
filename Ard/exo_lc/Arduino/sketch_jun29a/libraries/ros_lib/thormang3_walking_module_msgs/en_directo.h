#ifndef _ROS_SERVICE_en_directo_h
#define _ROS_SERVICE_en_directo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_walking_module_msgs
{

static const char EN_DIRECTO[] = "thormang3_walking_module_msgs/en_directo";

  class en_directoRequest : public ros::Msg
  {
    public:
      typedef int32_t _en_type;
      _en_type en;

    en_directoRequest():
      en(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_en;
      u_en.real = this->en;
      *(outbuffer + offset + 0) = (u_en.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_en.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_en.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_en.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->en);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_en;
      u_en.base = 0;
      u_en.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_en.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_en.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_en.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->en = u_en.real;
      offset += sizeof(this->en);
     return offset;
    }

    const char * getType(){ return EN_DIRECTO; };
    const char * getMD5(){ return "f75c8d8c86b08795c3f4826b2b417d66"; };

  };

  class en_directoResponse : public ros::Msg
  {
    public:

    en_directoResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return EN_DIRECTO; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class en_directo {
    public:
    typedef en_directoRequest Request;
    typedef en_directoResponse Response;
  };

}
#endif
