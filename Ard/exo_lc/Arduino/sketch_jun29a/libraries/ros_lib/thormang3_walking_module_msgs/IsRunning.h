#ifndef _ROS_SERVICE_IsRunning_h
#define _ROS_SERVICE_IsRunning_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_walking_module_msgs
{

static const char ISRUNNING[] = "thormang3_walking_module_msgs/IsRunning";

  class IsRunningRequest : public ros::Msg
  {
    public:

    IsRunningRequest()
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

    const char * getType(){ return ISRUNNING; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class IsRunningResponse : public ros::Msg
  {
    public:
      typedef bool _is_running_type;
      _is_running_type is_running;

    IsRunningResponse():
      is_running(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_running;
      u_is_running.real = this->is_running;
      *(outbuffer + offset + 0) = (u_is_running.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_running);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_running;
      u_is_running.base = 0;
      u_is_running.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_running = u_is_running.real;
      offset += sizeof(this->is_running);
     return offset;
    }

    const char * getType(){ return ISRUNNING; };
    const char * getMD5(){ return "ae3468a1af93d845e943210e7cef5a54"; };

  };

  class IsRunning {
    public:
    typedef IsRunningRequest Request;
    typedef IsRunningResponse Response;
  };

}
#endif
