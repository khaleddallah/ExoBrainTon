#ifndef _ROS_SERVICE_StartWalking_h
#define _ROS_SERVICE_StartWalking_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_walking_module_msgs
{

static const char STARTWALKING[] = "thormang3_walking_module_msgs/StartWalking";

  class StartWalkingRequest : public ros::Msg
  {
    public:

    StartWalkingRequest()
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

    const char * getType(){ return STARTWALKING; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class StartWalkingResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      enum { NO_ERROR =  0 };
      enum { NOT_ENABLED_WALKING_MODULE =  2 };
      enum { NO_STEP_DATA =  16 };
      enum { ROBOT_IS_WALKING_NOW =  1024 };

    StartWalkingResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_result.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_result.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_result.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    const char * getType(){ return STARTWALKING; };
    const char * getMD5(){ return "4705489a13b0ea78117047acc1833787"; };

  };

  class StartWalking {
    public:
    typedef StartWalkingRequest Request;
    typedef StartWalkingResponse Response;
  };

}
#endif
