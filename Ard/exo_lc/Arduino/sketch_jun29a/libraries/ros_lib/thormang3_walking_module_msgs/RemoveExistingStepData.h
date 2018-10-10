#ifndef _ROS_SERVICE_RemoveExistingStepData_h
#define _ROS_SERVICE_RemoveExistingStepData_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_walking_module_msgs
{

static const char REMOVEEXISTINGSTEPDATA[] = "thormang3_walking_module_msgs/RemoveExistingStepData";

  class RemoveExistingStepDataRequest : public ros::Msg
  {
    public:

    RemoveExistingStepDataRequest()
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

    const char * getType(){ return REMOVEEXISTINGSTEPDATA; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class RemoveExistingStepDataResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      enum { NO_ERROR =  0 };
      enum { ROBOT_IS_WALKING_NOW =  1024 };

    RemoveExistingStepDataResponse():
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

    const char * getType(){ return REMOVEEXISTINGSTEPDATA; };
    const char * getMD5(){ return "cd634758b3b5ceea79dc64fb22a0b1a0"; };

  };

  class RemoveExistingStepData {
    public:
    typedef RemoveExistingStepDataRequest Request;
    typedef RemoveExistingStepDataResponse Response;
  };

}
#endif
