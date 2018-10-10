#ifndef _ROS_SERVICE_SetDampingBalanceParam_h
#define _ROS_SERVICE_SetDampingBalanceParam_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "thormang3_walking_module_msgs/DampingBalanceParam.h"

namespace thormang3_walking_module_msgs
{

static const char SETDAMPINGBALANCEPARAM[] = "thormang3_walking_module_msgs/SetDampingBalanceParam";

  class SetDampingBalanceParamRequest : public ros::Msg
  {
    public:
      typedef float _updating_duration_type;
      _updating_duration_type updating_duration;
      typedef thormang3_walking_module_msgs::DampingBalanceParam _balance_param_type;
      _balance_param_type balance_param;

    SetDampingBalanceParamRequest():
      updating_duration(0),
      balance_param()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_updating_duration;
      u_updating_duration.real = this->updating_duration;
      *(outbuffer + offset + 0) = (u_updating_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_updating_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_updating_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_updating_duration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->updating_duration);
      offset += this->balance_param.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_updating_duration;
      u_updating_duration.base = 0;
      u_updating_duration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_updating_duration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_updating_duration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_updating_duration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->updating_duration = u_updating_duration.real;
      offset += sizeof(this->updating_duration);
      offset += this->balance_param.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETDAMPINGBALANCEPARAM; };
    const char * getMD5(){ return "234831e0dbeee478991bb6b08c613563"; };

  };

  class SetDampingBalanceParamResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      enum { NO_ERROR =  0 };
      enum { NOT_ENABLED_WALKING_MODULE =  2 };
      enum { PREV_REQUEST_IS_NOT_FINISHED =  32 };
      enum { TIME_CONST_IS_ZERO_OR_NEGATIVE =  64 };

    SetDampingBalanceParamResponse():
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

    const char * getType(){ return SETDAMPINGBALANCEPARAM; };
    const char * getMD5(){ return "f3cad264072acca4b3b4f81fd31a92dd"; };

  };

  class SetDampingBalanceParam {
    public:
    typedef SetDampingBalanceParamRequest Request;
    typedef SetDampingBalanceParamResponse Response;
  };

}
#endif
