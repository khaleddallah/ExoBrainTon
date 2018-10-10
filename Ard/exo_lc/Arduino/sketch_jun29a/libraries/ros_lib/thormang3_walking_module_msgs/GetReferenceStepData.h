#ifndef _ROS_SERVICE_GetReferenceStepData_h
#define _ROS_SERVICE_GetReferenceStepData_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "thormang3_walking_module_msgs/StepData.h"

namespace thormang3_walking_module_msgs
{

static const char GETREFERENCESTEPDATA[] = "thormang3_walking_module_msgs/GetReferenceStepData";

  class GetReferenceStepDataRequest : public ros::Msg
  {
    public:

    GetReferenceStepDataRequest()
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

    const char * getType(){ return GETREFERENCESTEPDATA; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetReferenceStepDataResponse : public ros::Msg
  {
    public:
      typedef thormang3_walking_module_msgs::StepData _reference_step_data_type;
      _reference_step_data_type reference_step_data;

    GetReferenceStepDataResponse():
      reference_step_data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->reference_step_data.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->reference_step_data.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETREFERENCESTEPDATA; };
    const char * getMD5(){ return "26366ec2bde813e29e6c0fc4afdd2038"; };

  };

  class GetReferenceStepData {
    public:
    typedef GetReferenceStepDataRequest Request;
    typedef GetReferenceStepDataResponse Response;
  };

}
#endif
