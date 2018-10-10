#ifndef _ROS_thormang3_walking_module_msgs_StepData_h
#define _ROS_thormang3_walking_module_msgs_StepData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "thormang3_walking_module_msgs/StepPositionData.h"
#include "thormang3_walking_module_msgs/StepTimeData.h"

namespace thormang3_walking_module_msgs
{

  class StepData : public ros::Msg
  {
    public:
      typedef thormang3_walking_module_msgs::StepPositionData _position_data_type;
      _position_data_type position_data;
      typedef thormang3_walking_module_msgs::StepTimeData _time_data_type;
      _time_data_type time_data;

    StepData():
      position_data(),
      time_data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->position_data.serialize(outbuffer + offset);
      offset += this->time_data.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->position_data.deserialize(inbuffer + offset);
      offset += this->time_data.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "thormang3_walking_module_msgs/StepData"; };
    const char * getMD5(){ return "5c123ac289a192451dda1d18cbffa1a0"; };

  };

}
#endif