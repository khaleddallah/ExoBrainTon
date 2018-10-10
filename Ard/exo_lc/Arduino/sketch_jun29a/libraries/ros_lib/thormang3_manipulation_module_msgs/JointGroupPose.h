#ifndef _ROS_thormang3_manipulation_module_msgs_JointGroupPose_h
#define _ROS_thormang3_manipulation_module_msgs_JointGroupPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/JointState.h"

namespace thormang3_manipulation_module_msgs
{

  class JointGroupPose : public ros::Msg
  {
    public:
      typedef float _mov_time_type;
      _mov_time_type mov_time;
      typedef sensor_msgs::JointState _joint_state_type;
      _joint_state_type joint_state;

    JointGroupPose():
      mov_time(0),
      joint_state()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->mov_time);
      offset += this->joint_state.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mov_time));
      offset += this->joint_state.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "thormang3_manipulation_module_msgs/JointGroupPose"; };
    const char * getMD5(){ return "899f3a4eb20263712835d205f0a70ddb"; };

  };

}
#endif