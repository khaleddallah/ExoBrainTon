#ifndef _ROS_thormang3_head_control_module_msgs_HeadJointPose_h
#define _ROS_thormang3_head_control_module_msgs_HeadJointPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/JointState.h"

namespace thormang3_head_control_module_msgs
{

  class HeadJointPose : public ros::Msg
  {
    public:
      typedef float _mov_time_type;
      _mov_time_type mov_time;
      typedef sensor_msgs::JointState _angle_type;
      _angle_type angle;

    HeadJointPose():
      mov_time(0),
      angle()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->mov_time);
      offset += this->angle.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mov_time));
      offset += this->angle.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "thormang3_head_control_module_msgs/HeadJointPose"; };
    const char * getMD5(){ return "eab859154f9ec997f44ed17e6c2bdd4b"; };

  };

}
#endif