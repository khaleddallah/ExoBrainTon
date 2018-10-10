#ifndef _ROS_thormang3_walking_module_msgs_RobotPose_h
#define _ROS_thormang3_walking_module_msgs_RobotPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace thormang3_walking_module_msgs
{

  class RobotPose : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _global_to_center_of_body_type;
      _global_to_center_of_body_type global_to_center_of_body;
      typedef geometry_msgs::Pose _global_to_right_foot_type;
      _global_to_right_foot_type global_to_right_foot;
      typedef geometry_msgs::Pose _global_to_left_foot_type;
      _global_to_left_foot_type global_to_left_foot;

    RobotPose():
      global_to_center_of_body(),
      global_to_right_foot(),
      global_to_left_foot()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->global_to_center_of_body.serialize(outbuffer + offset);
      offset += this->global_to_right_foot.serialize(outbuffer + offset);
      offset += this->global_to_left_foot.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->global_to_center_of_body.deserialize(inbuffer + offset);
      offset += this->global_to_right_foot.deserialize(inbuffer + offset);
      offset += this->global_to_left_foot.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "thormang3_walking_module_msgs/RobotPose"; };
    const char * getMD5(){ return "8fdddc6841f7a8d7a6b383521406a476"; };

  };

}
#endif