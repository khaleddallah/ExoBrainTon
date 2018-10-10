#ifndef _ROS_thormang3_feet_ft_module_msgs_BothWrench_h
#define _ROS_thormang3_feet_ft_module_msgs_BothWrench_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Wrench.h"

namespace thormang3_feet_ft_module_msgs
{

  class BothWrench : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef geometry_msgs::Wrench _right_type;
      _right_type right;
      typedef geometry_msgs::Wrench _left_type;
      _left_type left;

    BothWrench():
      name(""),
      right(),
      left()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += this->right.serialize(outbuffer + offset);
      offset += this->left.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += this->right.deserialize(inbuffer + offset);
      offset += this->left.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "thormang3_feet_ft_module_msgs/BothWrench"; };
    const char * getMD5(){ return "9e88bfefeefe18c3ce328fedf7d3089b"; };

  };

}
#endif