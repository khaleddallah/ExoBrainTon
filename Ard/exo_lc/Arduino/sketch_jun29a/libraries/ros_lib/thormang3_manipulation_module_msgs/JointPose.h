#ifndef _ROS_thormang3_manipulation_module_msgs_JointPose_h
#define _ROS_thormang3_manipulation_module_msgs_JointPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_manipulation_module_msgs
{

  class JointPose : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef float _value_type;
      _value_type value;
      typedef float _time_type;
      _time_type time;

    JointPose():
      name(""),
      value(0),
      time(0)
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
      offset += serializeAvrFloat64(outbuffer + offset, this->value);
      offset += serializeAvrFloat64(outbuffer + offset, this->time);
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
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->value));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time));
     return offset;
    }

    const char * getType(){ return "thormang3_manipulation_module_msgs/JointPose"; };
    const char * getMD5(){ return "c62f067356d062eeed2059f97624ab9d"; };

  };

}
#endif