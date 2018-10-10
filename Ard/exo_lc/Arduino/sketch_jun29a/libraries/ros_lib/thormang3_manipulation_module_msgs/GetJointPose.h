#ifndef _ROS_SERVICE_GetJointPose_h
#define _ROS_SERVICE_GetJointPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_manipulation_module_msgs
{

static const char GETJOINTPOSE[] = "thormang3_manipulation_module_msgs/GetJointPose";

  class GetJointPoseRequest : public ros::Msg
  {
    public:
      typedef const char* _joint_name_type;
      _joint_name_type joint_name;

    GetJointPoseRequest():
      joint_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_joint_name = strlen(this->joint_name);
      varToArr(outbuffer + offset, length_joint_name);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_name, length_joint_name);
      offset += length_joint_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_joint_name;
      arrToVar(length_joint_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joint_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joint_name-1]=0;
      this->joint_name = (char *)(inbuffer + offset-1);
      offset += length_joint_name;
     return offset;
    }

    const char * getType(){ return GETJOINTPOSE; };
    const char * getMD5(){ return "0be1351618e1dc030eb7959d9a4902de"; };

  };

  class GetJointPoseResponse : public ros::Msg
  {
    public:
      typedef float _joint_value_type;
      _joint_value_type joint_value;

    GetJointPoseResponse():
      joint_value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint_value));
     return offset;
    }

    const char * getType(){ return GETJOINTPOSE; };
    const char * getMD5(){ return "6f21b5c09d11c0919f7bbdc3773565a7"; };

  };

  class GetJointPose {
    public:
    typedef GetJointPoseRequest Request;
    typedef GetJointPoseResponse Response;
  };

}
#endif
