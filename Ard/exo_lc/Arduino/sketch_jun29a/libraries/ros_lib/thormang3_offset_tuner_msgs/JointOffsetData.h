#ifndef _ROS_thormang3_offset_tuner_msgs_JointOffsetData_h
#define _ROS_thormang3_offset_tuner_msgs_JointOffsetData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_offset_tuner_msgs
{

  class JointOffsetData : public ros::Msg
  {
    public:
      typedef const char* _joint_name_type;
      _joint_name_type joint_name;
      typedef float _goal_value_type;
      _goal_value_type goal_value;
      typedef float _offset_value_type;
      _offset_value_type offset_value;
      typedef int32_t _p_gain_type;
      _p_gain_type p_gain;
      typedef int32_t _i_gain_type;
      _i_gain_type i_gain;
      typedef int32_t _d_gain_type;
      _d_gain_type d_gain;

    JointOffsetData():
      joint_name(""),
      goal_value(0),
      offset_value(0),
      p_gain(0),
      i_gain(0),
      d_gain(0)
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
      offset += serializeAvrFloat64(outbuffer + offset, this->goal_value);
      offset += serializeAvrFloat64(outbuffer + offset, this->offset_value);
      union {
        int32_t real;
        uint32_t base;
      } u_p_gain;
      u_p_gain.real = this->p_gain;
      *(outbuffer + offset + 0) = (u_p_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_p_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_p_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_p_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p_gain);
      union {
        int32_t real;
        uint32_t base;
      } u_i_gain;
      u_i_gain.real = this->i_gain;
      *(outbuffer + offset + 0) = (u_i_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_gain);
      union {
        int32_t real;
        uint32_t base;
      } u_d_gain;
      u_d_gain.real = this->d_gain;
      *(outbuffer + offset + 0) = (u_d_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_d_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_d_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_d_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->d_gain);
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
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->goal_value));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->offset_value));
      union {
        int32_t real;
        uint32_t base;
      } u_p_gain;
      u_p_gain.base = 0;
      u_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->p_gain = u_p_gain.real;
      offset += sizeof(this->p_gain);
      union {
        int32_t real;
        uint32_t base;
      } u_i_gain;
      u_i_gain.base = 0;
      u_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->i_gain = u_i_gain.real;
      offset += sizeof(this->i_gain);
      union {
        int32_t real;
        uint32_t base;
      } u_d_gain;
      u_d_gain.base = 0;
      u_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->d_gain = u_d_gain.real;
      offset += sizeof(this->d_gain);
     return offset;
    }

    const char * getType(){ return "thormang3_offset_tuner_msgs/JointOffsetData"; };
    const char * getMD5(){ return "f2ced0a4562683b45a9aab2f0e3e1f84"; };

  };

}
#endif