#ifndef _ROS_thormang3_foot_step_generator_FootStepCommand_h
#define _ROS_thormang3_foot_step_generator_FootStepCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_foot_step_generator
{

  class FootStepCommand : public ros::Msg
  {
    public:
      typedef const char* _command_type;
      _command_type command;
      typedef int32_t _step_num_type;
      _step_num_type step_num;
      typedef float _step_time_type;
      _step_time_type step_time;
      typedef float _step_length_type;
      _step_length_type step_length;
      typedef float _side_step_length_type;
      _side_step_length_type side_step_length;
      typedef float _step_angle_rad_type;
      _step_angle_rad_type step_angle_rad;

    FootStepCommand():
      command(""),
      step_num(0),
      step_time(0),
      step_length(0),
      side_step_length(0),
      step_angle_rad(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      union {
        int32_t real;
        uint32_t base;
      } u_step_num;
      u_step_num.real = this->step_num;
      *(outbuffer + offset + 0) = (u_step_num.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_step_num.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_step_num.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_step_num.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->step_num);
      offset += serializeAvrFloat64(outbuffer + offset, this->step_time);
      offset += serializeAvrFloat64(outbuffer + offset, this->step_length);
      offset += serializeAvrFloat64(outbuffer + offset, this->side_step_length);
      offset += serializeAvrFloat64(outbuffer + offset, this->step_angle_rad);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
      union {
        int32_t real;
        uint32_t base;
      } u_step_num;
      u_step_num.base = 0;
      u_step_num.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_step_num.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_step_num.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_step_num.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->step_num = u_step_num.real;
      offset += sizeof(this->step_num);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->step_time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->step_length));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->side_step_length));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->step_angle_rad));
     return offset;
    }

    const char * getType(){ return "thormang3_foot_step_generator/FootStepCommand"; };
    const char * getMD5(){ return "4091bcbe2dc1a5c91f6da6b96beb0965"; };

  };

}
#endif