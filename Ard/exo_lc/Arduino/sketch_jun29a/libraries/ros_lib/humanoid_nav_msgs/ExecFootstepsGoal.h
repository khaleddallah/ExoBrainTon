#ifndef _ROS_humanoid_nav_msgs_ExecFootstepsGoal_h
#define _ROS_humanoid_nav_msgs_ExecFootstepsGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "humanoid_nav_msgs/StepTarget.h"

namespace humanoid_nav_msgs
{

  class ExecFootstepsGoal : public ros::Msg
  {
    public:
      uint32_t footsteps_length;
      typedef humanoid_nav_msgs::StepTarget _footsteps_type;
      _footsteps_type st_footsteps;
      _footsteps_type * footsteps;
      typedef float _feedback_frequency_type;
      _feedback_frequency_type feedback_frequency;

    ExecFootstepsGoal():
      footsteps_length(0), footsteps(NULL),
      feedback_frequency(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->footsteps_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->footsteps_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->footsteps_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->footsteps_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->footsteps_length);
      for( uint32_t i = 0; i < footsteps_length; i++){
      offset += this->footsteps[i].serialize(outbuffer + offset);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->feedback_frequency);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t footsteps_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      footsteps_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      footsteps_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      footsteps_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->footsteps_length);
      if(footsteps_lengthT > footsteps_length)
        this->footsteps = (humanoid_nav_msgs::StepTarget*)realloc(this->footsteps, footsteps_lengthT * sizeof(humanoid_nav_msgs::StepTarget));
      footsteps_length = footsteps_lengthT;
      for( uint32_t i = 0; i < footsteps_length; i++){
      offset += this->st_footsteps.deserialize(inbuffer + offset);
        memcpy( &(this->footsteps[i]), &(this->st_footsteps), sizeof(humanoid_nav_msgs::StepTarget));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->feedback_frequency));
     return offset;
    }

    const char * getType(){ return "humanoid_nav_msgs/ExecFootstepsGoal"; };
    const char * getMD5(){ return "40a3f8092ef3bb49c3253baa6eb94932"; };

  };

}
#endif