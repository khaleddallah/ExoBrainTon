#ifndef _ROS_thormang3_foot_step_generator_Step2DArray_h
#define _ROS_thormang3_foot_step_generator_Step2DArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "thormang3_foot_step_generator/Step2D.h"

namespace thormang3_foot_step_generator
{

  class Step2DArray : public ros::Msg
  {
    public:
      uint32_t footsteps_2d_length;
      typedef thormang3_foot_step_generator::Step2D _footsteps_2d_type;
      _footsteps_2d_type st_footsteps_2d;
      _footsteps_2d_type * footsteps_2d;

    Step2DArray():
      footsteps_2d_length(0), footsteps_2d(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->footsteps_2d_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->footsteps_2d_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->footsteps_2d_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->footsteps_2d_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->footsteps_2d_length);
      for( uint32_t i = 0; i < footsteps_2d_length; i++){
      offset += this->footsteps_2d[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t footsteps_2d_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      footsteps_2d_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      footsteps_2d_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      footsteps_2d_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->footsteps_2d_length);
      if(footsteps_2d_lengthT > footsteps_2d_length)
        this->footsteps_2d = (thormang3_foot_step_generator::Step2D*)realloc(this->footsteps_2d, footsteps_2d_lengthT * sizeof(thormang3_foot_step_generator::Step2D));
      footsteps_2d_length = footsteps_2d_lengthT;
      for( uint32_t i = 0; i < footsteps_2d_length; i++){
      offset += this->st_footsteps_2d.deserialize(inbuffer + offset);
        memcpy( &(this->footsteps_2d[i]), &(this->st_footsteps_2d), sizeof(thormang3_foot_step_generator::Step2D));
      }
     return offset;
    }

    const char * getType(){ return "thormang3_foot_step_generator/Step2DArray"; };
    const char * getMD5(){ return "f258e5cefa9165c31378996d01ab01cd"; };

  };

}
#endif