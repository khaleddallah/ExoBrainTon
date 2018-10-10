#ifndef _ROS_thormang3_foot_step_generator_Step2D_h
#define _ROS_thormang3_foot_step_generator_Step2D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose2D.h"

namespace thormang3_foot_step_generator
{

  class Step2D : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose2D _step2d_type;
      _step2d_type step2d;
      typedef uint8_t _moving_foot_type;
      _moving_foot_type moving_foot;
      enum { LEFT_FOOT_SWING =  1  };
      enum { RIGHT_FOOT_SWING =  2  };
      enum { STANDING =  3  };

    Step2D():
      step2d(),
      moving_foot(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->step2d.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->moving_foot >> (8 * 0)) & 0xFF;
      offset += sizeof(this->moving_foot);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->step2d.deserialize(inbuffer + offset);
      this->moving_foot =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->moving_foot);
     return offset;
    }

    const char * getType(){ return "thormang3_foot_step_generator/Step2D"; };
    const char * getMD5(){ return "8b716dffcd181458918724c59549dd00"; };

  };

}
#endif