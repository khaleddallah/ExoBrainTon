#ifndef _ROS_marker_msgs_MarkerWithCovarianceStamped_h
#define _ROS_marker_msgs_MarkerWithCovarianceStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "marker_msgs/MarkerWithCovariance.h"

namespace marker_msgs
{

  class MarkerWithCovarianceStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef marker_msgs::MarkerWithCovariance _marker_type;
      _marker_type marker;

    MarkerWithCovarianceStamped():
      header(),
      marker()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->marker.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->marker.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "marker_msgs/MarkerWithCovarianceStamped"; };
    const char * getMD5(){ return "1c6b4e4db4f1f1f63303a467ab44aef6"; };

  };

}
#endif