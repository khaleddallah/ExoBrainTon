#ifndef _ROS_SERVICE_GetPresentJointOffsetData_h
#define _ROS_SERVICE_GetPresentJointOffsetData_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "thormang3_offset_tuner_msgs/JointOffsetPositionData.h"

namespace thormang3_offset_tuner_msgs
{

static const char GETPRESENTJOINTOFFSETDATA[] = "thormang3_offset_tuner_msgs/GetPresentJointOffsetData";

  class GetPresentJointOffsetDataRequest : public ros::Msg
  {
    public:

    GetPresentJointOffsetDataRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETPRESENTJOINTOFFSETDATA; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetPresentJointOffsetDataResponse : public ros::Msg
  {
    public:
      uint32_t present_data_array_length;
      typedef thormang3_offset_tuner_msgs::JointOffsetPositionData _present_data_array_type;
      _present_data_array_type st_present_data_array;
      _present_data_array_type * present_data_array;

    GetPresentJointOffsetDataResponse():
      present_data_array_length(0), present_data_array(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->present_data_array_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->present_data_array_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->present_data_array_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->present_data_array_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->present_data_array_length);
      for( uint32_t i = 0; i < present_data_array_length; i++){
      offset += this->present_data_array[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t present_data_array_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      present_data_array_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      present_data_array_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      present_data_array_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->present_data_array_length);
      if(present_data_array_lengthT > present_data_array_length)
        this->present_data_array = (thormang3_offset_tuner_msgs::JointOffsetPositionData*)realloc(this->present_data_array, present_data_array_lengthT * sizeof(thormang3_offset_tuner_msgs::JointOffsetPositionData));
      present_data_array_length = present_data_array_lengthT;
      for( uint32_t i = 0; i < present_data_array_length; i++){
      offset += this->st_present_data_array.deserialize(inbuffer + offset);
        memcpy( &(this->present_data_array[i]), &(this->st_present_data_array), sizeof(thormang3_offset_tuner_msgs::JointOffsetPositionData));
      }
     return offset;
    }

    const char * getType(){ return GETPRESENTJOINTOFFSETDATA; };
    const char * getMD5(){ return "fe3cab7abbd49468a5d502064853404e"; };

  };

  class GetPresentJointOffsetData {
    public:
    typedef GetPresentJointOffsetDataRequest Request;
    typedef GetPresentJointOffsetDataResponse Response;
  };

}
#endif
