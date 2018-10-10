#ifndef _ROS_thormang3_offset_tuner_msgs_JointTorqueOnOffArray_h
#define _ROS_thormang3_offset_tuner_msgs_JointTorqueOnOffArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "thormang3_offset_tuner_msgs/JointTorqueOnOff.h"

namespace thormang3_offset_tuner_msgs
{

  class JointTorqueOnOffArray : public ros::Msg
  {
    public:
      uint32_t torque_enable_data_length;
      typedef thormang3_offset_tuner_msgs::JointTorqueOnOff _torque_enable_data_type;
      _torque_enable_data_type st_torque_enable_data;
      _torque_enable_data_type * torque_enable_data;

    JointTorqueOnOffArray():
      torque_enable_data_length(0), torque_enable_data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->torque_enable_data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->torque_enable_data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->torque_enable_data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->torque_enable_data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torque_enable_data_length);
      for( uint32_t i = 0; i < torque_enable_data_length; i++){
      offset += this->torque_enable_data[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t torque_enable_data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      torque_enable_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      torque_enable_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      torque_enable_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->torque_enable_data_length);
      if(torque_enable_data_lengthT > torque_enable_data_length)
        this->torque_enable_data = (thormang3_offset_tuner_msgs::JointTorqueOnOff*)realloc(this->torque_enable_data, torque_enable_data_lengthT * sizeof(thormang3_offset_tuner_msgs::JointTorqueOnOff));
      torque_enable_data_length = torque_enable_data_lengthT;
      for( uint32_t i = 0; i < torque_enable_data_length; i++){
      offset += this->st_torque_enable_data.deserialize(inbuffer + offset);
        memcpy( &(this->torque_enable_data[i]), &(this->st_torque_enable_data), sizeof(thormang3_offset_tuner_msgs::JointTorqueOnOff));
      }
     return offset;
    }

    const char * getType(){ return "thormang3_offset_tuner_msgs/JointTorqueOnOffArray"; };
    const char * getMD5(){ return "1ca4db772b4d802ac00aebf4469fc8bf"; };

  };

}
#endif