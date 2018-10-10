#ifndef _ROS_thormang3_action_module_msgs_StartAction_h
#define _ROS_thormang3_action_module_msgs_StartAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_action_module_msgs
{

  class StartAction : public ros::Msg
  {
    public:
      typedef int32_t _page_num_type;
      _page_num_type page_num;
      uint32_t joint_name_array_length;
      typedef char* _joint_name_array_type;
      _joint_name_array_type st_joint_name_array;
      _joint_name_array_type * joint_name_array;

    StartAction():
      page_num(0),
      joint_name_array_length(0), joint_name_array(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_page_num;
      u_page_num.real = this->page_num;
      *(outbuffer + offset + 0) = (u_page_num.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_page_num.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_page_num.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_page_num.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->page_num);
      *(outbuffer + offset + 0) = (this->joint_name_array_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_name_array_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_name_array_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_name_array_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_name_array_length);
      for( uint32_t i = 0; i < joint_name_array_length; i++){
      uint32_t length_joint_name_arrayi = strlen(this->joint_name_array[i]);
      varToArr(outbuffer + offset, length_joint_name_arrayi);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_name_array[i], length_joint_name_arrayi);
      offset += length_joint_name_arrayi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_page_num;
      u_page_num.base = 0;
      u_page_num.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_page_num.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_page_num.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_page_num.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->page_num = u_page_num.real;
      offset += sizeof(this->page_num);
      uint32_t joint_name_array_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_name_array_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_name_array_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_name_array_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_name_array_length);
      if(joint_name_array_lengthT > joint_name_array_length)
        this->joint_name_array = (char**)realloc(this->joint_name_array, joint_name_array_lengthT * sizeof(char*));
      joint_name_array_length = joint_name_array_lengthT;
      for( uint32_t i = 0; i < joint_name_array_length; i++){
      uint32_t length_st_joint_name_array;
      arrToVar(length_st_joint_name_array, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_name_array; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_name_array-1]=0;
      this->st_joint_name_array = (char *)(inbuffer + offset-1);
      offset += length_st_joint_name_array;
        memcpy( &(this->joint_name_array[i]), &(this->st_joint_name_array), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "thormang3_action_module_msgs/StartAction"; };
    const char * getMD5(){ return "089f02f04489a5eddf9886b2ae161539"; };

  };

}
#endif