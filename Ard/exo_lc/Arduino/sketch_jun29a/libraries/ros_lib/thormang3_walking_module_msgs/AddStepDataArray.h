#ifndef _ROS_SERVICE_AddStepDataArray_h
#define _ROS_SERVICE_AddStepDataArray_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "thormang3_walking_module_msgs/StepData.h"

namespace thormang3_walking_module_msgs
{

static const char ADDSTEPDATAARRAY[] = "thormang3_walking_module_msgs/AddStepDataArray";

  class AddStepDataArrayRequest : public ros::Msg
  {
    public:
      typedef bool _auto_start_type;
      _auto_start_type auto_start;
      typedef bool _remove_existing_step_data_type;
      _remove_existing_step_data_type remove_existing_step_data;
      uint32_t step_data_array_length;
      typedef thormang3_walking_module_msgs::StepData _step_data_array_type;
      _step_data_array_type st_step_data_array;
      _step_data_array_type * step_data_array;

    AddStepDataArrayRequest():
      auto_start(0),
      remove_existing_step_data(0),
      step_data_array_length(0), step_data_array(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_auto_start;
      u_auto_start.real = this->auto_start;
      *(outbuffer + offset + 0) = (u_auto_start.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->auto_start);
      union {
        bool real;
        uint8_t base;
      } u_remove_existing_step_data;
      u_remove_existing_step_data.real = this->remove_existing_step_data;
      *(outbuffer + offset + 0) = (u_remove_existing_step_data.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->remove_existing_step_data);
      *(outbuffer + offset + 0) = (this->step_data_array_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->step_data_array_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->step_data_array_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->step_data_array_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->step_data_array_length);
      for( uint32_t i = 0; i < step_data_array_length; i++){
      offset += this->step_data_array[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_auto_start;
      u_auto_start.base = 0;
      u_auto_start.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->auto_start = u_auto_start.real;
      offset += sizeof(this->auto_start);
      union {
        bool real;
        uint8_t base;
      } u_remove_existing_step_data;
      u_remove_existing_step_data.base = 0;
      u_remove_existing_step_data.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->remove_existing_step_data = u_remove_existing_step_data.real;
      offset += sizeof(this->remove_existing_step_data);
      uint32_t step_data_array_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      step_data_array_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      step_data_array_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      step_data_array_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->step_data_array_length);
      if(step_data_array_lengthT > step_data_array_length)
        this->step_data_array = (thormang3_walking_module_msgs::StepData*)realloc(this->step_data_array, step_data_array_lengthT * sizeof(thormang3_walking_module_msgs::StepData));
      step_data_array_length = step_data_array_lengthT;
      for( uint32_t i = 0; i < step_data_array_length; i++){
      offset += this->st_step_data_array.deserialize(inbuffer + offset);
        memcpy( &(this->step_data_array[i]), &(this->st_step_data_array), sizeof(thormang3_walking_module_msgs::StepData));
      }
     return offset;
    }

    const char * getType(){ return ADDSTEPDATAARRAY; };
    const char * getMD5(){ return "ceb2a926f880ff0d3107efdc25c41f8c"; };

  };

  class AddStepDataArrayResponse : public ros::Msg
  {
    public:
      typedef int32_t _result_type;
      _result_type result;
      enum { NO_ERROR =  0 };
      enum { NOT_ENABLED_WALKING_MODULE =  2 };
      enum { PROBLEM_IN_POSITION_DATA =  4 };
      enum { PROBLEM_IN_TIME_DATA =  8 };
      enum { TOO_MANY_STEP_DATA =  128 };
      enum { ROBOT_IS_WALKING_NOW =  1024 };

    AddStepDataArrayResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_result.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_result.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_result.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_result.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    const char * getType(){ return ADDSTEPDATAARRAY; };
    const char * getMD5(){ return "0f88c15da5fc32c19b6aa84196f9ee87"; };

  };

  class AddStepDataArray {
    public:
    typedef AddStepDataArrayRequest Request;
    typedef AddStepDataArrayResponse Response;
  };

}
#endif
