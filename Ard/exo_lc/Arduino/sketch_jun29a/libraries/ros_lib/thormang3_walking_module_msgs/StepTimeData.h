#ifndef _ROS_thormang3_walking_module_msgs_StepTimeData_h
#define _ROS_thormang3_walking_module_msgs_StepTimeData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace thormang3_walking_module_msgs
{

  class StepTimeData : public ros::Msg
  {
    public:
      typedef int16_t _walking_state_type;
      _walking_state_type walking_state;
      typedef float _abs_step_time_type;
      _abs_step_time_type abs_step_time;
      typedef float _dsp_ratio_type;
      _dsp_ratio_type dsp_ratio;
      typedef float _start_time_delay_ratio_x_type;
      _start_time_delay_ratio_x_type start_time_delay_ratio_x;
      typedef float _start_time_delay_ratio_y_type;
      _start_time_delay_ratio_y_type start_time_delay_ratio_y;
      typedef float _start_time_delay_ratio_z_type;
      _start_time_delay_ratio_z_type start_time_delay_ratio_z;
      typedef float _start_time_delay_ratio_roll_type;
      _start_time_delay_ratio_roll_type start_time_delay_ratio_roll;
      typedef float _start_time_delay_ratio_pitch_type;
      _start_time_delay_ratio_pitch_type start_time_delay_ratio_pitch;
      typedef float _start_time_delay_ratio_yaw_type;
      _start_time_delay_ratio_yaw_type start_time_delay_ratio_yaw;
      typedef float _finish_time_advance_ratio_x_type;
      _finish_time_advance_ratio_x_type finish_time_advance_ratio_x;
      typedef float _finish_time_advance_ratio_y_type;
      _finish_time_advance_ratio_y_type finish_time_advance_ratio_y;
      typedef float _finish_time_advance_ratio_z_type;
      _finish_time_advance_ratio_z_type finish_time_advance_ratio_z;
      typedef float _finish_time_advance_ratio_roll_type;
      _finish_time_advance_ratio_roll_type finish_time_advance_ratio_roll;
      typedef float _finish_time_advance_ratio_pitch_type;
      _finish_time_advance_ratio_pitch_type finish_time_advance_ratio_pitch;
      typedef float _finish_time_advance_ratio_yaw_type;
      _finish_time_advance_ratio_yaw_type finish_time_advance_ratio_yaw;
      enum { IN_WALKING_STARTING =  0  };
      enum { IN_WALKING =  1  };
      enum { IN_WALKING_ENDING =  2  };

    StepTimeData():
      walking_state(0),
      abs_step_time(0),
      dsp_ratio(0),
      start_time_delay_ratio_x(0),
      start_time_delay_ratio_y(0),
      start_time_delay_ratio_z(0),
      start_time_delay_ratio_roll(0),
      start_time_delay_ratio_pitch(0),
      start_time_delay_ratio_yaw(0),
      finish_time_advance_ratio_x(0),
      finish_time_advance_ratio_y(0),
      finish_time_advance_ratio_z(0),
      finish_time_advance_ratio_roll(0),
      finish_time_advance_ratio_pitch(0),
      finish_time_advance_ratio_yaw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_walking_state;
      u_walking_state.real = this->walking_state;
      *(outbuffer + offset + 0) = (u_walking_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_walking_state.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->walking_state);
      union {
        float real;
        uint32_t base;
      } u_abs_step_time;
      u_abs_step_time.real = this->abs_step_time;
      *(outbuffer + offset + 0) = (u_abs_step_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_abs_step_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_abs_step_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_abs_step_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->abs_step_time);
      union {
        float real;
        uint32_t base;
      } u_dsp_ratio;
      u_dsp_ratio.real = this->dsp_ratio;
      *(outbuffer + offset + 0) = (u_dsp_ratio.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dsp_ratio.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dsp_ratio.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dsp_ratio.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dsp_ratio);
      union {
        float real;
        uint32_t base;
      } u_start_time_delay_ratio_x;
      u_start_time_delay_ratio_x.real = this->start_time_delay_ratio_x;
      *(outbuffer + offset + 0) = (u_start_time_delay_ratio_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_start_time_delay_ratio_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_start_time_delay_ratio_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_start_time_delay_ratio_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_time_delay_ratio_x);
      union {
        float real;
        uint32_t base;
      } u_start_time_delay_ratio_y;
      u_start_time_delay_ratio_y.real = this->start_time_delay_ratio_y;
      *(outbuffer + offset + 0) = (u_start_time_delay_ratio_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_start_time_delay_ratio_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_start_time_delay_ratio_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_start_time_delay_ratio_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_time_delay_ratio_y);
      union {
        float real;
        uint32_t base;
      } u_start_time_delay_ratio_z;
      u_start_time_delay_ratio_z.real = this->start_time_delay_ratio_z;
      *(outbuffer + offset + 0) = (u_start_time_delay_ratio_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_start_time_delay_ratio_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_start_time_delay_ratio_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_start_time_delay_ratio_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_time_delay_ratio_z);
      union {
        float real;
        uint32_t base;
      } u_start_time_delay_ratio_roll;
      u_start_time_delay_ratio_roll.real = this->start_time_delay_ratio_roll;
      *(outbuffer + offset + 0) = (u_start_time_delay_ratio_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_start_time_delay_ratio_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_start_time_delay_ratio_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_start_time_delay_ratio_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_time_delay_ratio_roll);
      union {
        float real;
        uint32_t base;
      } u_start_time_delay_ratio_pitch;
      u_start_time_delay_ratio_pitch.real = this->start_time_delay_ratio_pitch;
      *(outbuffer + offset + 0) = (u_start_time_delay_ratio_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_start_time_delay_ratio_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_start_time_delay_ratio_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_start_time_delay_ratio_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_time_delay_ratio_pitch);
      union {
        float real;
        uint32_t base;
      } u_start_time_delay_ratio_yaw;
      u_start_time_delay_ratio_yaw.real = this->start_time_delay_ratio_yaw;
      *(outbuffer + offset + 0) = (u_start_time_delay_ratio_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_start_time_delay_ratio_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_start_time_delay_ratio_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_start_time_delay_ratio_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_time_delay_ratio_yaw);
      union {
        float real;
        uint32_t base;
      } u_finish_time_advance_ratio_x;
      u_finish_time_advance_ratio_x.real = this->finish_time_advance_ratio_x;
      *(outbuffer + offset + 0) = (u_finish_time_advance_ratio_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_finish_time_advance_ratio_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_finish_time_advance_ratio_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_finish_time_advance_ratio_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->finish_time_advance_ratio_x);
      union {
        float real;
        uint32_t base;
      } u_finish_time_advance_ratio_y;
      u_finish_time_advance_ratio_y.real = this->finish_time_advance_ratio_y;
      *(outbuffer + offset + 0) = (u_finish_time_advance_ratio_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_finish_time_advance_ratio_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_finish_time_advance_ratio_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_finish_time_advance_ratio_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->finish_time_advance_ratio_y);
      union {
        float real;
        uint32_t base;
      } u_finish_time_advance_ratio_z;
      u_finish_time_advance_ratio_z.real = this->finish_time_advance_ratio_z;
      *(outbuffer + offset + 0) = (u_finish_time_advance_ratio_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_finish_time_advance_ratio_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_finish_time_advance_ratio_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_finish_time_advance_ratio_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->finish_time_advance_ratio_z);
      union {
        float real;
        uint32_t base;
      } u_finish_time_advance_ratio_roll;
      u_finish_time_advance_ratio_roll.real = this->finish_time_advance_ratio_roll;
      *(outbuffer + offset + 0) = (u_finish_time_advance_ratio_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_finish_time_advance_ratio_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_finish_time_advance_ratio_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_finish_time_advance_ratio_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->finish_time_advance_ratio_roll);
      union {
        float real;
        uint32_t base;
      } u_finish_time_advance_ratio_pitch;
      u_finish_time_advance_ratio_pitch.real = this->finish_time_advance_ratio_pitch;
      *(outbuffer + offset + 0) = (u_finish_time_advance_ratio_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_finish_time_advance_ratio_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_finish_time_advance_ratio_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_finish_time_advance_ratio_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->finish_time_advance_ratio_pitch);
      union {
        float real;
        uint32_t base;
      } u_finish_time_advance_ratio_yaw;
      u_finish_time_advance_ratio_yaw.real = this->finish_time_advance_ratio_yaw;
      *(outbuffer + offset + 0) = (u_finish_time_advance_ratio_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_finish_time_advance_ratio_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_finish_time_advance_ratio_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_finish_time_advance_ratio_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->finish_time_advance_ratio_yaw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_walking_state;
      u_walking_state.base = 0;
      u_walking_state.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_walking_state.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->walking_state = u_walking_state.real;
      offset += sizeof(this->walking_state);
      union {
        float real;
        uint32_t base;
      } u_abs_step_time;
      u_abs_step_time.base = 0;
      u_abs_step_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_abs_step_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_abs_step_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_abs_step_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->abs_step_time = u_abs_step_time.real;
      offset += sizeof(this->abs_step_time);
      union {
        float real;
        uint32_t base;
      } u_dsp_ratio;
      u_dsp_ratio.base = 0;
      u_dsp_ratio.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dsp_ratio.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dsp_ratio.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dsp_ratio.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dsp_ratio = u_dsp_ratio.real;
      offset += sizeof(this->dsp_ratio);
      union {
        float real;
        uint32_t base;
      } u_start_time_delay_ratio_x;
      u_start_time_delay_ratio_x.base = 0;
      u_start_time_delay_ratio_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_start_time_delay_ratio_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_start_time_delay_ratio_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_start_time_delay_ratio_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->start_time_delay_ratio_x = u_start_time_delay_ratio_x.real;
      offset += sizeof(this->start_time_delay_ratio_x);
      union {
        float real;
        uint32_t base;
      } u_start_time_delay_ratio_y;
      u_start_time_delay_ratio_y.base = 0;
      u_start_time_delay_ratio_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_start_time_delay_ratio_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_start_time_delay_ratio_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_start_time_delay_ratio_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->start_time_delay_ratio_y = u_start_time_delay_ratio_y.real;
      offset += sizeof(this->start_time_delay_ratio_y);
      union {
        float real;
        uint32_t base;
      } u_start_time_delay_ratio_z;
      u_start_time_delay_ratio_z.base = 0;
      u_start_time_delay_ratio_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_start_time_delay_ratio_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_start_time_delay_ratio_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_start_time_delay_ratio_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->start_time_delay_ratio_z = u_start_time_delay_ratio_z.real;
      offset += sizeof(this->start_time_delay_ratio_z);
      union {
        float real;
        uint32_t base;
      } u_start_time_delay_ratio_roll;
      u_start_time_delay_ratio_roll.base = 0;
      u_start_time_delay_ratio_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_start_time_delay_ratio_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_start_time_delay_ratio_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_start_time_delay_ratio_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->start_time_delay_ratio_roll = u_start_time_delay_ratio_roll.real;
      offset += sizeof(this->start_time_delay_ratio_roll);
      union {
        float real;
        uint32_t base;
      } u_start_time_delay_ratio_pitch;
      u_start_time_delay_ratio_pitch.base = 0;
      u_start_time_delay_ratio_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_start_time_delay_ratio_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_start_time_delay_ratio_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_start_time_delay_ratio_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->start_time_delay_ratio_pitch = u_start_time_delay_ratio_pitch.real;
      offset += sizeof(this->start_time_delay_ratio_pitch);
      union {
        float real;
        uint32_t base;
      } u_start_time_delay_ratio_yaw;
      u_start_time_delay_ratio_yaw.base = 0;
      u_start_time_delay_ratio_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_start_time_delay_ratio_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_start_time_delay_ratio_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_start_time_delay_ratio_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->start_time_delay_ratio_yaw = u_start_time_delay_ratio_yaw.real;
      offset += sizeof(this->start_time_delay_ratio_yaw);
      union {
        float real;
        uint32_t base;
      } u_finish_time_advance_ratio_x;
      u_finish_time_advance_ratio_x.base = 0;
      u_finish_time_advance_ratio_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_finish_time_advance_ratio_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_finish_time_advance_ratio_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_finish_time_advance_ratio_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->finish_time_advance_ratio_x = u_finish_time_advance_ratio_x.real;
      offset += sizeof(this->finish_time_advance_ratio_x);
      union {
        float real;
        uint32_t base;
      } u_finish_time_advance_ratio_y;
      u_finish_time_advance_ratio_y.base = 0;
      u_finish_time_advance_ratio_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_finish_time_advance_ratio_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_finish_time_advance_ratio_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_finish_time_advance_ratio_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->finish_time_advance_ratio_y = u_finish_time_advance_ratio_y.real;
      offset += sizeof(this->finish_time_advance_ratio_y);
      union {
        float real;
        uint32_t base;
      } u_finish_time_advance_ratio_z;
      u_finish_time_advance_ratio_z.base = 0;
      u_finish_time_advance_ratio_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_finish_time_advance_ratio_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_finish_time_advance_ratio_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_finish_time_advance_ratio_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->finish_time_advance_ratio_z = u_finish_time_advance_ratio_z.real;
      offset += sizeof(this->finish_time_advance_ratio_z);
      union {
        float real;
        uint32_t base;
      } u_finish_time_advance_ratio_roll;
      u_finish_time_advance_ratio_roll.base = 0;
      u_finish_time_advance_ratio_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_finish_time_advance_ratio_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_finish_time_advance_ratio_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_finish_time_advance_ratio_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->finish_time_advance_ratio_roll = u_finish_time_advance_ratio_roll.real;
      offset += sizeof(this->finish_time_advance_ratio_roll);
      union {
        float real;
        uint32_t base;
      } u_finish_time_advance_ratio_pitch;
      u_finish_time_advance_ratio_pitch.base = 0;
      u_finish_time_advance_ratio_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_finish_time_advance_ratio_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_finish_time_advance_ratio_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_finish_time_advance_ratio_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->finish_time_advance_ratio_pitch = u_finish_time_advance_ratio_pitch.real;
      offset += sizeof(this->finish_time_advance_ratio_pitch);
      union {
        float real;
        uint32_t base;
      } u_finish_time_advance_ratio_yaw;
      u_finish_time_advance_ratio_yaw.base = 0;
      u_finish_time_advance_ratio_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_finish_time_advance_ratio_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_finish_time_advance_ratio_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_finish_time_advance_ratio_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->finish_time_advance_ratio_yaw = u_finish_time_advance_ratio_yaw.real;
      offset += sizeof(this->finish_time_advance_ratio_yaw);
     return offset;
    }

    const char * getType(){ return "thormang3_walking_module_msgs/StepTimeData"; };
    const char * getMD5(){ return "31818d93beb7ae1b2c26ce96b0285f66"; };

  };

}
#endif