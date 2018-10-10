#ifndef _ROS_thormang3_walking_module_msgs_StepPositionData_h
#define _ROS_thormang3_walking_module_msgs_StepPositionData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "thormang3_walking_module_msgs/PoseXYZRPY.h"
#include "thormang3_walking_module_msgs/PoseZRPY.h"

namespace thormang3_walking_module_msgs
{

  class StepPositionData : public ros::Msg
  {
    public:
      typedef int16_t _moving_foot_type;
      _moving_foot_type moving_foot;
      typedef float _foot_z_swap_type;
      _foot_z_swap_type foot_z_swap;
      typedef float _body_z_swap_type;
      _body_z_swap_type body_z_swap;
      typedef float _x_zmp_shift_type;
      _x_zmp_shift_type x_zmp_shift;
      typedef float _y_zmp_shift_type;
      _y_zmp_shift_type y_zmp_shift;
      typedef float _torso_yaw_angle_rad_type;
      _torso_yaw_angle_rad_type torso_yaw_angle_rad;
      typedef thormang3_walking_module_msgs::PoseXYZRPY _left_foot_pose_type;
      _left_foot_pose_type left_foot_pose;
      typedef thormang3_walking_module_msgs::PoseXYZRPY _right_foot_pose_type;
      _right_foot_pose_type right_foot_pose;
      typedef thormang3_walking_module_msgs::PoseZRPY _body_pose_type;
      _body_pose_type body_pose;
      enum { LEFT_FOOT_SWING =  1  };
      enum { RIGHT_FOOT_SWING =  2  };
      enum { STANDING =  3  };

    StepPositionData():
      moving_foot(0),
      foot_z_swap(0),
      body_z_swap(0),
      x_zmp_shift(0),
      y_zmp_shift(0),
      torso_yaw_angle_rad(0),
      left_foot_pose(),
      right_foot_pose(),
      body_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_moving_foot;
      u_moving_foot.real = this->moving_foot;
      *(outbuffer + offset + 0) = (u_moving_foot.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_moving_foot.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->moving_foot);
      union {
        float real;
        uint32_t base;
      } u_foot_z_swap;
      u_foot_z_swap.real = this->foot_z_swap;
      *(outbuffer + offset + 0) = (u_foot_z_swap.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foot_z_swap.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foot_z_swap.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foot_z_swap.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foot_z_swap);
      union {
        float real;
        uint32_t base;
      } u_body_z_swap;
      u_body_z_swap.real = this->body_z_swap;
      *(outbuffer + offset + 0) = (u_body_z_swap.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_body_z_swap.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_body_z_swap.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_body_z_swap.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->body_z_swap);
      union {
        float real;
        uint32_t base;
      } u_x_zmp_shift;
      u_x_zmp_shift.real = this->x_zmp_shift;
      *(outbuffer + offset + 0) = (u_x_zmp_shift.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_zmp_shift.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_zmp_shift.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_zmp_shift.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_zmp_shift);
      union {
        float real;
        uint32_t base;
      } u_y_zmp_shift;
      u_y_zmp_shift.real = this->y_zmp_shift;
      *(outbuffer + offset + 0) = (u_y_zmp_shift.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_zmp_shift.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_zmp_shift.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_zmp_shift.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_zmp_shift);
      union {
        float real;
        uint32_t base;
      } u_torso_yaw_angle_rad;
      u_torso_yaw_angle_rad.real = this->torso_yaw_angle_rad;
      *(outbuffer + offset + 0) = (u_torso_yaw_angle_rad.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torso_yaw_angle_rad.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torso_yaw_angle_rad.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torso_yaw_angle_rad.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torso_yaw_angle_rad);
      offset += this->left_foot_pose.serialize(outbuffer + offset);
      offset += this->right_foot_pose.serialize(outbuffer + offset);
      offset += this->body_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_moving_foot;
      u_moving_foot.base = 0;
      u_moving_foot.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_moving_foot.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->moving_foot = u_moving_foot.real;
      offset += sizeof(this->moving_foot);
      union {
        float real;
        uint32_t base;
      } u_foot_z_swap;
      u_foot_z_swap.base = 0;
      u_foot_z_swap.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foot_z_swap.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foot_z_swap.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foot_z_swap.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foot_z_swap = u_foot_z_swap.real;
      offset += sizeof(this->foot_z_swap);
      union {
        float real;
        uint32_t base;
      } u_body_z_swap;
      u_body_z_swap.base = 0;
      u_body_z_swap.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_body_z_swap.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_body_z_swap.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_body_z_swap.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->body_z_swap = u_body_z_swap.real;
      offset += sizeof(this->body_z_swap);
      union {
        float real;
        uint32_t base;
      } u_x_zmp_shift;
      u_x_zmp_shift.base = 0;
      u_x_zmp_shift.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_zmp_shift.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_zmp_shift.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_zmp_shift.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_zmp_shift = u_x_zmp_shift.real;
      offset += sizeof(this->x_zmp_shift);
      union {
        float real;
        uint32_t base;
      } u_y_zmp_shift;
      u_y_zmp_shift.base = 0;
      u_y_zmp_shift.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_zmp_shift.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_zmp_shift.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_zmp_shift.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_zmp_shift = u_y_zmp_shift.real;
      offset += sizeof(this->y_zmp_shift);
      union {
        float real;
        uint32_t base;
      } u_torso_yaw_angle_rad;
      u_torso_yaw_angle_rad.base = 0;
      u_torso_yaw_angle_rad.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_torso_yaw_angle_rad.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_torso_yaw_angle_rad.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_torso_yaw_angle_rad.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->torso_yaw_angle_rad = u_torso_yaw_angle_rad.real;
      offset += sizeof(this->torso_yaw_angle_rad);
      offset += this->left_foot_pose.deserialize(inbuffer + offset);
      offset += this->right_foot_pose.deserialize(inbuffer + offset);
      offset += this->body_pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "thormang3_walking_module_msgs/StepPositionData"; };
    const char * getMD5(){ return "88a7ca881b40058b9b8edc224fb86a74"; };

  };

}
#endif