/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/


// GripperModule::GripperModule()


// void change_param::ChangeParamCallback(const thormang3_walking_module_msgs::SetBalanceParam & msg)
// {
//   if (msg != last)
//   {
//     present = msg ;
//     setBalanceParamService();
//     last=present;
//   }
// }


// void change_param::setBalanceParamService( )
// {
//   if(set_balance_param_client.call(set_balance_param_srv) == true)
//   {
//     int lastresult = set_balance_param_srv.response.result;
//     if( lastresult == thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR)
//       ROS_INFO("[Demo]  : Succeed to set balance param");
//     else
//     {
//       if(lastresult & thormang3_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE)
//         ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE");
//       if(lastresult & thormang3_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED)
//         ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED");
// //      if(lastresult & thormang3_walking_module_msgs::SetBalanceParam::Response::CUT_OFF_FREQUENCY_IS_ZERO_OR_NEGATIVE)
// //        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE");
//     }
//   }
//   else
//   {
//     ROS_ERROR("[Demo]  : Failed to set balance param ");
//   }
// }


// change_param::change_param(){
// }


// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "change_param");
//     change_param cp;
//     ros::NodeHandle n;
//     ros::Subscriber sub = n.subscribe("/change_param", 1000, cp.ChangeParamCallback);
//     ros::ServiceClient  set_balance_param_client;
//     set_balance_param_client  = n.serviceClient<thormang3_walking_module_msgs::SetBalanceParam>("/robotis/walking/set_balance_param");


//     ros::spin();

//     return 0 ; 
// }


//###################################################
#include <stdio.h>
#include "changer/changer.h"

// #include <dynamic_reconfigure/server.h>
// #include <changer/paramConfig.h> 



using namespace thormang3;



changerM::changerM()
  : control_cycle_msec_(8)
{
  enable_       = false;
  module_name_  = "changerM"; // set unique module name
  //control_mode_ = robotis_framework::PositionControl;

  //result_["r_sho_pitch"] = new robotis_framework::DynamixelState();
  //result_["r_sho_roll"] = new robotis_framework::DynamixelState();
  //result_["r_el"] = new robotis_framework::DynamixelState();
  //f = boost::bind(&thormang3::changer::callback_dr, _1, _2);
  //server.setCallback(f);


}

changerM::~changerM()
{
  queue_thread_.join();
}

void changerM::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&changerM::queueThread, this));
  //queue_thread_ = boost::thread(f);
  //f=boost::bind(&callback_dr, _1 , _2);
  //queue_thread_ = boost::thread(f);
    /*rd*/
//  queue_thread_ = boost::thread(f)
  //server.setCallback(f);

}

void changerM::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* dynamic */
  dynamic_reconfigure::Server<changer::MyParamConfig> server;
  dynamic_reconfigure::Server<changer::MyParamConfig>::CallbackType f;
  f = boost::bind(&changerM::callback,this, _1 , _2);
  server.setCallback(f);


  /* subscriber */
  changeSub = ros_node.subscribe("/change_param", 10, &changerM::ChangeParamSubCallback, this);

  /* services */
  set_balance_param_client  = ros_node.serviceClient<thormang3_walking_module_msgs::SetBalanceParam>("/robotis/walking/set_balance_param");

  // ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  // while(ros_node.ok())
  //   callback_queue.callAvailable(duration);
  ros::spin();

}

void changerM::ChangeParamSubCallback(const thormang3_walking_module_msgs::BalanceParam & msg)
{
  if (   last.cob_x_offset_m                      != msg.cob_x_offset_m
      || last.cob_y_offset_m                      != msg.cob_y_offset_m     
      || last.hip_roll_swap_angle_rad             != msg.hip_roll_swap_angle_rad
      || last.foot_roll_gyro_p_gain               != msg.foot_roll_gyro_p_gain
      || last.foot_roll_gyro_d_gain               != msg.foot_roll_gyro_d_gain 
      || last.foot_pitch_gyro_p_gain              != msg.foot_pitch_gyro_p_gain    
      || last.foot_pitch_gyro_d_gain              != msg.foot_pitch_gyro_d_gain              
      || last.foot_roll_angle_p_gain              != msg.foot_roll_angle_p_gain             
      || last.foot_roll_angle_d_gain              != msg.foot_roll_angle_d_gain     
      || last.foot_pitch_angle_p_gain             != msg.foot_pitch_angle_p_gain
      || last.foot_pitch_angle_d_gain             != msg.foot_pitch_angle_d_gain
      || last.foot_x_force_p_gain                 != msg.foot_x_force_p_gain            
      || last.foot_x_force_d_gain                 != msg.foot_x_force_d_gain                 
      || last.foot_y_force_p_gain                 != msg.foot_y_force_p_gain 
      || last.foot_y_force_d_gain                 != msg.foot_y_force_d_gain                 
      || last.foot_z_force_p_gain                 != msg.foot_z_force_p_gain                 
      || last.foot_z_force_d_gain                 != msg.foot_z_force_d_gain                 
      || last.foot_roll_torque_p_gain             != msg.foot_roll_torque_p_gain             
      || last.foot_roll_torque_d_gain             != msg.foot_roll_torque_d_gain            
      || last.foot_pitch_torque_p_gain            != msg.foot_pitch_torque_p_gain
      || last.foot_pitch_torque_d_gain            != msg.foot_pitch_torque_d_gain           
      || last.roll_gyro_cut_off_frequency         != msg.roll_gyro_cut_off_frequency
      || last.pitch_gyro_cut_off_frequency        != msg.pitch_gyro_cut_off_frequency
      || last.roll_angle_cut_off_frequency        != msg.roll_angle_cut_off_frequency        
      || last.pitch_angle_cut_off_frequency       != msg.pitch_angle_cut_off_frequency 
      || last.foot_x_force_cut_off_frequency      != msg.foot_x_force_cut_off_frequency     
      || last.foot_y_force_cut_off_frequency      != msg.foot_y_force_cut_off_frequency     
      || last.foot_z_force_cut_off_frequency      != msg.foot_z_force_cut_off_frequency
      || last.foot_roll_torque_cut_off_frequency  != msg.foot_roll_torque_cut_off_frequency
      || last.foot_pitch_torque_cut_off_frequency != msg.foot_pitch_torque_cut_off_frequency
  )
  {
    present = msg ;
    setBalanceParamService();
    ROS_INFO("cococococ");
    last=present;
  }
}


void changerM::setBalanceParamService( )
{
   thormang3_walking_module_msgs::SetBalanceParam set_balance_param_srv;
   set_balance_param_srv.request.balance_param = present;
  if(set_balance_param_client.call(set_balance_param_srv) == true)
  {
    int lastresult = set_balance_param_srv.response.result;
    if( lastresult == thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR)
      ROS_INFO("[Demo]  : Succeed to set balance param");
    else
    {
      if(lastresult & thormang3_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE");
      if(lastresult & thormang3_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED)
        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED");
//      if(lastresult & thormang3_walking_module_msgs::SetBalanceParam::Response::CUT_OFF_FREQUENCY_IS_ZERO_OR_NEGATIVE)
//        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE");
    }
  }
  else
  {
    ROS_ERROR("[Demo]  : Failed to set balance param ");
  }
}





void changerM::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                   std::map<std::string, double> sensors)
{
 return;
}

void changerM::stop()
{
  return;
}

bool changerM::isRunning()
{
  return false;
}

 void changerM::callback(changer::MyParamConfig &config , uint32_t level){
present.cob_x_offset_m                     = config.cob_x_offset_m;
present.cob_y_offset_m                      = config.cob_y_offset_m     ;
present.hip_roll_swap_angle_rad             = config.hip_roll_swap_angle_rad;
present.foot_roll_gyro_p_gain               = config.foot_roll_gyro_p_gain;
present.foot_roll_gyro_d_gain               = config.foot_roll_gyro_d_gain ;
present.foot_pitch_gyro_p_gain              = config.foot_pitch_gyro_p_gain    ;
present.foot_pitch_gyro_d_gain              = config.foot_pitch_gyro_d_gain    ;          
present.foot_roll_angle_p_gain              = config.foot_roll_angle_p_gain   ;          
present.foot_roll_angle_d_gain              = config.foot_roll_angle_d_gain  ;   
present.foot_pitch_angle_p_gain             = config.foot_pitch_angle_p_gain;
present.foot_pitch_angle_d_gain             = config.foot_pitch_angle_d_gain;
present.foot_x_force_p_gain                 = config.foot_x_force_p_gain   ;         
present.foot_x_force_d_gain                 = config.foot_x_force_d_gain  ;               
present.foot_y_force_p_gain                 = config.foot_y_force_p_gain ;
present.foot_y_force_d_gain                 = config.foot_y_force_d_gain          ;       
present.foot_z_force_p_gain                 = config.foot_z_force_p_gain         ;        
present.foot_z_force_d_gain                 = config.foot_z_force_d_gain        ;         
present.foot_roll_torque_p_gain             = config.foot_roll_torque_p_gain   ;          
present.foot_roll_torque_d_gain             = config.foot_roll_torque_d_gain  ;          
present.foot_pitch_torque_p_gain            = config.foot_pitch_torque_p_gain;
present.foot_pitch_torque_d_gain            = config.foot_pitch_torque_d_gain    ;       
present.roll_gyro_cut_off_frequency         = config.roll_gyro_cut_off_frequency;
present.pitch_gyro_cut_off_frequency        = config.pitch_gyro_cut_off_frequency;
present.roll_angle_cut_off_frequency        = config.roll_angle_cut_off_frequency   ;     
present.pitch_angle_cut_off_frequency       = config.pitch_angle_cut_off_frequency ;
present.foot_x_force_cut_off_frequency      = config.foot_x_force_cut_off_frequency  ;   
present.foot_y_force_cut_off_frequency      = config.foot_y_force_cut_off_frequency ;    
present.foot_z_force_cut_off_frequency      = config.foot_z_force_cut_off_frequency;
present.foot_roll_torque_cut_off_frequency  = config.foot_roll_torque_cut_off_frequency;
present.foot_pitch_torque_cut_off_frequency = config.foot_pitch_torque_cut_off_frequency;
setBalanceParamService( ); 
  }




