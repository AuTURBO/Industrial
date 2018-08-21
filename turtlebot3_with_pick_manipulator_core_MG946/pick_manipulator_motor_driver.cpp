/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors:  */

//#include "../../include/open_manipulator/open_manipulator_motor_driver.h"
#include "pick_manipulator_motor_driver.h"

#define JOINT_NUM 4
#define GRIPPER   1

Servo joint[JOINT_NUM];
Servo gripper;
double write_joint_position[JOINT_NUM] = {0.0,};
double write_gripper_position = {0};

PickManipulatorMotorDriver::PickManipulatorMotorDriver()
{

}

PickManipulatorMotorDriver::~PickManipulatorMotorDriver()
{
  closeDynamixel();
}

bool PickManipulatorMotorDriver::init(void)
{

  //double init_joint_position[JOINT_NUM] = {0.0, -1.5707, 1.37, 0.2258};
  double init_joint_position[JOINT_NUM] = {0,0,0,0};
  double init_gripper_position = {10}; //30
  int pin_num[JOINT_NUM] = {3,5,6,9} ;
  
  DEBUG_SERIAL.begin(57600);
  
  gripper.begin();
  gripper.attach(11);
  gripper.offset(1, 20);  

  for (int index = 0; index < JOINT_NUM; index++)
  {
    joint[index].begin() ;
	  joint[index].attach(pin_num[index]) ;
	  joint[index].offset(1, 20);
	  joint[index].write((int)init_joint_position[index]);
	  write_joint_position[index] = init_joint_position[index];
  }
  
  write_gripper_position = init_gripper_position;

  joint_torque_state_	= true;
  gripper_torque_state_ = true;
   
  //DEBUG_SERIAL.println("Success to init PickManipulator Motor Driver(joint and gripper controller)");
  //nh2.loginfo("/////////////////////////////////////////////////////////////////////////////////");
  return true;
}

void PickManipulatorMotorDriver::closeDynamixel(void)
{

}

bool PickManipulatorMotorDriver::setJointTorque(bool onoff)
{ 
  joint_torque_state_ = onoff;
}

bool PickManipulatorMotorDriver::getJointTorque()
{
  return joint_torque_state_;
}

bool PickManipulatorMotorDriver::setGripperTorque(bool onoff)
{ 
  gripper_torque_state_ = onoff;
}

bool PickManipulatorMotorDriver::getGripperTorque()
{
  return gripper_torque_state_;
}

bool PickManipulatorMotorDriver::readPosition(double *value)
{   
  for (int index = 0; index < JOINT_NUM; index++)
  {
    //value[index] = joint_controller_.convertValue2Radian(dxl_id_[index], present_position[index]);
	  value[index]  = write_joint_position[index] ; 
  }
}

bool PickManipulatorMotorDriver::readVelocity(double *value)
{
  for (int index = 0; index < JOINT_NUM; index++)
  {
    //value[index] = joint_controller_.convertValue2Velocity(dxl_id_[index], present_velocity[index]);
    value[index] = 10 ;
  }
}

bool PickManipulatorMotorDriver::writeJointPosition(double *value)
{
  int32_t goal_position[JOINT_NUM] = {0, };
  float pi = 3.14 ;

  for (int index = 0; index < JOINT_NUM; index++)
  {
    //goal_position[index] = joint_controller_.convertRadian2Value(dxl_id_[index], value[index]);
    value[index] = value[index]*180/pi ;
    joint[index].write((int)value[index]);
	  write_joint_position[index] = value[index] ;
  }
  return true;
}
bool PickManipulatorMotorDriver::writeGripperPosition(double value)
{
  gripper.write(value);
  write_gripper_position = value ;
  return true;
}
/*
int32_t PickManipulatorMotorDriver::convertRadian2Value(uint8_t id, float radian)
{
  int32_t value = 0;
  int8_t factor = getToolsFactor(id);

  if (radian > 0)
  {
    value = (radian * (tools_[factor].getValueOfMaxRadianPosition() - tools_[factor].getValueOfZeroRadianPosition()) / tools_[factor].getMaxRadian()) + tools_[factor].getValueOfZeroRadianPosition();
  }
  else if (radian < 0)
  {
    value = (radian * (tools_[factor].getValueOfMinRadianPosition() - tools_[factor].getValueOfZeroRadianPosition()) / tools_[factor].getMinRadian()) + tools_[factor].getValueOfZeroRadianPosition();
  }
  else
  {
    value = tools_[factor].getValueOfZeroRadianPosition();
  }

  return value;
}*/
