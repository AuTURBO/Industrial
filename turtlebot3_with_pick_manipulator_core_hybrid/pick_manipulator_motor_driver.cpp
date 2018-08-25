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


Servo joint[MG_JOINT_NUM];
Servo gripper;
double write_joint_position[MG_JOINT_NUM] = {0.0,};
double write_gripper_position = {0};

PickManipulatorMotorDriver::PickManipulatorMotorDriver()
{
  dxl_id_[0] = JOINT1;
}

PickManipulatorMotorDriver::~PickManipulatorMotorDriver()
{
  closeDynamixel();
}

bool PickManipulatorMotorDriver::init(void)
{

  //double init_joint_position[MG_JOINT_NUM] = {0.0, -1.5707, 1.37, 0.2258};
  double init_joint_position[MG_JOINT_NUM] = {0,};
  double init_gripper_position = {10}; //30
  //int pin_num[MG_JOINT_NUM] = {3,5,6,9} ;
  int pin_num[XL_JOINT_NUM + MG_JOINT_NUM] = {0,3,5,6} ;
  
  DEBUG_SERIAL.begin(57600);
  bool joint_controller_state = false;
  bool gripper_controller_state = false;

  joint_controller_state   = joint_controller_.begin(DEVICENAME, BAUDRATE);

  if (joint_controller_state == false)
    DEBUG_SERIAL.println("Failed to open port(joint controller)");

  uint16_t get_model_number;
  for (int num = 0; num < XL_JOINT_NUM; num++)
  {
    joint_controller_state = joint_controller_.ping(dxl_id_[num], &get_model_number);
    if (joint_controller_state == false){
      DEBUG_SERIAL.println("Failed to ping(joint controller)");
      return false ;
    }
    else
      joint_controller_.jointMode(dxl_id_[num]);
  }

  protocol_version_ = joint_controller_.getProtocolVersion(); 

  joint_controller_.addSyncWrite("Goal_Position");

  if (protocol_version_ == 2.0)
  {
    joint_controller_.addSyncRead("Present_Position");
    joint_controller_.addSyncRead("Present_Velocity");
  }

  writeJointPosition(init_joint_position);

  //////////////////////////////////////////////////////////////////////////////
  
  gripper.begin();
  gripper.attach(11);
  gripper.offset(1, 20);  

  for (int index = XL_JOINT_NUM; index < MG_JOINT_NUM; index++)
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
  // Disable Dynamixel Torque
  setJointTorque(false);
}

bool PickManipulatorMotorDriver::setJointTorque(bool onoff)
{ 
  for (int num = 0; num < XL_JOINT_NUM; num++)  
    joint_controller_.itemWrite(dxl_id_[num], "Torque_Enable", onoff);

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

  int32_t* get_joint_present_position = NULL;

  if (protocol_version_ == 2.0)
    get_joint_present_position = joint_controller_.syncRead("Present_Position");
  else if (protocol_version_ == 1.0)
  {
    for (int index = 0; index < XL_JOINT_NUM; index++)
      get_joint_present_position[index] = joint_controller_.itemRead(dxl_id_[index], "Present_Position");
  }

  //int32_t get_gripper_present_position = gripper_controller_.itemRead(GRIPPER, "Present_Position");
  int32_t present_position[XL_JOINT_NUM] = {0, };

  for (int index = 0; index < XL_JOINT_NUM; index++)
    present_position[index] = get_joint_present_position[index];

  //present_position[DXL_NUM-1] = get_gripper_present_position;

  for (int index = 0; index < XL_JOINT_NUM; index++)
  {
    value[index] = joint_controller_.convertValue2Radian(dxl_id_[index], present_position[index]);
  }

  //value[DXL_NUM-1] = gripper_controller_.convertValue2Radian(GRIPPER, present_position[DXL_NUM-1]);

  
  for (int index = XL_JOINT_NUM; index < MG_JOINT_NUM + GRIPPER ; index++)
  {
    //value[index] = joint_controller_.convertValue2Radian(dxl_id_[index], present_position[index]);
    value[index]  = write_joint_position[index] ; 
  }
}

bool PickManipulatorMotorDriver::readVelocity(double *value)
{

  int32_t* get_joint_present_velocity = NULL;

  if (protocol_version_ == 2.0)
    get_joint_present_velocity = joint_controller_.syncRead("Present_Velocity");
  else if (protocol_version_ == 1.0)
  {
    for (int index = 0; index < XL_JOINT_NUM; index++)
      get_joint_present_velocity[index] = joint_controller_.itemRead(dxl_id_[index], "Present_Velocity");
  }

  //int32_t get_gripper_present_velocity = gripper_controller_.itemRead(GRIPPER, "Present_Velocity");
  int32_t present_velocity[XL_JOINT_NUM] = {0, };

  for (int index = 0; index < XL_JOINT_NUM; index++)
    present_velocity[index] = get_joint_present_velocity[index];

  //present_velocity[DXL_NUM-1] = get_gripper_present_velocity;

  for (int index = 0; index < XL_JOINT_NUM; index++)
  {
    value[index] = joint_controller_.convertValue2Velocity(dxl_id_[index], present_velocity[index]);
  }

  //value[DXL_NUM-1] = gripper_controller_.convertValue2Velocity(GRIPPER, present_velocity[DXL_NUM-1]);


  for (int index = XL_JOINT_NUM; index < MG_JOINT_NUM + GRIPPER ; index++)
  {
    //value[index] = joint_controller_.convertValue2Velocity(dxl_id_[index], present_velocity[index]);
    value[index] = 10 ;
  }
}

bool PickManipulatorMotorDriver::writeJointPosition(double *value)
{
  int32_t goal_position[XL_JOINT_NUM + MG_JOINT_NUM] = {0, };
  float pi = 3.14 ;
  DEBUG_SERIAL.print("xl..");
  for (int index = 0; index < XL_JOINT_NUM; index++)
  {
    /*DEBUG_SERIAL.print( dxl_id_[index]);
    DEBUG_SERIAL.print( "_");
    DEBUG_SERIAL.print( value[index]);
    DEBUG_SERIAL.println( ",");*/
    goal_position[index] = joint_controller_.convertRadian2Value(dxl_id_[index], value[index]);
  }

    DEBUG_SERIAL.print( value[0] );
    DEBUG_SERIAL.print( "->" );
    DEBUG_SERIAL.println( goal_position[0] );
  joint_controller_.syncWrite("Goal_Position", goal_position);

  for (int index = XL_JOINT_NUM; index < XL_JOINT_NUM + MG_JOINT_NUM; index++)
  {
    /*DEBUG_SERIAL.print( index );
    DEBUG_SERIAL.print( "_");
    DEBUG_SERIAL.print( value[index]);
    DEBUG_SERIAL.print( ",");*/
    write_joint_position[index] = value[index] ;
    value[index] = value[index]*180/pi ;
    joint[index].write((int)value[index]);	  
  }
  DEBUG_SERIAL.println( "");
  return true;
}
bool PickManipulatorMotorDriver::writeGripperPosition(double value)
{
  gripper.write(value);
  write_gripper_position = value ;
  return true;
}
