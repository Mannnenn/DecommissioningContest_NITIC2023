// Copyright 2020 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Add code by Sota Ogasawara,2024


#include <ros/ros.h>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "dynamixel_sdk_examples/GetPosition.h"
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "cable_reel/GetServo.h"


using namespace dynamixel;

// Control table address

// EEPROM AREA
#define ADDR_OPERATING_MODE 11
#define ADDR_HOMING_OFFSET 20
#define ADDR_PWM_LIMIT 36
#define ADDR_CURRENT_LIMIT 38
#define ADDR_VELOCITY_LIMIT 44

// RAM AREA
#define ADDR_TORQUE_ENABLE 64
#define ADDR_LED 65
#define ADDR_GOAL_CURRENT 102
#define ADDR_GOAL_VELOCITY 104

// Read only for DYNAMIXEL X series
#define ADDR_PRESENT_CURRENT 126
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0 // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define CABLE_REEL_ID 1            // cable reel for current control, XM430-W350-R
#define POLER_CENTER_ID 2          // poler center for velocity control, XL430-W250-T
#define POLER_OUTER_ID 3           // poler outer for velocity control, XL430-W250-T
#define BAUDRATE 1000000           // Baum rate of DYNAMIXEL
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

// Const variables for initialization

#define OPERATING_MODE_CURRENT_CONTROL 0
#define OPERATING_MODE_VELOCITY_CONTROL 1
#define HOMING_OFFSET_POLER_CENTER -1536 //-135 [deg] / 0.088 [deg/LSB]
#define HOMING_OFFSET_POLER_OUTER 0
#define PWM_LIMIT_CABLE_REEL 400
#define PWM_LIMIT_POLER_CENTER 600
#define PWM_LIMIT_POLER_OUTER 442
#define CURRENT_LIMIT_CABLE_REEL 442
#define VELOCITY_LIMIT_CABLE_REEL 885
#define VELOCITY_LIMIT_POLER_CENTER 88
#define VELOCITY_LIMIT_POLER_OUTER 265

#define ON 1
#define OFF 0

// Initialize PortHandler instance

PortHandler *portHandler;
PacketHandler *packetHandler;

// Initialize PortHandler instance
int initializePortHandler(PortHandler **portHandler, PacketHandler **packetHandler)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  *portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  *packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!(*portHandler)->openPort())
  {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!(*portHandler)->setBaudRate(BAUDRATE))
  {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  return 0;
}

// Set initial settings for EEPROM area

void setEEPROM(PortHandler *portHandler, uint8_t id, PacketHandler *packetHandler, uint8_t operating_mode, uint32_t homing_offset, uint16_t pwm_limit, uint16_t current_limit, uint32_t velocity_limit)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Set operating mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, id, ADDR_OPERATING_MODE, operating_mode, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("Failed to change operating mode! Result: %d", dxl_comm_result);
  }

  // Set homing offset
  dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, id, ADDR_HOMING_OFFSET, homing_offset, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("Failed to set homing offset! Result: %d", dxl_comm_result);
  }

  // Set PWM limit
  dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler, id, ADDR_PWM_LIMIT, pwm_limit, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("Failed to set PWM limit! Result: %d", dxl_comm_result);
  }

  // Set current limit for cable reel,XM430-W350-R
  if (id == CABLE_REEL_ID)
  {
    // Set current limit
    dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler, id, ADDR_CURRENT_LIMIT, current_limit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      ROS_ERROR("Failed to set current limit! Result: %d", dxl_comm_result);
    }
  }

  // Set velocity limit
  dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, id, ADDR_VELOCITY_LIMIT, velocity_limit, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("Failed to set velocity limit! Result: %d", dxl_comm_result);
  }
}

// Enable Dynamixel Torque
int enableTorque(PortHandler *portHandler, uint8_t id, PacketHandler *packetHandler)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, ON, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", id);
    return -1;
  }
  return 0;
}

// Disable Dynamixel Torque
int disableTorque(PortHandler *portHandler, uint8_t id, PacketHandler *packetHandler)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, OFF, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", id);
    return -1;
  }
  return 0;
}



void setCurrentCallback(const dynamixel_sdk_examples::SetPosition::ConstPtr &msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Current Value of X series is 2 byte data.
  uint32_t position = (unsigned int)msg->position; // Convert int16 -> uint16

  // Write Goal Position (length : 4 bytes)
  dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler, (uint8_t)msg->id, ADDR_GOAL_CURRENT, position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("setCurrent : [ID:%d] [CURRENT:%d]", msg->id, msg->position);
  }
  else
  {
    ROS_ERROR("Failed to set current! Result: %d", dxl_comm_result);
  }
}



void setVelocityCallback(const dynamixel_sdk_examples::SetPosition::ConstPtr &msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Velocity Value of X series is 4 byte data.
  uint32_t position = (unsigned int)msg->position; // Convert int32 -> uint32

  // Write Goal Position (length : 4 bytes)
  dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler, (uint8_t)msg->id, ADDR_GOAL_VELOCITY, position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("setVelocity : [ID:%d] [VELOCITY:%d]", msg->id, msg->position);
  }
  else
  {
    ROS_ERROR("Failed to set velocity! Result: %d", dxl_comm_result);
  }
}

int getCurrent(PortHandler *portHandler, uint8_t id, PacketHandler *packetHandler)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Current Value of X series is 4 byte data.
  int16_t current = 0;

  // Read Present Current (length : 4 bytes) and Convert uint16 -> int16
  dxl_comm_result = packetHandler->read2ByteTxRx(
      portHandler, (uint8_t)id, ADDR_PRESENT_CURRENT, (uint16_t *)&current, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getCurrent : [ID:%d] -> [CURRENT:%d]", id, current);
    return current;
  }
  else
  {
    ROS_INFO("Failed to get current! Result: %d", dxl_comm_result);
    return -1;
  }
}

int getVelocity(PortHandler *portHandler, uint8_t id, PacketHandler *packetHandler)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Velocity Value of X series is 4 byte data.
  int32_t velocity = 0;

  // Read Present Velocity (length : 4 bytes) and Convert uint32 -> int32
  dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, (uint8_t)id, ADDR_PRESENT_VELOCITY, (uint32_t *)&velocity, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getVelocity : [ID:%d] -> [VELOCITY:%d]", id, velocity);
    return velocity;
  }
  else
  {
    ROS_INFO("Failed to get velocity! Result: %d", dxl_comm_result);
    return -1;
  }
}

int getPresentPosition(PortHandler *portHandler, uint8_t id, PacketHandler *packetHandler)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data.
  int32_t position = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, (uint8_t)id, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", id, position);
    return position;
  }
  else
  {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    return -1;
  }
}



int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "read_write_node");
  ros::NodeHandle nh;

  // Initialize ROS publisher, subscriber
  ros::Publisher servo_state_pub = nh.advertise<cable_reel::GetServo>("servo_state", 10);

  ros::Subscriber set_current_sub = nh.subscribe("/set_current", 10, setCurrentCallback);
  ros::Subscriber set_velocity_sub = nh.subscribe("/set_velocity", 10, setVelocityCallback);

  printf("Succeeded to initialize ROS node!\n");

  bool is_torque_enable = true;
  if (nh.getParam("torque_enable", is_torque_enable))
  {
    ROS_INFO("Got param: %s", is_torque_enable ? "true" : "false");
  }
  else
  {
    ROS_ERROR("Failed to get param 'torque_enable'");
  }




  if (initializePortHandler(&portHandler, &packetHandler) == -1)
  {
    return -1;
  }

  printf("Succeeded to open the port!\n");

  // Set initial settings for EEPROM area
  setEEPROM(portHandler, CABLE_REEL_ID, packetHandler, OPERATING_MODE_CURRENT_CONTROL, 0, PWM_LIMIT_CABLE_REEL, CURRENT_LIMIT_CABLE_REEL, VELOCITY_LIMIT_CABLE_REEL);
  setEEPROM(portHandler, POLER_CENTER_ID, packetHandler, OPERATING_MODE_VELOCITY_CONTROL, HOMING_OFFSET_POLER_CENTER, PWM_LIMIT_POLER_CENTER, CURRENT_LIMIT_CABLE_REEL, VELOCITY_LIMIT_POLER_CENTER);
  setEEPROM(portHandler, POLER_OUTER_ID, packetHandler, OPERATING_MODE_VELOCITY_CONTROL, HOMING_OFFSET_POLER_OUTER, PWM_LIMIT_POLER_OUTER, CURRENT_LIMIT_CABLE_REEL, VELOCITY_LIMIT_POLER_OUTER);
  printf("Succeeded to set initial settings for EEPROM area!\n");


  if (is_torque_enable)
  {
    // Enable Dynamixel Torque
    if (enableTorque(portHandler, CABLE_REEL_ID, packetHandler) == -1)
      return -1;
    if (enableTorque(portHandler, POLER_CENTER_ID, packetHandler) == -1)
      return -1;
    if (enableTorque(portHandler, POLER_OUTER_ID, packetHandler) == -1)
      return -1;

    printf("Succeeded to enable torque!\n");
  }
  else
  {
    // Disable Dynamixel Torque
    if (disableTorque(portHandler, CABLE_REEL_ID, packetHandler) == -1)
      return -1;
    if (disableTorque(portHandler, POLER_CENTER_ID, packetHandler) == -1)
      return -1;
    if (disableTorque(portHandler, POLER_OUTER_ID, packetHandler) == -1)
      return -1;

    printf("Succeeded to disable torque!\n");
  }


  // Set the loop period, 10[Hz]
  ros::Rate loop_rate(10);

  // Get current, velocity, position for each Dynamixel ID
  while (ros::ok())
  {
    cable_reel::GetServo srv;

    srv.current_cable_reel = (int16_t)getCurrent(portHandler, CABLE_REEL_ID, packetHandler);
    srv.velocity_cable_reel = (int32_t)getVelocity(portHandler, CABLE_REEL_ID, packetHandler);
    srv.velocity_poler_center = (int32_t)getVelocity(portHandler, POLER_CENTER_ID, packetHandler);
    srv.velocity_poler_outer = (int32_t)getVelocity(portHandler, POLER_OUTER_ID, packetHandler);
    srv.position_cable_reel = (int32_t)getPresentPosition(portHandler, CABLE_REEL_ID, packetHandler);
    srv.position_poler_center = (int32_t)getPresentPosition(portHandler, POLER_CENTER_ID, packetHandler);
    srv.position_poler_outer = (int32_t)getPresentPosition(portHandler, POLER_OUTER_ID, packetHandler);

    servo_state_pub.publish(srv);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
