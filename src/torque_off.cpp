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
#include "dynamixel_sdk/dynamixel_sdk.h"


using namespace dynamixel;

// RAM AREA
#define ADDR_TORQUE_ENABLE 64

// Protocol version
#define PROTOCOL_VERSION 2.0 // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define CABLE_REEL_ID 1            // cable reel for current control, XM430-W350-R
#define POLER_CENTER_ID 2          // poler center for velocity control, XL430-W250-T
#define POLER_OUTER_ID 3           // poler outer for velocity control, XL430-W250-T
#define BAUDRATE 1000000           // Baum rate of DYNAMIXEL
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command


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


int main(int argc, char **argv)
{

    if (initializePortHandler(&portHandler, &packetHandler) == -1)
    {
        return -1;
    }

    printf("Succeeded to open the port!\n");


    // Enable Dynamixel Torque
    if (disableTorque(portHandler, CABLE_REEL_ID, packetHandler) == -1)
        return -1;
    if (disableTorque(portHandler, POLER_CENTER_ID, packetHandler) == -1)
        return -1;
    if (disableTorque(portHandler, POLER_OUTER_ID, packetHandler) == -1)
        return -1;

    printf("Succeeded to disenable torque!\n");

    // Initialize ROS node
    ros::init(argc, argv, "read_write_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    ros::spinOnce();

    portHandler->closePort();
    return 0;
}
