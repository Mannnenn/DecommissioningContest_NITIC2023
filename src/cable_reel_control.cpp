#include <ros/ros.h>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include <sensor_msgs/Joy.h>
#include "cable_reel/GetServo.h"

#include "dynamixel_sdk_examples/GetPosition.h"
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <angles/angles.h>

#include "cmath"

// Default setting
#define CABLE_REEL_ID 1   // cable reel for current control, XM430-W350-R
#define POLER_CENTER_ID 2 // poler center for velocity control, XL430-W250-T
#define POLER_OUTER_ID 3  // poler outer for velocity control, XL430-W250-T

#define CONTROLLER_CIRCLE_BUTTON 1

const int16_t CURRENT_OF_CABLE_REEL = 5; // [LSB]

int16_t current_cable_real = 0;

bool cmd_cable_reel_move = false;

ros::Publisher current_pub;

void publishCurrent(int16_t current, uint8_t id)
{
    dynamixel_sdk_examples::SetPosition msg;
    msg.id = id;
    msg.position = current; // [LSB]
    current_pub.publish(msg);
}

void publishCurrent()
{
    publishCurrent(CURRENT_OF_CABLE_REEL, CABLE_REEL_ID);
}

void stopCableReel()
{
    publishCurrent(0, CABLE_REEL_ID);
}

void servoStateCallback(const cable_reel::GetServo::ConstPtr &msg)
{
    current_cable_real = msg->current_cable_reel; // [LSB]

    if (cmd_cable_reel_move)
    {
        publishCurrent();
    }
    else
    {
        stopCableReel();
    }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    // Toggle cable reel movement
    static bool last_button_state = false;
    bool current_button_state = joy->buttons[CONTROLLER_CIRCLE_BUTTON];

    if (current_button_state && !last_button_state)
    {
        cmd_cable_reel_move = !cmd_cable_reel_move;
    }

    last_button_state = current_button_state;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "cable_reel_control");
    ros::NodeHandle nh;

    // Initialize ROS publisher, subscriber
    current_pub = nh.advertise<dynamixel_sdk_examples::SetPosition>("set_current", 10);

    // Initialize ROS publisher, subscriber
    ros::Subscriber current_cable_reel_sub = nh.subscribe("servo_state", 5, servoStateCallback);

    ros::Subscriber joy = nh.subscribe("joy", 10, joyCallback);

    printf("Succeeded to initialize ROS node!\n");

    ros::spin();

    return 0;
}
