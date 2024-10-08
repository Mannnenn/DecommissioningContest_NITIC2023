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

#define CONTROLLER_CROSS_BUTTON 0
#define CONTROLLER_TRIANGLE_BUTTON 2

// Mechanism parameters
const float VELOCITY_UNIT_SCALE = 0.229;  // 0.229 [RPM] / 1 [LSB]
const float POSITION_UNIT_SCALE = 0.0879; // 0.088 [deg] / 1 [LSB]

const float OUTER_GEAR_TEETH_NUM = 20;
const float PLANET_GEAR_TEETH_NUM = 20;
const float DOUBLE_GEAR_TEETH_NUM = 70;

const float RADIUS_OF_CABLE_REEL = 12; // [mm]
const float RADIUS_OF_ROPE = 0.5;      // [mm]

const float WIDTH_OF_PEN = 24; // [mm]

const float MAX_RADIUS_OF_POLER = 190; // [mm]
const float MIN_RADIUS_OF_POLER = 0;   // [mm]

// Restriction of velocity,In range of motion or out of range of motion
const float RATIO_OF_VELOCITY_WITHIN = - (PLANET_GEAR_TEETH_NUM / OUTER_GEAR_TEETH_NUM) * WIDTH_OF_PEN / ((RADIUS_OF_CABLE_REEL + RADIUS_OF_ROPE) * M_PI) + DOUBLE_GEAR_TEETH_NUM / OUTER_GEAR_TEETH_NUM; // In range of motion, 3.2
const float RATIO_OF_VELOCITY_WITHOUT = (DOUBLE_GEAR_TEETH_NUM / OUTER_GEAR_TEETH_NUM);                                                                                                                 // Out of range of motion, 3.5

const float epsilon = 0.5; // for float comparison

const int32_t VELOCITY_OF_POLER_OF_CENTER = - 20; // [LSB]

// Global variables, which are updated by topic subscribers
float velocity_poler_center = 0;
float velocity_poler_outer = 0;
float position_poler_center = 0;
float position_poler_outer = 0;

// Global variables, which are updated by calculation
float angle_poler = 0;
float radius_poler = 0;

// This is a value shows that the poler is commanded to move
bool cmd_poler_move = false;
bool inverse_cmd_poler_move = false;

// This is a value shows that the velocity and position of poler is safe or not
bool is_velocity_of_poler_ok = false;
bool is_range_of_motion_of_poler_ok = false;

ros::Publisher velocity_pub;

bool isRangeOfMotionOfPolerOk()
{
    is_range_of_motion_of_poler_ok = (MIN_RADIUS_OF_POLER <= radius_poler && radius_poler <= MAX_RADIUS_OF_POLER);
    return (is_range_of_motion_of_poler_ok);
}

bool isVelocityOfPolerOk()
{
    // If the poler is in range of motion, the velocity of poler is all right
    if (is_range_of_motion_of_poler_ok)
    {
        is_velocity_of_poler_ok = true;
    }
    // If the poler is out of range of motion, the velocity of poler is need to be checked
    else
    {
        // If the poler is out of range of motion, the velocity of poler is zero, unnecessary to check
        if (velocity_poler_center == 0 && velocity_poler_outer == 0)
        {
            is_velocity_of_poler_ok = true;
        }
        // If the poler is out of range of motion, the velocity of poler is not zero, we need to check that the velocity of poler is equal to the ratio of velocity of poler without range of motion
        else
        {
            ROS_INFO("Ratio of Velocity is %f", velocity_poler_outer / velocity_poler_center);
            is_velocity_of_poler_ok = (fabs(velocity_poler_outer / velocity_poler_center - RATIO_OF_VELOCITY_WITHOUT) < epsilon);
        }
    }
    return (is_velocity_of_poler_ok);
}

void publishVelocity(int32_t velocity, uint8_t id)
{
    dynamixel_sdk_examples::SetPosition msg;
    msg.id = id;
    msg.position = velocity; // [LSB]
    velocity_pub.publish(msg);
}

// Publish velocity to poler center and poler outer,rotate 360 degree,increasing radius as pen width
void publishVelocityInSafeRange()
{
    publishVelocity(VELOCITY_OF_POLER_OF_CENTER, POLER_CENTER_ID);                                             // [LSB]
    publishVelocity(static_cast<int>(VELOCITY_OF_POLER_OF_CENTER * RATIO_OF_VELOCITY_WITHIN), POLER_OUTER_ID); // [LSB]
}

void publishVelocityInSafeRangeInverse()
{
    publishVelocity(-VELOCITY_OF_POLER_OF_CENTER, POLER_CENTER_ID);                                             // [LSB]
    publishVelocity(static_cast<int>(-VELOCITY_OF_POLER_OF_CENTER * RATIO_OF_VELOCITY_WITHIN), POLER_OUTER_ID); // [LSB]
}

// Stop poler center and poler outer when poler is out of range of motion or unsafe velocity
void publishVelocityInUnsafeRange()
{
    publishVelocity(0, POLER_CENTER_ID); // [LSB]
    publishVelocity(0, POLER_OUTER_ID);  // [LSB]
}

void calculatePositionOfPoler(float position_poler_center, float position_poler_outer, float &angle_poler, float &radius_poler)
{
    float amount_of_movement_of_poler_center = 0;
    float amount_of_movement_of_poler_outer = 0;

    amount_of_movement_of_poler_center = -(DOUBLE_GEAR_TEETH_NUM / PLANET_GEAR_TEETH_NUM) * (RADIUS_OF_CABLE_REEL + RADIUS_OF_ROPE) * position_poler_center;
    amount_of_movement_of_poler_outer = (OUTER_GEAR_TEETH_NUM / PLANET_GEAR_TEETH_NUM) * (RADIUS_OF_CABLE_REEL + RADIUS_OF_ROPE) * position_poler_outer;

    angle_poler = angles::normalize_angle_positive(position_poler_center);
    radius_poler = amount_of_movement_of_poler_outer + amount_of_movement_of_poler_center;
}

void servoStateCallback(const cable_reel::GetServo::ConstPtr &msg)
{
    // Convert from [LSB] to [rad/s]
    velocity_poler_center = ((float)msg->velocity_poler_center) * VELOCITY_UNIT_SCALE * (M_PI / 180) / 60; // Convert from [LSB] to [rad/s]
    velocity_poler_outer = ((float)msg->velocity_poler_outer) * VELOCITY_UNIT_SCALE * (M_PI / 180) / 60;   // Convert from [LSB] to [rad/s]

    // Convert from [LSB] to [rad]
    position_poler_center = ((float)msg->position_poler_center) * POSITION_UNIT_SCALE * (M_PI / 180); // Convert from [LSB] to [rad]
    position_poler_outer = ((float)msg->position_poler_outer) * POSITION_UNIT_SCALE * (M_PI / 180);   // Convert from [LSB] to [rad]


    // Calculate the angle and radius of poler
    calculatePositionOfPoler(position_poler_center, position_poler_outer, angle_poler, radius_poler);

    // Check the velocity and position of poler
    if (isRangeOfMotionOfPolerOk())
    {
        ROS_INFO("Poler is in range of motion.");

        if (isVelocityOfPolerOk())
        {
            if (cmd_poler_move)
            {
                if (!inverse_cmd_poler_move)
                {
                    ROS_INFO("Poler is safe to move.At angle %f and radius %f", angle_poler, radius_poler);
                    publishVelocityInSafeRange();
                }
                else
                {
                    ROS_INFO("Poler is safe to move.At angle %f and radius %f", angle_poler, radius_poler);
                    publishVelocityInSafeRangeInverse();
                }

            }
            else
            {
                ROS_INFO("Poler is safe to stop.At angle %f and radius %f", angle_poler, radius_poler);
                publishVelocityInUnsafeRange();
            }
        }
        else
        {
            ROS_ERROR("Poler is not safe to velocity.");
            publishVelocityInUnsafeRange();
        }
    }
    else
    {
        ROS_WARN("Poler is not in range of motion.");
        ROS_INFO("At angle %f and radius %f", angle_poler, radius_poler);
        publishVelocityInUnsafeRange();
    }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    cmd_poler_move = joy->buttons[CONTROLLER_CROSS_BUTTON];
    inverse_cmd_poler_move = joy->buttons[CONTROLLER_TRIANGLE_BUTTON];
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "poler_control");
    ros::NodeHandle nh;

    // Initialize ROS publisher, subscriber
    velocity_pub = nh.advertise<dynamixel_sdk_examples::SetPosition>("set_velocity", 10);

    // Initialize ROS publisher, subscriber
    ros::Subscriber velocity_poler_center_sub = nh.subscribe("servo_state", 5, servoStateCallback);

    ros::Subscriber joy = nh.subscribe("joy", 10, joyCallback);

    printf("Succeeded to initialize ROS node!\n");

    ros::spin();

    return 0;
}
