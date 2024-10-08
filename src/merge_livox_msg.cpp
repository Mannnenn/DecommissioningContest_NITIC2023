#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>

ros::Publisher pub;
livox_ros_driver::CustomMsg global_msg;

// Callback function
void livoxCB_1(const livox_ros_driver::CustomMsg::ConstPtr& msg)
{
    // Create a publisher object
    ros::NodeHandle nh;
    pub = nh.advertise<livox_ros_driver::CustomMsg>("merged_point", 10);

    global_msg = *msg;


}

void livoxCB_2(const livox_ros_driver::CustomMsg::ConstPtr& msg)
{
    // Create a new message
    livox_ros_driver::CustomMsg new_msg;

    // Copy the data from the incoming message to the new message
    new_msg = *msg;


    //TODO: Confirm that the this code is necessary
    // For each point in the incoming message
    for (size_t i = 0; i < msg->points.size(); ++i)
    {
        // Copy the x, y, z, reflectivity, tag, and line values
        new_msg.points[i].offset_time = msg->points[i].offset_time;
        new_msg.points[i].x = msg->points[i].x;
        new_msg.points[i].y = msg->points[i].y;
        new_msg.points[i].z = msg->points[i].z;
        new_msg.points[i].reflectivity = msg->points[i].reflectivity;
        new_msg.points[i].tag = msg->points[i].tag;
        new_msg.points[i].line = msg->points[i].line;
    }

    // Add the new message to the global message
    global_msg.points.insert(global_msg.points.end(), new_msg.points.begin(), new_msg.points.end());
    global_msg.point_num += new_msg.point_num;


    // Publish the global message
    pub.publish(global_msg);
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "message_relay");

    // Create a NodeHandle object
    ros::NodeHandle nh;

    // Create a subscriber object
    ros::Subscriber sub_1 = nh.subscribe("/livox/lidar_192_168_1_139", 10, livoxCB_1);
    ros::Subscriber sub_2 = nh.subscribe("/livox/lidar_192_168_1_147", 10, livoxCB_2);

    // Spin
    ros::spin();

    return 0;
}
