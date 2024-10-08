#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2, combinedCloud;
ros::Publisher pub;

void cloud_cb1 (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::fromROSMsg (*input, cloud1);
}

void cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::fromROSMsg (*input, cloud2);
}

void combineAndPublish()
{
    *combinedCloud = *cloud1;
    *combinedCloud += *cloud2;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(combinedCloud, output);
    pub.publish(output);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "add_pointclouds");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe ("input1", 1, cloud_cb1);
    ros::Subscriber sub2 = nh.subscribe ("input2", 1, cloud_cb2);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("combined", 1);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        combineAndPublish();
        loop_rate.sleep();
    }

    return 0;
}