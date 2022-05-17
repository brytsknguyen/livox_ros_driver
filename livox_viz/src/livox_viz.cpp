//
// Created by Thien-Minh Nguyen on 15/12/20.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "livox_ros_driver/CustomMsg.h"
#include "livox_ros_driver/CustomPoint.h"

struct PointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t  ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint32_t, t, t)
                                  (uint16_t, reflectivity, reflectivity)
                                  (uint8_t,  ring, ring)
                                  (uint16_t, noise, noise)
                                  (uint32_t, range, range))

typedef pcl::PointCloud<PointXYZIRT> PointCloudXYZIRT;

using namespace pcl;

ros::Subscriber livox_sub;
ros::Publisher  livox_pub;

void livox_callback(const livox_ros_driver::CustomMsgConstPtr &msg)
{
    int Npts = msg->points.size();

    // Create and allocate memory of the pointcloud
    PointCloudXYZIRT pc;
    pc.resize(Npts);
    
    #pragma omp parallel for num_threads(NUM_CORE)
    for(int i = 0; i < Npts; i++)
    {
        auto &src = msg->points[i];
        auto &dst = pc.points[i];

        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.reflectivity;
        dst.ring = src.line;
        dst.t = src.offset_time;
    }

    // Convert to the ros msg
    sensor_msgs::PointCloud2 pc_ros;
    pcl::toROSMsg(pc, pc_ros);
    pc_ros.header = msg->header;

    // Publish the ros msg
    livox_pub.publish(pc_ros);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_viz");

    // Notify the start
    ROS_INFO("---> KEYFRAME MATCHING START.");

    // Create the node handle
    ros::NodeHandle nh("~");

    // Subscribe to the livox top
    livox_sub = nh.subscribe("/livox/lidar", 100, livox_callback);
    
    // Advertise to the livox top
    livox_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_viz", 100);
    
    // Spin and publish
    ros::spin();
}