#ifndef EIGEN_TO_ROS_H
#define EIGEN_TO_ROS_H

#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>
#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Function to convert a pointcloud to a ros message (WARNING: the timestamp is not set)
sensor_msgs::PointCloud2 cloudAndScoreToRosMsg(const Eigen::MatrixXf& pts, const Eigen::VectorXf& scores)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.width = pts.cols();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (int i = 0; i < pts.cols(); i++)
    {
        pcl::PointXYZI point;
        point.x = pts(0, i);
        point.y = pts(1, i);
        point.z = pts(2, i);
        point.intensity = scores(i);
        cloud.points[i] = point;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "map";
    return output;
}

// Function to convert a pointcloud to a ros message (WARNING: the timestamp is not set)
sensor_msgs::PointCloud2 cloudToRosMsg(const Eigen::MatrixXf& pts)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = pts.cols();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (int i = 0; i < pts.cols(); i++)
    {
        pcl::PointXYZ point;
        point.x = pts(0, i);
        point.y = pts(1, i);
        point.z = pts(2, i);
        cloud.points[i] = point;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "map";
    return output;
}

// Function to convert a pointcloud to a ros message
sensor_msgs::PointCloud2 cloudAndScoreToRosMsg(const Eigen::MatrixXf& pts, const Eigen::VectorXf& scores, const std_msgs::Header& header)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.width = pts.cols();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (int i = 0; i < pts.cols(); i++)
    {
        pcl::PointXYZI point;
        point.x = pts(0, i);
        point.y = pts(1, i);
        point.z = pts(2, i);
        point.intensity = scores(i);
        cloud.points[i] = point;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header = header;
    return output;
}
// Function to convert a pointcloud to a ros message
sensor_msgs::PointCloud2 cloudToRosMsg(const Eigen::MatrixXf& pts, const std_msgs::Header& header)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = pts.cols();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (int i = 0; i < pts.cols(); i++)
    {
        pcl::PointXYZ point;
        point.x = pts(0, i);
        point.y = pts(1, i);
        point.z = pts(2, i);
        cloud.points[i] = point;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header = header;
    output.header.frame_id = "map";
    return output;
}


#endif