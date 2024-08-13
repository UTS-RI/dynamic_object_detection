#include "ros/ros.h"
#include "pcl_ros/transforms.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>

class ApplyTF
{
public:
    ApplyTF();
    ~ApplyTF();

private:
    // ros variables
    ros::NodeHandle nh_;
    ros::Subscriber sub_to_cloud_;
    ros::Subscriber sub_to_tf_;
    ros::Publisher pub_new_cloud_;

    Eigen::Matrix4f transform_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub_;
    message_filters::Subscriber<geometry_msgs::TransformStamped> tfSub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> MySyncPolicy;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    
    void processNewCloud(const boost::shared_ptr<const sensor_msgs::PointCloud2> &cloud,
                         const boost::shared_ptr<const geometry_msgs::TransformStamped> &tf);
};

ApplyTF::ApplyTF()
{
    pub_new_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("registered_pointcloud", 1000);

    // undistorded LIDAR
    cloudSub_.subscribe(nh_, "pointcloud", 1);
    tfSub_.subscribe(nh_, "transform", 1);

    sync_ = boost::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(100), cloudSub_, tfSub_);
    sync_->registerCallback(boost::bind(&ApplyTF::processNewCloud, this, _1, _2));
}

ApplyTF::~ApplyTF()
{
}

void ApplyTF::processNewCloud(const boost::shared_ptr<const sensor_msgs::PointCloud2> &cloud,
                              const boost::shared_ptr<const geometry_msgs::TransformStamped> &tf)
{
    // get the transform
    Eigen::Matrix4f transform;
    pcl_ros::transformAsMatrix(tf->transform, transform);

    // apply the transform
    sensor_msgs::PointCloud2 out;
    pcl_ros::transformPointCloud(transform.matrix(), *cloud, out);

    out.header.frame_id = "map";

    // publish the new cloud
    pub_new_cloud_.publish(out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apply_tf");
    ROS_INFO("Starting apply_tf node");

    ApplyTF apply_tf;

    ros::spin();

    return 0;
}

