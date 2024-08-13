#include "ros/ros.h"
#include "pcl_ros/transforms.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "normal_filter/nanoflann_wrapper.hpp"
#include "normal_filter/EigenToROS.hpp"



class Upsampling
{
public:
    Upsampling();
    ~Upsampling();

private:
    // ros variables
    ros::NodeHandle nh_;
    ros::Subscriber sub_to_cloud_;
    ros::Subscriber sub_to_dynamic_;
    ros::Publisher pub_upsampled_cloud_with_score_;
    ros::Publisher pub_static_;
    ros::Publisher pub_dynamic_;

    int neighborhood_size_ = 100;
    float neighbourhood_radius_ = 0.2;
    bool use_radius_search_ = true;
    float dynamic_threshold_ = 0.1;

    Eigen::Matrix4f transform_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> dynamicSub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    
    void processNewCloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud_input,
                         const boost::shared_ptr<const sensor_msgs::PointCloud2>& downsampled_cloud_input);
    Eigen::VectorXf update_dynamic_scores(const Eigen::MatrixXf& cloud, 
                                          const Eigen::MatrixXf& dynamic_cloud, 
                                          const Eigen::VectorXf& downsampled_cloud_dynamic_scores );
};

Upsampling::Upsampling()
{
    // get the parameters
    nh_.getParam("neighborhood_size_upsampling", neighborhood_size_);
    nh_.getParam("neighborhood_radius_upsampling", neighbourhood_radius_);
    nh_.getParam("use_radius_search", use_radius_search_);
    nh_.getParam("dynamic_threshold", dynamic_threshold_);

    // print the parameters to the screen
    ROS_INFO("neighborhood_size_upsampling: %d", neighborhood_size_);
    ROS_INFO("neighborhood_radius_upsampling: %f", neighbourhood_radius_);
    ROS_INFO("use_radius_search: %d", use_radius_search_);
    ROS_INFO("dynamic_threshold: %f", dynamic_threshold_);

    // set the subscribers
    cloudSub_.subscribe(nh_, "registered_pointcloud", 1);
    dynamicSub_.subscribe(nh_, "pointcloud_dynamic_score_downsampled", 1);

    sync_ = boost::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(100), cloudSub_, dynamicSub_);
    sync_->registerCallback(boost::bind(&Upsampling::processNewCloud, this, _1, _2));

    // set the publishers
    pub_upsampled_cloud_with_score_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_dynamic_score_upsampled", 1000);
    pub_static_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_static", 1000);
    pub_dynamic_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_dynamic", 1000);
}

Upsampling::~Upsampling()
{
}

void Upsampling::processNewCloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud_input,
                                 const boost::shared_ptr<const sensor_msgs::PointCloud2>& downsampled_cloud_input)
{
    // start timmer
    ros::Time begin = ros::Time::now();

    // convert the cloud to eigen matrix of size 4xN
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    Eigen::MatrixXf full_resolution_cloud = temp_cloud->getMatrixXfMap().topRows(3);

    // convert the dynamic cloud to eigen matrix of size 4xN
    pcl::PCLPointCloud2 pcl_pc2_dynamic;
    pcl_conversions::toPCL(*downsampled_cloud_input, pcl_pc2_dynamic);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2_dynamic, *temp_cloud_downsampled);
    Eigen::MatrixXf downsampled_cloud = temp_cloud_downsampled->getMatrixXfMap().topRows(3);
    Eigen::VectorXf downsampled_cloud_dynamic_scores = temp_cloud_downsampled->getMatrixXfMap().row(4);

    // upsample the cloud
    Eigen::VectorXf dynamic_scores = update_dynamic_scores(full_resolution_cloud, downsampled_cloud, downsampled_cloud_dynamic_scores);

    // stop timmer
    ros::Time end = ros::Time::now();
    ROS_INFO("upsampling took %f seconds", (end - begin).toSec());

    // set cloud to publish
    sensor_msgs::PointCloud2 cloud_to_publish = cloudAndScoreToRosMsg(full_resolution_cloud, dynamic_scores);

    // publish the clouds
    if (pub_upsampled_cloud_with_score_.getNumSubscribers() > 0)
    {
        pub_upsampled_cloud_with_score_.publish(cloud_to_publish);
    }
    if ((pub_static_.getNumSubscribers() > 0) || (pub_dynamic_.getNumSubscribers() > 0))
    {
        // compute the number of values from dynamic_scores that are above the threshold
        int number_of_dynamic_points = (dynamic_scores.array() > dynamic_threshold_).count();

        Eigen::MatrixXf cloud_to_process_static(full_resolution_cloud.rows(), full_resolution_cloud.cols() - number_of_dynamic_points);
        Eigen::MatrixXf cloud_to_process_dynamic(full_resolution_cloud.rows(), number_of_dynamic_points);
        int counter_static = 0;
        int counter_dynamic = 0;
        for (int i = 0; i < dynamic_scores.size(); i++)
        {
            if (dynamic_scores(i) > dynamic_threshold_)
            {
                cloud_to_process_dynamic.col(counter_dynamic) = full_resolution_cloud.col(i);
                counter_dynamic++;
            }
            else
            {
                cloud_to_process_static.col(counter_static) = full_resolution_cloud.col(i);
                counter_static++;
            }
        }
        // create the outputs in ROS format, set the headers, and publish
        sensor_msgs::PointCloud2 static_output = cloudToRosMsg(cloud_to_process_static, cloud_input->header);
        sensor_msgs::PointCloud2 dynamic_output = cloudToRosMsg(cloud_to_process_dynamic, cloud_input->header);
        // publish
        pub_static_.publish(static_output);
        pub_dynamic_.publish(dynamic_output);
    }
}

// create the upsampling function
Eigen::VectorXf Upsampling::update_dynamic_scores(const Eigen::MatrixXf& cloud, const Eigen::MatrixXf& dynamic_cloud, const Eigen::VectorXf& downsampled_cloud_dynamic_scores )
{

    // check if clouds have 3 rows
    if (cloud.rows() != 3)
        throw std::invalid_argument("Error in " + std::string(__func__) + ": cloud must have 3 rows, found " + std::to_string(cloud.rows()) + " rows and " + std::to_string(cloud.cols()) + " cols");
    if (dynamic_cloud.rows() != 3)
        throw std::invalid_argument("Error in " + std::string(__func__) + ": dynamic_cloud must have 3 rows, found " + std::to_string(dynamic_cloud.rows()) + " rows and " + std::to_string(dynamic_cloud.cols()) + " cols");


    // create the nanoflann wrapper
    nanoflann_wrapper<float> kd_tree(cloud);

    // for each point in the dynamic cloud
    Eigen::VectorXf scores = Eigen::VectorXf::Zero(cloud.cols());
    for (int i = 0; i < dynamic_cloud.cols(); i++)
    {
        if (downsampled_cloud_dynamic_scores(i) > dynamic_threshold_)
        {
            Eigen::Vector3f query_point = dynamic_cloud.col(i);

            // query the kdtree
            std::vector<int> indexes;
            if (use_radius_search_)
                indexes = kd_tree.radius_search(query_point, neighbourhood_radius_*neighbourhood_radius_);
            else
                indexes = kd_tree.KNN_search(query_point, neighborhood_size_);

           // update the scores
            for (auto index : indexes)
                scores(index) = 1.0;
        }
    }

    return scores;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "upsampling");
    ROS_INFO("Starting upsampling node");

    Upsampling upsampling;

    ros::spin();

    return 0;
}

