#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

// include pointcloud headers
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>

#include "normal_filter/nanoflann_wrapper.hpp"
#include "normal_filter/downsampling.hpp"
#include "normal_filter/EigenToROS.hpp"

using namespace std::chrono;


class stackedPointCloud
{
public:
    stackedPointCloud();
    ~stackedPointCloud();

private:
    Eigen::MatrixXf cloud_stack_;
    std::vector <Eigen::MatrixXf> clouds_waiting_to_be_processed_;
    std::vector <std::vector<int>> in_cloud_samples_to_be_processed_;
    std::vector <ros::Time> timestamps_waiting_to_be_processed_;
    std::vector <std_msgs::Header> headers_waiting_to_be_processed_;

    // actual params
    int neighborhood_size_ = 200;
    double neighbourhood_radius_ = 0.5;
    bool use_radius_search_ = true;
    int downsampling_bin_number_ = 600; // size of the voxel grid, currently scale divided by downsampling_bin_number_
    float voxel_size_ = 0.1;

    double dynamic_threshold_ = 0.5;
    int number_of_clouds_used_as_padding_ = 20; // number of scans used as padding before and after the current scan

    // internal variables
    bool visualization_ = true;
    int number_of_clouds_stored_ = 0;
    int max_number_of_clouds_stored_ = 1 + 2*number_of_clouds_used_as_padding_;
    std::vector <int> cloud_sizes_;
    int cloud_counter_ = 0;
    ros::Time stamp_zero_;
    ros::Time stamp_last_;
    
    // pointer to the kd-tree object
    nanoflann_wrapper<float> *kd_tree;

    // ros stuff
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    ros::Publisher pub_;
    ros::Publisher pub_stacked_pointcloud_;
    
    void processNewCloud(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input);
    void stackCloud(Eigen::MatrixXf &new_cloud, std::vector<int> &in_cloud_samples, ros::Time &stamp_current, const std_msgs::Header &header_current);
};

stackedPointCloud::stackedPointCloud()
{
    // get the rosparams
    nh_.getParam("neighborhood_size", neighborhood_size_);
    nh_.getParam("neighborhood_radius", neighbourhood_radius_);
    nh_.getParam("use_radius_search", use_radius_search_);
    nh_.getParam("downsampling_bin_number", downsampling_bin_number_);
    nh_.getParam("voxel_size", voxel_size_);
    nh_.getParam("dynamic_threshold", dynamic_threshold_);
    nh_.getParam("number_of_clouds_used_as_padding", number_of_clouds_used_as_padding_);
    nh_.getParam("visualization", visualization_);

    // set the subscribers
    sub = nh_.subscribe("registered_pointcloud", 100, &stackedPointCloud::processNewCloud, this);

    max_number_of_clouds_stored_ = 1 + 2*number_of_clouds_used_as_padding_;

    // set the publishers
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_dynamic_score_downsampled", 1000);
    pub_stacked_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("stacked_pointcloud", 1000);
}


stackedPointCloud::~stackedPointCloud()
{
    
}


void stackedPointCloud::stackCloud(Eigen::MatrixXf &new_cloud, 
                                   std::vector<int> &in_cloud_samples, 
                                   ros::Time &stamp_current, 
                                   const std_msgs::Header &header_current)
{
    // add new_cloud to the cloud
    if (number_of_clouds_stored_ == 0)
    {
        // initiate the cloud
        cloud_stack_ = new_cloud;

        // update the stacked clouds info in the class
        cloud_sizes_.push_back(new_cloud.cols());
        number_of_clouds_stored_++;
    }
    else if (number_of_clouds_stored_ < max_number_of_clouds_stored_)
    {
        // stack the cloud
        cloud_stack_.conservativeResize(Eigen::NoChange, cloud_stack_.cols() + new_cloud.cols());
        cloud_stack_.rightCols(new_cloud.cols()) = new_cloud;

        // update the stacked clouds info in the class
        cloud_sizes_.push_back(new_cloud.cols());
        number_of_clouds_stored_++;
    }
    else
    {
        // remove the first cloud (shift the right rows to the top)
        cloud_stack_.leftCols(cloud_stack_.cols() - cloud_sizes_.at(0)) = cloud_stack_.rightCols(cloud_stack_.cols() - cloud_sizes_.at(0));

        // add the last cloud at the right
        cloud_stack_.conservativeResize(Eigen::NoChange, cloud_stack_.cols() - cloud_sizes_.at(0) + new_cloud.cols());
        cloud_stack_.rightCols(new_cloud.cols()) = new_cloud;

        // update the stacked clouds info in the class
        cloud_sizes_.erase(cloud_sizes_.begin());
        cloud_sizes_.push_back(new_cloud.cols());
    }

    // stack new_cloud and stamp_current into clouds_waiting_to_be_processed_ and timestamps_waiting_to_be_processed_
    clouds_waiting_to_be_processed_.push_back(new_cloud);
    in_cloud_samples_to_be_processed_.push_back(in_cloud_samples);
    timestamps_waiting_to_be_processed_.push_back(stamp_current);
    headers_waiting_to_be_processed_.push_back(header_current);

    // remove the excess clouds
    if (clouds_waiting_to_be_processed_.size() > number_of_clouds_used_as_padding_+1)
    {
        clouds_waiting_to_be_processed_.erase(clouds_waiting_to_be_processed_.begin());
        in_cloud_samples_to_be_processed_.erase(in_cloud_samples_to_be_processed_.begin());
        timestamps_waiting_to_be_processed_.erase(timestamps_waiting_to_be_processed_.begin());
        headers_waiting_to_be_processed_.erase(headers_waiting_to_be_processed_.begin());
    }
    
}


void stackedPointCloud::processNewCloud(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
    // set the first timestamp to stamp_zero_
    if (cloud_counter_ == 0)
    {
        stamp_zero_ = input->header.stamp;
        stamp_last_ = stamp_zero_;
    }
    else // drop the first scan to have a correct linear interpolation for the points timestamps (from the previous timestamp to current timestamp)
    {
        ros::Time begin = ros::Time::now();

        // convert the cloud to eigen matrix of size 4xN
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
        Eigen::MatrixXf new_cloud = temp_cloud->getMatrixXfMap();

        // get the timestamp of the cloud
        ros::Time stamp_current = input->header.stamp;
        double timestamps_beginning = (stamp_last_-stamp_zero_).toSec();
        double timestamps_end = (stamp_current-stamp_zero_).toSec();

        // set the last row into a vector of the timestamp
        Eigen::VectorXf timestamps_linear = Eigen::VectorXf::LinSpaced(new_cloud.cols(), timestamps_beginning, timestamps_end);
        new_cloud.row(3) = timestamps_linear;

        // downsampling, in_cloud_samples contains the index of the points from the input cloud that end up in the output cloud
        auto [downsampled_cloud, in_cloud_samples] = voxel_grid_downsampling(new_cloud, voxel_size_);

        //resize new_cloud
        new_cloud.conservativeResize(Eigen::NoChange, downsampled_cloud.cols());
        new_cloud.topRows(3) = downsampled_cloud.topRows(3);

        // stack the new cloud into cloud_
        stackCloud(new_cloud, in_cloud_samples, stamp_current, input->header);

        ros::Time end_of_preprocessing = ros::Time::now();
        ROS_INFO("Time for stacking the data: %f", (end_of_preprocessing - begin).toSec());

        // check if there are enough clouds in the stack to perform the computation
        if (number_of_clouds_stored_ == max_number_of_clouds_stored_ &&
            clouds_waiting_to_be_processed_.size() == number_of_clouds_used_as_padding_+1)
        {
            Eigen::MatrixXf cloud_to_process = clouds_waiting_to_be_processed_.at(0);

            // kdtree with float data
            ros::Time begin2 = ros::Time::now();
            Eigen::MatrixXf cloud_float = cloud_stack_.topRows(3);
            kd_tree = new nanoflann_wrapper<float>(cloud_float);
            ros::Time end2 = ros::Time::now();
            ROS_INFO("Time for building the kd-tree: %f", (end2 - begin2).toSec());

            // for all points in the latest scan perform a knn search
            ros::Time begin3 = ros::Time::now();
            Eigen::VectorXf dynamic_scores = Eigen::VectorXf::Zero(cloud_to_process.cols());
            std::vector <int> indices_prediction;

            ROS_INFO("Querry for %ld points", cloud_to_process.cols());
            if (use_radius_search_)
                ROS_INFO("Using radius search with radius %f", neighbourhood_radius_);
            else
                ROS_INFO("Using knn search with k %d", neighborhood_size_);

            #pragma omp parallel for
            for (int i = 0; i < cloud_to_process.cols(); i++)
            {
                Eigen::Vector3f query_point;
                Eigen::MatrixXf neighborhood;

                // query the kdtree
                query_point = cloud_to_process.col(i).topRows(3);
                std::vector<int> indexes;
                if (use_radius_search_)
                    indexes = kd_tree->radius_search(query_point, neighbourhood_radius_);
                else
                    indexes = kd_tree->KNN_search(query_point, neighborhood_size_);

                // get the neighborhood
                neighborhood = Eigen::MatrixXf::Zero(4, indexes.size());
                for (int j = 0; j < indexes.size(); j++)
                {
                    neighborhood.col(j) = cloud_stack_.col(indexes.at(j));
                }

                if (indexes.size() > 10) { // otherwise the normal computation does not make sense
                    Eigen::MatrixXf neighborhood_temp = neighborhood.transpose();

                    // compute the covariance: see https://stackoverflow.com/a/15142446/2562693
                    Eigen::MatrixXf centered = neighborhood_temp.rowwise() - neighborhood_temp.colwise().mean();
                    Eigen::MatrixXf cov = (centered.adjoint() * centered) / float(neighborhood_temp.rows() - 1);

                    // compute the eigenvalues and eigenvectors
                    Eigen::EigenSolver<Eigen::MatrixXf> es(cov);
                    Eigen::VectorXf eigenvalues = es.eigenvalues().real();
                    Eigen::MatrixXf eigenvectors = es.eigenvectors().real();

                    // get the index of the smallest eigenvector
                    int min_index = 0; eigenvalues.minCoeff(&min_index);
                    Eigen::VectorXf smallest_eigenvector = eigenvectors.col(min_index);

                    // get the time component of the smallest eigenvector
                    float dynamic_score = abs(smallest_eigenvector(3));

                    // add the dynamic score to the vector
                    dynamic_scores(i) = dynamic_score;
                }
                
            }

            // iterate through the dynamic scores and find the indices of the points with the highest dynamic scores                            
            for (int i = 0; i < dynamic_scores.size(); i++)
                if (dynamic_scores(i) > dynamic_threshold_)
                    indices_prediction.push_back(i);

            ros::Time end3 = ros::Time::now();
            ROS_INFO("Time for queries and dynamic scores computation: %f", (end3 - begin3).toSec());

            // publish the dynamic scores
            if (pub_.getNumSubscribers() > 0)
            {
                sensor_msgs::PointCloud2 output = cloudAndScoreToRosMsg(cloud_to_process, dynamic_scores, headers_waiting_to_be_processed_.at(0));
                pub_.publish(output);
            }
            if (pub_stacked_pointcloud_.getNumSubscribers() > 0)
            {
                sensor_msgs::PointCloud2 output = cloudToRosMsg(cloud_stack_.topRows(3), headers_waiting_to_be_processed_.at(0));
                pub_stacked_pointcloud_.publish(output);
            }

        }

        stamp_last_ = stamp_current;
    }

    cloud_counter_++;

}



int main(int argc, char **argv)
{
    // initialize the node
    ros::init(argc, argv, "listener");
    ROS_INFO("Starting dynamic_detector node");

    // create the class that does all the work
    stackedPointCloud stackedPointCloud;
    
    // spin
    ros::spin();

    return 0;
}
