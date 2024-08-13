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

#include "phtree-cpp/include/phtree/phtree.h"
#include "phtree-cpp/include/phtree/filter.h"

#include "normal_filter/types.h"
#include "normal_filter/ros_utils.h"

using namespace std::chrono;


// Typedefs for the PhTree
typedef improbable::phtree::PhPointD<3> PointPh;
template <typename V>
using TreePh = improbable::phtree::PhTreeD<3, V>;

typedef std::tuple<int, int, int> CellIndex;


// Typedefs for the HashMap
template <typename V>
using HashMap = ankerl::unordered_dense::map<CellIndex, V>;


struct Cell
{
    Vec4 point_sum = Vec4::Zero();
    Mat4 covariance_sum = Mat4::Zero();
    int count = 0;

    template <typename T>
    Cell(const PointTemplated<T>& point)
    {
        Vec4 temp = Vec4((double)point.x, (double)point.y, (double)point.z, (double)point.t);
        point_sum = temp;
        covariance_sum = temp * temp.transpose();
        count = 1;
    }

    template <typename T>
    void addPoint(const PointTemplated<T>& point)
    {
        Vec4 temp = Vec4((double)point.x, (double)point.y, (double)point.z, (double)point.t);
        point_sum += temp;
        covariance_sum += temp * temp.transpose();
        count++;
    }

    template <typename T>
    bool removePoint(const PointTemplated<T>& point)
    {
        Vec4 temp = Vec4(point.x, point.y, point.z, point.t);
        point_sum -= temp;
        covariance_sum -= temp * temp.transpose();
        count--;

        return count == 0;
    }


    Vec3 getCentroid()
    {
        return point_sum.segment<3>(0) / (double)count;
    }

};

typedef std::shared_ptr<Cell> CellPtr;


// Functor to query the neighbors of a point in the PhTree radius search
struct PhNeighborQuery
{
    void operator()(const PointPh& point, CellPtr& cell)
    {
        neighbors.push_back({point, cell});
    }
    std::vector<std::pair<PointPh, CellPtr> > neighbors;
};






class stackedPointCloud
{
public:
    stackedPointCloud();
    ~stackedPointCloud();

private:
    // actual params
    double neighbourhood_radius_ = 0.5;
    double downsampled_voxel_size_ = 0.1;
    double inv_downsampled_voxel_size_ = 1.0/downsampled_voxel_size_;
    double voxel_size_ = 0.05;
    double inv_voxel_size_ = 1.0/voxel_size_;
    double half_voxel_size_ = voxel_size_/2.0;

    double dynamic_threshold_ = 0.5;
    int number_of_clouds_used_as_padding_ = 20; // number of scans used as padding before and after the current scan

    // internal variables
    int max_number_of_clouds_stored_ = 1 + 2*number_of_clouds_used_as_padding_;
    int cloud_counter_ = 0;
    ros::Time stamp_zero_;

    // time multiplier
    double time_multiplier_;
    
    //// pointer to the kd-tree object
    //nanoflann_wrapper<float> *kd_tree;


    // Phtree structure
    TreePh<CellPtr> phtree_;

    // HashMap structure
    HashMap<CellPtr> local_map_;

    // Downsampled point clouds
    std::vector<std::vector<Pointf>> downsampled_clouds_;
    std::vector<std_msgs::Header> headers_;
    
    // Buffer pointer the position of where the next cloud will be stored
    int buffer_pointer_ = 0;


    // ros stuff
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    ros::Publisher pub_;
    ros::Publisher pub_stacked_pointcloud_;
    ros::Publisher pub_local_map_debug_;
    
    void processNewCloud(const sensor_msgs::PointCloud2ConstPtr &input);
    void stackCloud(Eigen::MatrixXf &new_cloud, std::vector<int> &in_cloud_samples, ros::Time &stamp_current, const std_msgs::Header &header_current);

    void publishStackedPointCloud(const std_msgs::Header &header);
    void publishLocalMapDebug(const std_msgs::Header &header);



    std::vector<Pointf> downSampleCloud(const std::vector<Pointf>& cloud);


    CellIndex getCellIndex(const Pointf& point)
    {
        int x = std::floor(point.x * inv_voxel_size_);
        int y = std::floor(point.y * inv_voxel_size_);
        int z = std::floor(point.z * inv_voxel_size_);

        return {x, y, z};
    }

    PointPh getPointPh(const CellIndex& cell_index)
    {
        PointPh point;
        point[0] = std::get<0>(cell_index)*voxel_size_ + half_voxel_size_;
        point[1] = std::get<1>(cell_index)*voxel_size_ + half_voxel_size_;
        point[2] = std::get<2>(cell_index)*voxel_size_ + half_voxel_size_;

        return point;
    }


    void incrementBufferPointer()
    {
        buffer_pointer_ = (buffer_pointer_ + 1) % max_number_of_clouds_stored_;
    }

    std::vector<Pointf>& getOldestCloud()
    {
        return downsampled_clouds_[buffer_pointer_];
    }

    std::vector<Pointf>& getMiddleCloud()
    {
        return downsampled_clouds_[(buffer_pointer_ + number_of_clouds_used_as_padding_) % max_number_of_clouds_stored_];
    }

    std_msgs::Header& getMiddleHeader()
    {
        return headers_[(buffer_pointer_ + number_of_clouds_used_as_padding_) % max_number_of_clouds_stored_];
    }
};

stackedPointCloud::stackedPointCloud()
{
    // get the rosparams
    nh_.getParam("neighborhood_radius", neighbourhood_radius_);
    nh_.getParam("voxel_size", downsampled_voxel_size_);
    nh_.getParam("dynamic_threshold", dynamic_threshold_);
    nh_.getParam("number_of_clouds_used_as_padding", number_of_clouds_used_as_padding_);
    nh_.param("time_multiplier", time_multiplier_, 1e-9);

    inv_downsampled_voxel_size_ = 1.0/downsampled_voxel_size_;
    voxel_size_ = std::max(std::min(0.05,downsampled_voxel_size_), downsampled_voxel_size_/3.0);
    inv_voxel_size_ = 1.0/voxel_size_;
    half_voxel_size_ = voxel_size_/2.0;


    // set the subscribers
    sub = nh_.subscribe("registered_pointcloud", 100, &stackedPointCloud::processNewCloud, this);

    max_number_of_clouds_stored_ = 1 + 2*number_of_clouds_used_as_padding_;

    // set the publishers
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_dynamic_score_downsampled", 1000);
    pub_stacked_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("stacked_pointcloud", 1000);
    pub_local_map_debug_ = nh_.advertise<sensor_msgs::PointCloud2>("local_map_debug", 1000);

    downsampled_clouds_.resize(max_number_of_clouds_stored_);
    headers_.resize(max_number_of_clouds_stored_);
}


stackedPointCloud::~stackedPointCloud()
{
    
}



void stackedPointCloud::publishStackedPointCloud(const std_msgs::Header &header)
{

    if(pub_stacked_pointcloud_.getNumSubscribers() > 0)
    {
        int max_id = std::min(cloud_counter_, max_number_of_clouds_stored_);

        std::vector<Pointf> stacked_cloud;
        for(int i = 0; i < max_id; i++)
        {
            std::vector<Pointf> cloud = downsampled_clouds_[i];
            stacked_cloud.insert(stacked_cloud.end(), cloud.begin(), cloud.end());
        }

        sensor_msgs::PointCloud2 output = ptsVecToPointCloud2MsgInternal(stacked_cloud, header);

        pub_stacked_pointcloud_.publish(output);
    }
}

void stackedPointCloud::publishLocalMapDebug(const std_msgs::Header &header)
{
    if(pub_local_map_debug_.getNumSubscribers() > 0)
    {
        std::vector<Pointf> local_map_cloud;
        local_map_cloud.reserve(local_map_.size());
        for(const auto& [cell_index, cell] : local_map_)
        {
            Vec3 centroid = cell->getCentroid();
            Pointf point = {(float)centroid(0), (float)centroid(1), (float)centroid(2), 0.0};
            local_map_cloud.push_back(point);
        }

        sensor_msgs::PointCloud2 output = ptsVecToPointCloud2MsgInternal(local_map_cloud, header);
        pub_local_map_debug_.publish(output);
    }
}





void stackedPointCloud::processNewCloud(const sensor_msgs::PointCloud2ConstPtr &input)
{
    ROS_INFO("---------------------------------");

    ros::Time begin = ros::Time::now();

    // set the first timestamp to stamp_zero_
    if (cloud_counter_ == 0)
    {
        stamp_zero_ = input->header.stamp;
    }
    cloud_counter_++;


    auto [cloud, has_time] = pointCloud2MsgToPtsVec<float>(input, time_multiplier_, false);
    std::vector<Pointf> downsampled_cloud = downSampleCloud(cloud);
    double time_offset = (input->header.stamp - stamp_zero_).toSec();
    for(auto& point : downsampled_cloud)
    {
        point.t += time_offset;
    }

    ros::Time end = ros::Time::now();
    ROS_INFO_STREAM("Received and downsampled cloud in " << (end - begin).toSec()*1000 << " ms");





    ros::Time start_bis = ros::Time::now();

    // If buffer is full, remove the oldest cloud from the local map
    if(cloud_counter_ >= max_number_of_clouds_stored_)
    {
        std::vector<Pointf> oldest_cloud = getOldestCloud();


        for(const auto& point : oldest_cloud)
        {
            CellIndex cell_index = getCellIndex(point);
            auto it = local_map_.find(cell_index);
            if(it != local_map_.end())
            {
                if(it->second->removePoint(point))
                {
                    local_map_.erase(it);
                    phtree_.erase(getPointPh(cell_index));
                }
            }
        }
        end = ros::Time::now();
        ROS_INFO_STREAM("Removed oldest cloud from PhTree in " << (end - start_bis).toSec()*1000 << " ms");
    }



    start_bis = ros::Time::now();
    // Add the downsampled cloud to the local map (HashMap and PhTree)
    downsampled_clouds_[buffer_pointer_] = downsampled_cloud;
    headers_[buffer_pointer_] = input->header;

    for(const auto& point : downsampled_cloud)
    {
        CellIndex cell_index = getCellIndex(point);
        if(local_map_.find(cell_index) == local_map_.end())
        {
            CellPtr new_cell = std::make_shared<Cell>(point);
            local_map_[cell_index] = new_cell;
            phtree_.emplace(getPointPh(cell_index), new_cell);
        }
        else
        {
            local_map_[cell_index]->addPoint(point);
        }
    }

    end = ros::Time::now();
    ROS_INFO_STREAM("Added downsampled cloud to local map in " << (end - start_bis).toSec()*1000 << " ms");


    // If buffer is full, perform the dynamic detection
    if(cloud_counter_ >= max_number_of_clouds_stored_)
    {
        start_bis = ros::Time::now();
        
        std::vector<Pointf> middle_cloud = getMiddleCloud();

        std::vector<float> dynamic_scores(middle_cloud.size(), 0.0);
        double distance_threshold = neighbourhood_radius_ + std::sqrt(3)*voxel_size_;
        #pragma omp parallel for
        for(int i = 0; i < middle_cloud.size(); i++)
        {
            Pointf& point = middle_cloud[i];
            PointPh query_point = {(double)point.x, (double)point.y, (double)point.z};

            PhNeighborQuery query;
            phtree_.for_each(query, improbable::phtree::FilterSphere(query_point, distance_threshold, phtree_.converter()));

            Vec4 point_sum = Vec4::Zero();
            Mat4 covariance_sum = Mat4::Zero();
            int count = 0;

            for(const auto& [neighbor_point, neighbor_cell] : query.neighbors)
            {
                Vec3 centroid = neighbor_cell->getCentroid();
                if((centroid - point.vec3d()).norm() < neighbourhood_radius_)
                {
                    point_sum += neighbor_cell->point_sum;
                    covariance_sum += neighbor_cell->covariance_sum;
                    count += neighbor_cell->count;
                }
            }


            if(count > 10)
            {
                Vec4 mean = point_sum / (double)count;
                Mat4 covariance = covariance_sum / (double)count - mean * mean.transpose();

                Eigen::EigenSolver<Mat4> es(covariance);
                Vec4 eigenvalues = es.eigenvalues().real();
                Mat4 eigenvectors = es.eigenvectors().real();

                int min_index = 0;
                eigenvalues.minCoeff(&min_index);
                Vec4 smallest_eigenvector = eigenvectors.col(min_index);

                float dynamic_score = std::abs(smallest_eigenvector(3));
                dynamic_scores[i] = dynamic_score;
            }

        }

        end = ros::Time::now();
        ROS_INFO_STREAM("Dynamic score computation in " << (end - start_bis).toSec()*1000 << " ms");

        // publish the dynamic scores
        if (pub_.getNumSubscribers() > 0)
        {
            sensor_msgs::PointCloud2 output = cloudAndScoreToRosMsg(middle_cloud, dynamic_scores, getMiddleHeader());
            pub_.publish(output);
        }
    }


    // publish the stacked pointcloud
    publishStackedPointCloud(input->header);
    publishLocalMapDebug(input->header);


    incrementBufferPointer();

    end = ros::Time::now();
    ROS_INFO_STREAM("Total time (including prints and publishing) " << (end - begin).toSec()*1000 << " ms");
}


std::vector<Pointf> stackedPointCloud::downSampleCloud(const std::vector<Pointf>& cloud)
{
    // Downsample the cloud
    std::vector<Pointf> downsampled_cloud;
    downsampled_cloud.reserve(cloud.size());

    HashMap<Pointf> scan_hash_map;

    for (const auto& point : cloud)
    {
        if(point.vec3f().norm() > 0.1)
        {
            CellIndex cell_index = std::make_tuple(std::floor(point.x * inv_downsampled_voxel_size_),
                                                   std::floor(point.y * inv_downsampled_voxel_size_),
                                                   std::floor(point.z * inv_downsampled_voxel_size_));

            auto it = scan_hash_map.find(cell_index);
            scan_hash_map[cell_index] = point;
        }
    }

    for (const auto& [cell_index, point] : scan_hash_map)
    {
        downsampled_cloud.push_back(point);
    }

    return downsampled_cloud;
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
