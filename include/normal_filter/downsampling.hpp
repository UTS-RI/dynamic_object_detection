/*
 *   Point cloud downsampling using
 *   - voxel grid downsampling
 *   - skip sampling
 *   by R. Falque
 *   09/09/2023
 */

#ifndef DOWNSAMPLING
#define DOWNSAMPLING

#include <Eigen/Core>
#include <vector>
#include <tuple>
#include <map>
#include <iostream>

#include "ankerl/unordered_dense.h"


static inline void getMinMax(const Eigen::MatrixXf &in_cloud, Eigen::Vector3f &min_point, Eigen::Vector3f &max_point)
{
	max_point = in_cloud.rowwise().maxCoeff();
	min_point = in_cloud.rowwise().minCoeff();
};

inline void getScale(const Eigen::MatrixXf &in_cloud, double &scale)
{
	Eigen::Vector3f min_point, max_point;
	getMinMax(in_cloud, min_point, max_point);
	scale = (max_point - min_point).norm();
};


/* voxel grid downsampling, go through each point of the cloud and allocate it to a voxel
 * use the last allocated point as the final point used in the downsampled version.
 * INPUT:
 * - input_cloud: 4xn matrix of the input cloud with the last row being the timestamp
 * - grid resolution: define the size of the voxel as the diagonal of the bounding box divided by the grid resolution
 * OUTPUT:
 * - output_cloud: 4xm matrix of the downsampled cloud with the last row being the timestamp
 * - input_cloud_samples: vector of the index of the points in the input cloud that are used in the output cloud
 */
std::tuple<Eigen::MatrixXf, std::vector<int>> voxel_grid_downsampling(Eigen::MatrixXf &input_cloud_with_timestamps, float voxel_size)
{
	// split the input cloud into the point cloud and the timestamps
	Eigen::MatrixXf input_cloud = input_cloud_with_timestamps.topRows(3);
	Eigen::VectorXf input_time = input_cloud_with_timestamps.row(3);

	/*
	// define the voxel size as proportional to the scale of the cloud divided by the grid resolution
	double scale;
	getScale(input_cloud, scale);
	double voxel_size = scale / grid_resolution;

	std::cout << "voxel size: " << voxel_size << std::endl;
	*/

	// create the voxel structure as anunordered map with tuple of {x, y, z} as a key and a vector of {time, index} as a value
	ankerl::unordered_dense::map<std::tuple<int, int, int>, int> voxels;

	// transform the input_cloud into a matrix of voxel indices
	Eigen::MatrixXi input_cloud_voxel_indices = ((input_cloud / voxel_size).array().floor().matrix()).cast<int>();

	// fill up the voxel struture with the last point of each voxel
	for (int i = 0; i < input_cloud.cols(); ++i)
	{
		voxels[{input_cloud_voxel_indices(0, i), input_cloud_voxel_indices(1, i), input_cloud_voxel_indices(2, i)}] =  i;
	}

	// create the outputs
	Eigen::MatrixXf output_cloud_with_timestamps = Eigen::MatrixXf::Zero(4, voxels.size());
	std::vector<int> input_cloud_samples;

	// fill up the pointcoud output
	int output_index = 0;
	for (auto & [key, val] : voxels)
	{
		// add the point to the output cloud
		output_cloud_with_timestamps.col(output_index) = input_cloud_with_timestamps.col(val);
		input_cloud_samples.push_back(val);
		output_index++;
	}

	return {output_cloud_with_timestamps, input_cloud_samples};
};

#endif