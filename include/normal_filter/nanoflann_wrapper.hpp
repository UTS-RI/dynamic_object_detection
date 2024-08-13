/*
 *   nanoflann wrapper
 *   by R. Falque
 *
 *   History:
 *   21/11/2018 : first version
 *   16/01/2020 : fix bug with target being passed by reference and not stored
 *   internally
 *   23/09/2023 : rewrite the wrapper to test multithreading for construction (ends up being slower)
 */

#ifndef NANOFLANN_WRAPPER
#define NANOFLANN_WRAPPER

#include <Eigen/Dense>
#include <iostream>
#include <memory>

#include <cstdlib>
#include <iostream>
#include <vector>

#include "nanoflann.hpp"

// define the pointcloud data type
template <typename T>
struct PointCloud
{
    struct Point
    {
        T x, y, z;
    };

    using coord_t = T;  //!< The type of each coordinate

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return pts[idx].x;
        else if (dim == 1)
            return pts[idx].y;
        else
            return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard
    // bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned
    //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
    //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const
    {
        return false;
    }
};





template <class num_t>
class nanoflann_wrapper {
    private:
        PointCloud<num_t> cloud;
        std::shared_ptr<nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t> >,
            PointCloud<num_t>, 3> >
            kd_tree_index;

    public:
        nanoflann_wrapper(const Eigen::Matrix<num_t, Eigen::Dynamic, Eigen::Dynamic>& target) {
            //nanoflann_wrapper(Eigen::MatrixXf& target) {
            // check inputs and fill up the structure
            if (target.cols() == 3) {
                cloud.pts.resize(target.rows());
                for (int i = 0; i < target.rows(); i++) {
                    cloud.pts[i].x = target(i, 0);
                    cloud.pts[i].y = target(i, 1);
                    cloud.pts[i].z = target(i, 2);
                }
            } else if (target.rows() == 3) {
                cloud.pts.resize(target.cols());
                for (int i = 0; i < target.cols(); i++) {
                    cloud.pts[i].x = target(0, i);
                    cloud.pts[i].y = target(1, i);
                    cloud.pts[i].z = target(2, i);
                }
            } else {
                throw std::invalid_argument("Error in " + std::string(__func__) + ": wrong input size");
            }

            // create the kdtree
            int dimensionality = 3;

            // nanoflann supports concurrent build
            nanoflann::KDTreeSingleIndexAdaptorParams params{};
            params.n_thread_build = 1;
            params.leaf_max_size = 100;

            this->kd_tree_index =
                std::make_shared<nanoflann::KDTreeSingleIndexAdaptor<
                    nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t> >,
                    PointCloud<num_t>, 3> >(
                    dimensionality, cloud, params);

            this->kd_tree_index->buildIndex();
        }

        ~nanoflann_wrapper() {}

        std::vector<int> KNN_search(Eigen::Matrix<num_t, Eigen::Dynamic, 1> query_point, int k) {
            // Query point:
            std::vector<num_t> query_pt;
            for (int d = 0; d < 3; d++) query_pt.push_back(query_point(d));

            // set wtf vectors
            std::vector<size_t> ret_indexes(k);
            std::vector<num_t> out_dists_sqr(k);
            nanoflann::KNNResultSet<num_t> resultSet(k);
            resultSet.init(&ret_indexes.at(0), &out_dists_sqr.at(0));

            // knn search
            this->kd_tree_index->findNeighbors(resultSet, &query_pt.at(0),
                                               nanoflann::SearchParameters(k));

            // pack result into std::vector<int>
            std::vector<int> indexes;
            for (int i = 0; i < k; i++) indexes.push_back(ret_indexes.at(i));

            return indexes;
        }

        bool KNN_search(Eigen::Matrix<num_t, Eigen::Dynamic, 1> query_point, int k,
                        std::vector<int>& indexes, std::vector<num_t>& distances) {
            // Query point:
            std::vector<num_t> query_pt;
            for (int d = 0; d < 3; d++) query_pt.push_back(query_point(d));

            indexes.clear();
            distances.clear();

            // set wtf vectors
            std::vector<size_t> ret_indexes(k);
            std::vector<num_t> out_dists_sqr(k);
            nanoflann::KNNResultSet<num_t> resultSet(k);
            resultSet.init(&ret_indexes.at(0), &out_dists_sqr.at(0));

            // knn search
            this->kd_tree_index->findNeighbors(resultSet, &query_pt.at(0),
                                               nanoflann::SearchParameters(k));

            // pack results back into std::vector<int>
            indexes.reserve(ret_indexes.size());
            distances.reserve(out_dists_sqr.size());
            for (int i = 0; i < ret_indexes.size(); i++) {
                indexes.push_back(ret_indexes.at(i));
                distances.push_back(out_dists_sqr.at(i));
            }

            return true;
        }

        std::vector<int> radius_search(Eigen::Matrix<num_t, Eigen::Dynamic, 1> query_point, num_t max_dist) {
            // Query point:
            std::vector<num_t> query_pt;
            for (int d = 0; d < 3; d++) query_pt.push_back(query_point(d));

            const num_t search_radius = static_cast<num_t>(max_dist);
            std::vector<nanoflann::ResultItem<uint32_t, num_t>> ret_matches;

            // nanoflanSearchParamsameters params;
            // params.sorted = false;

            const size_t nMatches =
                this->kd_tree_index->radiusSearch(&query_pt[0], search_radius, ret_matches);
            
            std::vector<int> indexes;
            indexes.reserve(nMatches);
            for (size_t i = 0; i < nMatches; i++)
                indexes.push_back(ret_matches[i].first);

            return indexes;
        }

        bool radius_search(Eigen::Matrix<num_t, Eigen::Dynamic, 1> query_point, num_t max_dist,
                        std::vector<int>& indexes, std::vector<num_t>& distances) {
            // Query point:
            std::vector<num_t> query_pt;
            for (int d = 0; d < 3; d++) query_pt.push_back(query_point(d));

            const num_t search_radius = static_cast<num_t>(max_dist);
            std::vector<nanoflann::ResultItem<uint32_t, num_t>> ret_matches;

            // nanoflanSearchParamsameters params;
            // params.sorted = false;

            const size_t nMatches =
                this->kd_tree_index->radiusSearch(&query_pt[0], search_radius, ret_matches);
            
            indexes.clear();
            distances.clear();

            for (size_t i = 0; i < nMatches; i++) {
                indexes.push_back(ret_matches[i].first);
                distances.push_back(ret_matches[i].second);
            }

            return true;
        }

};

#endif
