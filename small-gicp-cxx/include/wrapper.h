#pragma once

#include <memory>
#include <vector>
#include <array>
#include <stdexcept>

#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/util/downsampling.hpp>
#include <small_gicp/util/normal_estimation.hpp>
#include <small_gicp/registration/registration.hpp>

#include "rust/cxx.h"

namespace small_gicp_cxx {

// Forward declarations for shared structs - actual definitions come from cxx
struct Point3d;
struct Transform;
struct RegistrationResult;
struct RegistrationSettings;

// Wrapper class for PointCloud
class PointCloud {
public:
    PointCloud();
    
    // Point access
    size_t size() const;
    void reserve(size_t n);
    void resize(size_t n);
    void clear();
    
    // Add points
    void add_point(Point3d point);
    void set_point(size_t index, Point3d point);
    Point3d get_point(size_t index) const;
    
    // Get raw data for efficient access
    rust::Slice<const double> points_data() const;
    rust::Slice<const double> normals_data() const;
    rust::Slice<const double> covs_data() const;
    
    // Compute normals and covariances
    void estimate_normals(int num_neighbors, int num_threads);
    void estimate_covariances(int num_neighbors, int num_threads);
    
    // Downsampling
    std::unique_ptr<PointCloud> voxel_downsample(double voxel_size, int num_threads) const;
    
    // Internal access
    small_gicp::PointCloud& get_internal() { return *cloud_; }
    const small_gicp::PointCloud& get_internal() const { return *cloud_; }
    
private:
    std::shared_ptr<small_gicp::PointCloud> cloud_;
};

// Wrapper class for KdTree
class KdTree {
public:
    explicit KdTree(const PointCloud& cloud, int num_threads = 1);
    
    // Nearest neighbor search
    size_t nearest_neighbor(Point3d point) const;
    rust::Vec<size_t> knn_search(Point3d point, size_t k) const;
    rust::Vec<size_t> radius_search(Point3d point, double radius) const;
    
    // Internal access for registration
    small_gicp::KdTree<small_gicp::PointCloud>& get_internal() { return *tree_; }
    const small_gicp::KdTree<small_gicp::PointCloud>& get_internal() const { return *tree_; }
    
private:
    std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> tree_;
};

// Wrapper class for GaussianVoxelMap
class GaussianVoxelMap {
public:
    explicit GaussianVoxelMap(double voxel_size);
    
    void insert(const PointCloud& cloud);
    size_t size() const;
    
    // Internal access for registration
    small_gicp::GaussianVoxelMap& get_internal() { return *voxelmap_; }
    const small_gicp::GaussianVoxelMap& get_internal() const { return *voxelmap_; }
    
private:
    std::shared_ptr<small_gicp::GaussianVoxelMap> voxelmap_;
};

// Registration functions
RegistrationResult align_points_icp(
    const PointCloud& source,
    const PointCloud& target,
    const KdTree& target_tree,
    const Transform& init_guess,
    const RegistrationSettings& settings
);

RegistrationResult align_points_point_to_plane_icp(
    const PointCloud& source,
    const PointCloud& target,
    const KdTree& target_tree,
    const Transform& init_guess,
    const RegistrationSettings& settings
);

RegistrationResult align_points_gicp(
    const PointCloud& source,
    const PointCloud& target,
    const KdTree& source_tree,
    const KdTree& target_tree,
    const Transform& init_guess,
    const RegistrationSettings& settings
);

RegistrationResult align_points_vgicp(
    const PointCloud& source,
    const GaussianVoxelMap& target_voxelmap,
    const Transform& init_guess,
    const RegistrationSettings& settings
);

// Utility functions
std::unique_ptr<PointCloud> create_point_cloud();
std::unique_ptr<KdTree> create_kdtree(const PointCloud& cloud, int num_threads);
std::unique_ptr<GaussianVoxelMap> create_voxelmap(double voxel_size);

} // namespace small_gicp_cxx