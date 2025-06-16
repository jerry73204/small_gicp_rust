#pragma once

#include <array>
#include <memory>
#include <stdexcept>
#include <vector>

#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling.hpp>
#include <small_gicp/util/normal_estimation.hpp>

#include "rust/cxx.h"

namespace small_gicp_cxx {

// Forward declarations for shared structs - actual definitions come from cxx
struct Point3d;
struct Transform;
struct KdTreeSettings;
struct RegistrationResult;
struct RegistrationSettings;
struct NearestNeighborResult;
struct KnnSearchResult;
struct GaussianVoxelData;
struct VoxelInfoData;

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

  // Bulk operations for performance
  void set_points_bulk(rust::Slice<const double> points);
  void set_normals_bulk(rust::Slice<const double> normals);
  void set_covariances_bulk(rust::Slice<const double> covariances);

  // Transformation operations
  void transform(const Transform &transform);
  std::unique_ptr<PointCloud> transformed(const Transform &transform) const;

  // Compute normals and covariances
  void estimate_normals(int num_neighbors, int num_threads);
  void estimate_covariances(int num_neighbors, int num_threads);

  // Downsampling
  std::unique_ptr<PointCloud> voxel_downsample(double voxel_size,
                                               int num_threads) const;

  // Internal access
  small_gicp::PointCloud &get_internal() { return *cloud_; }
  const small_gicp::PointCloud &get_internal() const { return *cloud_; }

private:
  std::shared_ptr<small_gicp::PointCloud> cloud_;
};

// Wrapper class for KdTree
class KdTree {
public:
  explicit KdTree(const PointCloud &cloud, int num_threads = 1);

  // Nearest neighbor search
  size_t nearest_neighbor(Point3d point) const;
  rust::Vec<size_t> knn_search(Point3d point, size_t k) const;
  rust::Vec<size_t> radius_search(Point3d point, double radius) const;

  // Distance-returning search methods
  NearestNeighborResult nearest_neighbor_with_distance(Point3d point) const;
  KnnSearchResult knn_search_with_distances(Point3d point, size_t k) const;
  KnnSearchResult radius_search_with_distances(Point3d point,
                                               double radius) const;

  // Internal access for registration
  small_gicp::KdTree<small_gicp::PointCloud> &get_internal() { return *tree_; }
  const small_gicp::KdTree<small_gicp::PointCloud> &get_internal() const {
    return *tree_;
  }

private:
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> tree_;
};

// Wrapper class for UnsafeKdTree (high-performance variant)
class UnsafeKdTree {
public:
  explicit UnsafeKdTree(const PointCloud &cloud, int num_threads = 1);

  // Zero-copy constructor from raw point data
  UnsafeKdTree(const double *points_data, size_t num_points,
               const KdTreeSettings &settings);

  // Nearest neighbor search
  size_t unsafe_nearest_neighbor(Point3d point) const;
  rust::Vec<size_t> unsafe_knn_search(Point3d point, size_t k) const;
  rust::Vec<size_t> unsafe_radius_search(Point3d point, double radius) const;

  // Distance-returning unsafe search methods
  NearestNeighborResult
  unsafe_nearest_neighbor_with_distance(Point3d point) const;
  KnnSearchResult unsafe_knn_search_with_distances(Point3d point,
                                                   size_t k) const;
  KnnSearchResult unsafe_radius_search_with_distances(Point3d point,
                                                      double radius) const;

  // Zero-copy validation
  bool unsafe_validate_data_ptr(const double *expected_ptr) const;

  // Internal access for registration
  small_gicp::UnsafeKdTree<small_gicp::PointCloud> &get_internal() {
    return *tree_;
  }
  const small_gicp::UnsafeKdTree<small_gicp::PointCloud> &get_internal() const {
    return *tree_;
  }

private:
  std::shared_ptr<small_gicp::UnsafeKdTree<small_gicp::PointCloud>> tree_;
  const double *original_data_ptr_; // For zero-copy validation
};

// Wrapper class for GaussianVoxelMap
class GaussianVoxelMap {
public:
  explicit GaussianVoxelMap(double voxel_size);

  void insert(const PointCloud &cloud);
  size_t size() const;
  double get_voxel_size() const;
  size_t get_num_voxels() const;
  void clear_voxels();
  bool has_voxel_at_coords(int x, int y, int z) const;

  // Internal access for registration
  small_gicp::GaussianVoxelMap &get_internal() { return *voxelmap_; }
  const small_gicp::GaussianVoxelMap &get_internal() const {
    return *voxelmap_;
  }

private:
  std::shared_ptr<small_gicp::GaussianVoxelMap> voxelmap_;
};

// Wrapper class for IncrementalVoxelMap
class IncrementalVoxelMap {
public:
  explicit IncrementalVoxelMap(double voxel_size);

  void incremental_insert(const PointCloud &cloud);
  size_t incremental_size() const;
  void incremental_clear();
  void incremental_finalize();
  double incremental_get_voxel_size() const;
  size_t incremental_get_num_voxels() const;
  void incremental_insert_point(double x, double y, double z);
  void incremental_insert_with_transform(const PointCloud &cloud,
                                         const Transform &transform);
  bool incremental_has_voxel_at_coords(int x, int y, int z) const;
  void incremental_set_search_offsets(int num_offsets);
  GaussianVoxelData incremental_get_voxel_data(int x, int y, int z) const;
  rust::Vec<VoxelInfoData>
  incremental_find_voxels_in_radius(double x, double y, double z,
                                    double radius) const;
  NearestNeighborResult incremental_nearest_neighbor_search(double x, double y,
                                                            double z) const;
  KnnSearchResult incremental_knn_search(double x, double y, double z,
                                         size_t k) const;
  std::array<int32_t, 3> incremental_get_voxel_coords(double x, double y,
                                                      double z) const;
  size_t incremental_get_voxel_index(int x, int y, int z) const;
  GaussianVoxelData incremental_get_gaussian_voxel_by_index(size_t index) const;

  // Internal access for registration
  small_gicp::IncrementalVoxelMap<small_gicp::GaussianVoxel> &get_internal() {
    return *voxelmap_;
  }
  const small_gicp::IncrementalVoxelMap<small_gicp::GaussianVoxel> &
  get_internal() const {
    return *voxelmap_;
  }

private:
  std::shared_ptr<small_gicp::IncrementalVoxelMap<small_gicp::GaussianVoxel>>
      voxelmap_;
};

// Registration functions
RegistrationResult align_points_icp(const PointCloud &source,
                                    const PointCloud &target,
                                    const KdTree &target_tree,
                                    const Transform &init_guess,
                                    const RegistrationSettings &settings);

RegistrationResult align_points_point_to_plane_icp(
    const PointCloud &source, const PointCloud &target,
    const KdTree &target_tree, const Transform &init_guess,
    const RegistrationSettings &settings);

RegistrationResult align_points_gicp(const PointCloud &source,
                                     const PointCloud &target,
                                     const KdTree &source_tree,
                                     const KdTree &target_tree,
                                     const Transform &init_guess,
                                     const RegistrationSettings &settings);

RegistrationResult align_points_vgicp(const PointCloud &source,
                                      const GaussianVoxelMap &target_voxelmap,
                                      const Transform &init_guess,
                                      const RegistrationSettings &settings);

// Utility functions
std::unique_ptr<PointCloud> create_point_cloud();
std::unique_ptr<KdTree> create_kdtree(const PointCloud &cloud, int num_threads);
std::unique_ptr<UnsafeKdTree> create_unsafe_kdtree(const PointCloud &cloud,
                                                   int num_threads);
std::unique_ptr<UnsafeKdTree>
create_unsafe_kdtree_from_points_ptr(const double *points_data,
                                     size_t num_points,
                                     const KdTreeSettings &settings);
std::unique_ptr<GaussianVoxelMap> create_voxelmap(double voxel_size);
std::unique_ptr<IncrementalVoxelMap>
create_incremental_voxelmap(double voxel_size);

// Preprocessing functions
std::unique_ptr<PointCloud> downsample_voxelgrid(const PointCloud &cloud,
                                                 double voxel_size,
                                                 int num_threads);
std::unique_ptr<PointCloud> downsample_random(const PointCloud &cloud,
                                              size_t num_samples);
void compute_normals(PointCloud &cloud, int num_neighbors, int num_threads);
void compute_covariances(PointCloud &cloud, int num_neighbors, int num_threads);

// I/O functions
std::unique_ptr<PointCloud> load_ply(rust::Str filename);
void save_ply(rust::Str filename, const PointCloud &cloud);

} // namespace small_gicp_cxx
