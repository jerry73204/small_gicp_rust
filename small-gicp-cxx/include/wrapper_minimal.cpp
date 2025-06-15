#include "small-gicp-cxx/src/ffi.rs.h"
#include "wrapper.h"

#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/incremental_voxelmap.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

#include <algorithm>
#include <fstream>
#include <numeric>
#include <random>

#include <limits>
#include <small_gicp/benchmark/read_points.hpp>

namespace small_gicp_cxx {

// PointCloud implementation - minimal
PointCloud::PointCloud() : cloud_(std::make_shared<small_gicp::PointCloud>()) {}

size_t PointCloud::size() const { return cloud_->size(); }

void PointCloud::reserve(size_t n) {
  // No-op for now
}

void PointCloud::resize(size_t n) { cloud_->resize(n); }

void PointCloud::clear() { cloud_->resize(0); }

void PointCloud::add_point(Point3d point) {
  size_t current_size = cloud_->size();
  cloud_->resize(current_size + 1);
  cloud_->point(current_size) = Eigen::Vector4d(point.x, point.y, point.z, 1.0);
}

void PointCloud::set_point(size_t index, Point3d point) {
  if (index < cloud_->size()) {
    cloud_->point(index) = Eigen::Vector4d(point.x, point.y, point.z, 1.0);
  }
}

Point3d PointCloud::get_point(size_t index) const {
  if (index >= cloud_->size()) {
    throw std::out_of_range("Point index out of range");
  }
  const auto &p = cloud_->point(index);
  return Point3d{p.x(), p.y(), p.z()};
}

rust::Slice<const double> PointCloud::points_data() const {
  if (cloud_->empty()) {
    return rust::Slice<const double>();
  }
  return rust::Slice<const double>(cloud_->points.data()->data(),
                                   cloud_->points.size() * 4);
}

rust::Slice<const double> PointCloud::normals_data() const {
  if (cloud_->empty() || cloud_->normals.empty()) {
    return rust::Slice<const double>();
  }
  return rust::Slice<const double>(cloud_->normals.data()->data(),
                                   cloud_->normals.size() * 4);
}

rust::Slice<const double> PointCloud::covs_data() const {
  if (cloud_->empty() || cloud_->covs.empty()) {
    return rust::Slice<const double>();
  }
  return rust::Slice<const double>(cloud_->covs.data()->data(),
                                   cloud_->covs.size() * 16);
}

void PointCloud::estimate_normals(int num_neighbors, int num_threads) {
  if (num_threads > 1) {
    small_gicp::estimate_normals_omp(*cloud_, num_neighbors, num_threads);
  } else {
    small_gicp::estimate_normals(*cloud_, num_neighbors);
  }
}

void PointCloud::estimate_covariances(int num_neighbors, int num_threads) {
  if (num_threads > 1) {
    small_gicp::estimate_covariances_omp(*cloud_, num_neighbors, num_threads);
  } else {
    small_gicp::estimate_covariances(*cloud_, num_neighbors);
  }
}

// Bulk operations for performance
void PointCloud::set_points_bulk(rust::Slice<const double> points) {
  if (points.length() % 4 != 0) {
    throw std::invalid_argument(
        "Points data must be a multiple of 4 (x, y, z, w)");
  }

  size_t num_points = points.length() / 4;
  cloud_->resize(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    cloud_->point(i) = Eigen::Vector4d(points[i * 4 + 0], points[i * 4 + 1],
                                       points[i * 4 + 2], points[i * 4 + 3]);
  }
}

void PointCloud::set_normals_bulk(rust::Slice<const double> normals) {
  if (normals.length() % 4 != 0) {
    throw std::invalid_argument(
        "Normals data must be a multiple of 4 (nx, ny, nz, 0)");
  }

  size_t num_normals = normals.length() / 4;
  cloud_->normals.resize(num_normals);

  for (size_t i = 0; i < num_normals; ++i) {
    cloud_->normals[i] =
        Eigen::Vector4d(normals[i * 4 + 0], normals[i * 4 + 1],
                        normals[i * 4 + 2], normals[i * 4 + 3]);
  }
}

void PointCloud::set_covariances_bulk(rust::Slice<const double> covariances) {
  if (covariances.length() % 16 != 0) {
    throw std::invalid_argument(
        "Covariances data must be a multiple of 16 (4x4 matrix)");
  }

  size_t num_covariances = covariances.length() / 16;
  cloud_->covs.resize(num_covariances);

  for (size_t i = 0; i < num_covariances; ++i) {
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        cloud_->covs[i](row, col) = covariances[i * 16 + row * 4 + col];
      }
    }
  }
}

// Transformation operations
void PointCloud::transform(const Transform &transform) {
  // Convert flat array to Eigen transformation matrix
  Eigen::Matrix4d transform_matrix;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      transform_matrix(i, j) = transform.matrix[i * 4 + j];
    }
  }
  Eigen::Isometry3d T(transform_matrix);

  // Transform all points
  for (size_t i = 0; i < cloud_->size(); ++i) {
    Eigen::Vector3d point = cloud_->point(i).head<3>();
    point = T * point;
    cloud_->point(i).head<3>() = point;
  }

  // Transform normals (only rotation part)
  if (!cloud_->normals.empty()) {
    Eigen::Matrix3d rotation = T.rotation();
    for (size_t i = 0; i < cloud_->normals.size(); ++i) {
      Eigen::Vector3d normal = cloud_->normals[i].head<3>();
      normal = rotation * normal;
      cloud_->normals[i].head<3>() = normal;
    }
  }

  // Transform covariances (R * C * R^T)
  if (!cloud_->covs.empty()) {
    Eigen::Matrix3d rotation = T.rotation();
    for (size_t i = 0; i < cloud_->covs.size(); ++i) {
      Eigen::Matrix3d cov = cloud_->covs[i].block<3, 3>(0, 0);
      cov = rotation * cov * rotation.transpose();
      cloud_->covs[i].block<3, 3>(0, 0) = cov;
    }
  }
}

std::unique_ptr<PointCloud>
PointCloud::transformed(const Transform &transform) const {
  auto result = std::make_unique<PointCloud>();

  // Copy the original cloud
  result->cloud_ = std::make_shared<small_gicp::PointCloud>(*cloud_);

  // Apply transformation to the copy
  result->transform(transform);

  return result;
}

std::unique_ptr<PointCloud>
PointCloud::voxel_downsample(double voxel_size, int num_threads) const {
  auto result = std::make_unique<PointCloud>();

  if (num_threads > 1) {
    result->cloud_ =
        small_gicp::voxelgrid_sampling_omp(*cloud_, voxel_size, num_threads);
  } else {
    result->cloud_ = small_gicp::voxelgrid_sampling(*cloud_, voxel_size);
  }

  return result;
}

// KdTree implementation with parallel support
KdTree::KdTree(const PointCloud &cloud, int num_threads) {
  auto cloud_ptr =
      std::make_shared<small_gicp::PointCloud>(cloud.get_internal());
  if (num_threads > 1) {
    // Use OpenMP builder for parallel construction
    small_gicp::KdTreeBuilderOMP builder(num_threads);
    tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
        cloud_ptr, builder);
  } else {
    // Use default sequential builder
    tree_ =
        std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(cloud_ptr);
  }
}

size_t KdTree::nearest_neighbor(Point3d point) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  size_t index;
  double sq_distance;
  if (tree_->nearest_neighbor_search(query, &index, &sq_distance) == 1) {
    return index;
  }
  return SIZE_MAX; // Not found
}

rust::Vec<size_t> KdTree::knn_search(Point3d point, size_t k) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  std::vector<size_t> indices(k);
  std::vector<double> sq_distances(k);

  size_t num_found =
      tree_->knn_search(query, k, indices.data(), sq_distances.data());

  rust::Vec<size_t> result;
  for (size_t i = 0; i < num_found; ++i) {
    result.push_back(indices[i]);
  }
  return result;
}

rust::Vec<size_t> KdTree::radius_search(Point3d point, double radius) const {
  // Note: KdTree doesn't have a built-in radius_search method in small_gicp
  // This is a placeholder implementation that returns empty results
  // In practice, radius search can be implemented using knn_search with
  // filtering
  rust::Vec<size_t> result;
  return result;
}

// Distance-returning search methods
NearestNeighborResult
KdTree::nearest_neighbor_with_distance(Point3d point) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  size_t index;
  double sq_distance;
  NearestNeighborResult result;

  if (tree_->nearest_neighbor_search(query, &index, &sq_distance) == 1) {
    result.index = index;
    result.squared_distance = sq_distance;
  } else {
    result.index = SIZE_MAX;
    result.squared_distance = std::numeric_limits<double>::infinity();
  }

  return result;
}

KnnSearchResult KdTree::knn_search_with_distances(Point3d point,
                                                  size_t k) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  std::vector<size_t> indices(k);
  std::vector<double> sq_distances(k);

  size_t num_found =
      tree_->knn_search(query, k, indices.data(), sq_distances.data());

  KnnSearchResult result;
  for (size_t i = 0; i < num_found; ++i) {
    result.indices.push_back(indices[i]);
    result.squared_distances.push_back(sq_distances[i]);
  }
  return result;
}

KnnSearchResult KdTree::radius_search_with_distances(Point3d point,
                                                     double radius) const {
  // Note: This is a placeholder implementation using knn_search with filtering
  // A proper implementation would require radius_search in the underlying
  // library
  KnnSearchResult result;

  // Use a large k for approximate radius search
  const size_t max_k = 1000;
  auto knn_result = knn_search_with_distances(point, max_k);

  double radius_sq = radius * radius;
  for (size_t i = 0; i < knn_result.indices.size(); ++i) {
    if (knn_result.squared_distances[i] <= radius_sq) {
      result.indices.push_back(knn_result.indices[i]);
      result.squared_distances.push_back(knn_result.squared_distances[i]);
    }
  }

  return result;
}

// Minimal GaussianVoxelMap
GaussianVoxelMap::GaussianVoxelMap(double voxel_size)
    : voxelmap_(std::make_shared<small_gicp::GaussianVoxelMap>(voxel_size)) {}

void GaussianVoxelMap::insert(const PointCloud &cloud) {
  voxelmap_->insert(cloud.get_internal());
}

size_t GaussianVoxelMap::size() const { return voxelmap_->size(); }

// ICP registration implementation
RegistrationResult align_points_icp(const PointCloud &source,
                                    const PointCloud &target,
                                    const KdTree &target_tree,
                                    const Transform &init_guess,
                                    const RegistrationSettings &settings) {
  // Convert flat array to Eigen Isometry3d
  Eigen::Matrix4d init_matrix;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      init_matrix(i, j) = init_guess.matrix[i * 4 + j];
    }
  }
  Eigen::Isometry3d init_T(init_matrix);

  // Create registration settings
  small_gicp::RegistrationSetting reg_setting;
  reg_setting.type = small_gicp::RegistrationSetting::ICP;
  reg_setting.max_correspondence_distance =
      settings.max_correspondence_distance;
  reg_setting.rotation_eps = settings.rotation_epsilon;
  reg_setting.translation_eps = settings.transformation_epsilon;
  reg_setting.max_iterations = settings.max_iterations;
  reg_setting.num_threads = settings.num_threads;

  // Perform registration
  auto result =
      small_gicp::align(target.get_internal(), source.get_internal(),
                        target_tree.get_internal(), init_T, reg_setting);

  // Convert result back to our format
  RegistrationResult reg_result;
  Eigen::Matrix4d result_matrix = result.T_target_source.matrix();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      reg_result.transformation.matrix[i * 4 + j] = result_matrix(i, j);
    }
  }
  reg_result.converged = result.converged;
  reg_result.iterations = result.iterations;
  reg_result.error = result.error;

  return reg_result;
}

// Point-to-Plane ICP registration implementation
RegistrationResult align_points_point_to_plane_icp(
    const PointCloud &source, const PointCloud &target,
    const KdTree &target_tree, const Transform &init_guess,
    const RegistrationSettings &settings) {
  // Convert flat array to Eigen Isometry3d
  Eigen::Matrix4d init_matrix;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      init_matrix(i, j) = init_guess.matrix[i * 4 + j];
    }
  }
  Eigen::Isometry3d init_T(init_matrix);

  // Create registration settings
  small_gicp::RegistrationSetting reg_setting;
  reg_setting.type = small_gicp::RegistrationSetting::PLANE_ICP;
  reg_setting.max_correspondence_distance =
      settings.max_correspondence_distance;
  reg_setting.rotation_eps = settings.rotation_epsilon;
  reg_setting.translation_eps = settings.transformation_epsilon;
  reg_setting.max_iterations = settings.max_iterations;
  reg_setting.num_threads = settings.num_threads;

  // Perform registration
  auto result =
      small_gicp::align(target.get_internal(), source.get_internal(),
                        target_tree.get_internal(), init_T, reg_setting);

  // Convert result back to our format
  RegistrationResult reg_result;
  Eigen::Matrix4d result_matrix = result.T_target_source.matrix();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      reg_result.transformation.matrix[i * 4 + j] = result_matrix(i, j);
    }
  }
  reg_result.converged = result.converged;
  reg_result.iterations = result.iterations;
  reg_result.error = result.error;

  return reg_result;
}

RegistrationResult align_points_gicp(const PointCloud &source,
                                     const PointCloud &target,
                                     const KdTree &source_tree,
                                     const KdTree &target_tree,
                                     const Transform &init_guess,
                                     const RegistrationSettings &settings) {
  // Convert flat array to Eigen Isometry3d
  Eigen::Matrix4d init_matrix;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      init_matrix(i, j) = init_guess.matrix[i * 4 + j];
    }
  }
  Eigen::Isometry3d init_T(init_matrix);

  // Create registration settings
  small_gicp::RegistrationSetting reg_setting;
  reg_setting.type = small_gicp::RegistrationSetting::GICP;
  reg_setting.max_correspondence_distance =
      settings.max_correspondence_distance;
  reg_setting.rotation_eps = settings.rotation_epsilon;
  reg_setting.translation_eps = settings.transformation_epsilon;
  reg_setting.max_iterations = settings.max_iterations;
  reg_setting.num_threads = settings.num_threads;

  // Perform registration (we only use target_tree, source_tree is ignored in
  // helper)
  auto result =
      small_gicp::align(target.get_internal(), source.get_internal(),
                        target_tree.get_internal(), init_T, reg_setting);

  // Convert result back to our format
  RegistrationResult reg_result;
  Eigen::Matrix4d result_matrix = result.T_target_source.matrix();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      reg_result.transformation.matrix[i * 4 + j] = result_matrix(i, j);
    }
  }
  reg_result.converged = result.converged;
  reg_result.iterations = result.iterations;
  reg_result.error = result.error;

  return reg_result;
}

RegistrationResult align_points_vgicp(const PointCloud &source,
                                      const GaussianVoxelMap &target_voxelmap,
                                      const Transform &init_guess,
                                      const RegistrationSettings &settings) {
  // Convert flat array to Eigen Isometry3d
  Eigen::Matrix4d init_matrix;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      init_matrix(i, j) = init_guess.matrix[i * 4 + j];
    }
  }
  Eigen::Isometry3d init_T(init_matrix);

  // Create registration settings
  small_gicp::RegistrationSetting reg_setting;
  reg_setting.type = small_gicp::RegistrationSetting::VGICP;
  reg_setting.max_correspondence_distance =
      settings.max_correspondence_distance;
  reg_setting.rotation_eps = settings.rotation_epsilon;
  reg_setting.translation_eps = settings.transformation_epsilon;
  reg_setting.max_iterations = settings.max_iterations;
  reg_setting.num_threads = settings.num_threads;
  reg_setting.voxel_resolution = 1.0; // Use default or could be configurable

  // Perform registration with voxel map
  auto result = small_gicp::align(target_voxelmap.get_internal(),
                                  source.get_internal(), init_T, reg_setting);

  // Convert result back to our format
  RegistrationResult reg_result;
  Eigen::Matrix4d result_matrix = result.T_target_source.matrix();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      reg_result.transformation.matrix[i * 4 + j] = result_matrix(i, j);
    }
  }
  reg_result.converged = result.converged;
  reg_result.iterations = result.iterations;
  reg_result.error = result.error;

  return reg_result;
}

// UnsafeKdTree implementation
UnsafeKdTree::UnsafeKdTree(const PointCloud &cloud, int num_threads) {
  auto cloud_ptr =
      std::make_shared<small_gicp::PointCloud>(cloud.get_internal());
  if (num_threads > 1) {
    // Use OpenMP builder for parallel construction
    small_gicp::KdTreeBuilderOMP builder(num_threads);
    tree_ = std::make_shared<small_gicp::UnsafeKdTree<small_gicp::PointCloud>>(
        *cloud_ptr, builder);
  } else {
    // Use default sequential builder
    tree_ = std::make_shared<small_gicp::UnsafeKdTree<small_gicp::PointCloud>>(
        *cloud_ptr);
  }
}

size_t UnsafeKdTree::unsafe_nearest_neighbor(Point3d point) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  size_t index;
  double sq_distance;
  if (tree_->nearest_neighbor_search(query, &index, &sq_distance) == 1) {
    return index;
  }
  return SIZE_MAX; // Not found
}

rust::Vec<size_t> UnsafeKdTree::unsafe_knn_search(Point3d point,
                                                  size_t k) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  std::vector<size_t> indices(k);
  std::vector<double> sq_distances(k);

  size_t num_found =
      tree_->knn_search(query, k, indices.data(), sq_distances.data());

  rust::Vec<size_t> result;
  for (size_t i = 0; i < num_found; ++i) {
    result.push_back(indices[i]);
  }
  return result;
}

rust::Vec<size_t> UnsafeKdTree::unsafe_radius_search(Point3d point,
                                                     double radius) const {
  // Note: UnsafeKdTree doesn't have a built-in radius_search method in
  // small_gicp This is a placeholder implementation that returns empty results
  // In practice, radius search can be implemented using knn_search with
  // filtering
  rust::Vec<size_t> result;
  return result;
}

// Distance-returning unsafe search methods
NearestNeighborResult
UnsafeKdTree::unsafe_nearest_neighbor_with_distance(Point3d point) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  size_t index;
  double sq_distance;
  NearestNeighborResult result;

  if (tree_->nearest_neighbor_search(query, &index, &sq_distance) == 1) {
    result.index = index;
    result.squared_distance = sq_distance;
  } else {
    result.index = SIZE_MAX;
    result.squared_distance = std::numeric_limits<double>::infinity();
  }

  return result;
}

KnnSearchResult UnsafeKdTree::unsafe_knn_search_with_distances(Point3d point,
                                                               size_t k) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  std::vector<size_t> indices(k);
  std::vector<double> sq_distances(k);

  size_t num_found =
      tree_->knn_search(query, k, indices.data(), sq_distances.data());

  KnnSearchResult result;
  for (size_t i = 0; i < num_found; ++i) {
    result.indices.push_back(indices[i]);
    result.squared_distances.push_back(sq_distances[i]);
  }
  return result;
}

KnnSearchResult
UnsafeKdTree::unsafe_radius_search_with_distances(Point3d point,
                                                  double radius) const {
  // Note: This is a placeholder implementation using knn_search with filtering
  KnnSearchResult result;

  // Use a large k for approximate radius search
  const size_t max_k = 1000;
  auto knn_result = unsafe_knn_search_with_distances(point, max_k);

  double radius_sq = radius * radius;
  for (size_t i = 0; i < knn_result.indices.size(); ++i) {
    if (knn_result.squared_distances[i] <= radius_sq) {
      result.indices.push_back(knn_result.indices[i]);
      result.squared_distances.push_back(knn_result.squared_distances[i]);
    }
  }

  return result;
}

// IncrementalVoxelMap implementation
IncrementalVoxelMap::IncrementalVoxelMap(double voxel_size) {
  voxelmap_ = std::make_shared<
      small_gicp::IncrementalVoxelMap<small_gicp::GaussianVoxel>>(voxel_size);
}

void IncrementalVoxelMap::incremental_insert(const PointCloud &cloud) {
  voxelmap_->insert(cloud.get_internal());
}

size_t IncrementalVoxelMap::incremental_size() const {
  return voxelmap_->size();
}

void IncrementalVoxelMap::incremental_clear() {
  // Note: IncrementalVoxelMap doesn't have a public clear method
  // LRU-based cleanup is handled automatically
  // For a complete reset, a new instance should be created
}

void IncrementalVoxelMap::incremental_finalize() {
  // Note: IncrementalVoxelMap finalizes voxels automatically during insertion
  // This is a no-op placeholder for API compatibility
}

// Factory functions
std::unique_ptr<PointCloud> create_point_cloud() {
  return std::make_unique<PointCloud>();
}

std::unique_ptr<KdTree> create_kdtree(const PointCloud &cloud,
                                      int num_threads) {
  return std::make_unique<KdTree>(cloud, num_threads);
}

std::unique_ptr<UnsafeKdTree> create_unsafe_kdtree(const PointCloud &cloud,
                                                   int num_threads) {
  return std::make_unique<UnsafeKdTree>(cloud, num_threads);
}

std::unique_ptr<GaussianVoxelMap> create_voxelmap(double voxel_size) {
  return std::make_unique<GaussianVoxelMap>(voxel_size);
}

std::unique_ptr<IncrementalVoxelMap>
create_incremental_voxelmap(double voxel_size) {
  return std::make_unique<IncrementalVoxelMap>(voxel_size);
}

// Preprocessing function implementations
std::unique_ptr<PointCloud> downsample_voxelgrid(const PointCloud &cloud,
                                                 double voxel_size,
                                                 int num_threads) {
  // Use the existing method which has proper access
  return cloud.voxel_downsample(voxel_size, num_threads);
}

std::unique_ptr<PointCloud> downsample_random(const PointCloud &cloud,
                                              size_t num_samples) {
  auto result = std::make_unique<PointCloud>();

  // Use the public API instead of private members
  const auto &input_cloud = cloud.get_internal();
  if (num_samples >= input_cloud.size()) {
    // Return a copy by copying points one by one
    result->resize(input_cloud.size());
    for (size_t i = 0; i < input_cloud.size(); ++i) {
      const auto &point = input_cloud.point(i);
      result->set_point(i, Point3d{point.x(), point.y(), point.z()});
    }
    return result;
  }

  // Resize the result cloud
  result->resize(num_samples);

  // Simple random sampling without replacement
  std::vector<size_t> indices(input_cloud.size());
  std::iota(indices.begin(), indices.end(), 0);

  // Use a simple random shuffle
  std::random_device rd;
  std::mt19937 gen(rd());
  std::shuffle(indices.begin(), indices.end(), gen);

  // Copy selected points using public API
  for (size_t i = 0; i < num_samples; ++i) {
    const auto &point = input_cloud.point(indices[i]);
    result->set_point(i, Point3d{point.x(), point.y(), point.z()});
  }

  return result;
}

void compute_normals(PointCloud &cloud, int num_neighbors, int num_threads) {
  // Use the existing method which has proper access
  cloud.estimate_normals(num_neighbors, num_threads);
}

void compute_covariances(PointCloud &cloud, int num_neighbors,
                         int num_threads) {
  // Use the existing method which has proper access
  cloud.estimate_covariances(num_neighbors, num_threads);
}

// I/O function implementations
std::unique_ptr<PointCloud> load_ply(rust::Str filename) {
  // Convert rust::Str to std::string
  std::string filename_str(filename);

  // Create result first
  auto result = std::make_unique<PointCloud>();

  // Use small_gicp's PLY reader (returns empty vector on error)
  auto points = small_gicp::read_ply(filename_str);
  if (points.empty()) {
    // Return empty cloud on error
    return result;
  }

  // Use public methods to build the cloud
  result->resize(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    const auto &p = points[i];
    result->set_point(i, Point3d{p.x(), p.y(), p.z()});
  }

  return result;
}

void save_ply(rust::Str filename, const PointCloud &cloud) {
  // Convert rust::Str to std::string
  std::string filename_str(filename);

  // Binary PLY writer to match the reader
  std::ofstream ofs(filename_str, std::ios::binary);
  if (!ofs) {
    // Silently fail - cxx doesn't support exceptions across FFI
    std::cerr << "Failed to open file for writing: " << filename_str
              << std::endl;
    return;
  }

  const auto &pc = cloud.get_internal();
  ofs << "ply\n";
  ofs << "format binary_little_endian 1.0\n";
  ofs << "element vertex " << pc.size() << "\n";
  ofs << "property float x\n";
  ofs << "property float y\n";
  ofs << "property float z\n";
  ofs << "property float w\n"; // Dummy property to work around reader bug
  if (!pc.normals.empty()) {
    ofs << "property float nx\n";
    ofs << "property float ny\n";
    ofs << "property float nz\n";
  }
  ofs << "end_header\n";

  // Write binary data
  // Always write at least 4 floats per point (x, y, z, w) to match reader
  // expectations
  std::vector<float> buffer;
  size_t floats_per_vertex = pc.normals.empty() ? 4 : 7;
  buffer.reserve(pc.size() * floats_per_vertex);

  for (size_t i = 0; i < pc.size(); ++i) {
    const auto &p = pc.point(i);
    buffer.push_back(static_cast<float>(p.x()));
    buffer.push_back(static_cast<float>(p.y()));
    buffer.push_back(static_cast<float>(p.z()));
    buffer.push_back(1.0f); // w component

    if (!pc.normals.empty()) {
      const auto &n = pc.normal(i);
      buffer.push_back(static_cast<float>(n.x()));
      buffer.push_back(static_cast<float>(n.y()));
      buffer.push_back(static_cast<float>(n.z()));
    }
  }

  ofs.write(reinterpret_cast<const char *>(buffer.data()),
            buffer.size() * sizeof(float));

  if (!ofs.good()) {
    std::cerr << "Error writing to PLY file: " << filename_str << std::endl;
  }
}

} // namespace small_gicp_cxx
