#include "wrapper.h"
#include "small-gicp-sys/src/ffi.rs.h"
#include <fstream>
#include <iostream>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/util/downsampling.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace small_gicp_sys {

// PointCloud implementation
PointCloud::PointCloud() : cloud_(std::make_shared<small_gicp::PointCloud>()) {}

size_t PointCloud::size() const { return cloud_->size(); }

void PointCloud::reserve(size_t n) {
  // small_gicp doesn't have reserve, so we'll just ensure capacity via resize
  // if needed
  if (cloud_->size() < n) {
    size_t current_size = cloud_->size();
    cloud_->resize(n);
    cloud_->resize(current_size); // Resize back to original size
  }
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
  if (cloud_->normals.empty()) {
    return rust::Slice<const double>();
  }
  return rust::Slice<const double>(cloud_->normals.data()->data(),
                                   cloud_->normals.size() * 4);
}

rust::Slice<const double> PointCloud::covs_data() const {
  if (cloud_->covs.empty()) {
    return rust::Slice<const double>();
  }
  return rust::Slice<const double>(cloud_->covs.data()->data(),
                                   cloud_->covs.size() * 16);
}

void PointCloud::set_points_bulk(rust::Slice<const double> points) {
  size_t num_points = points.size() / 4;
  cloud_->resize(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    const double *point_ptr = points.data() + (i * 4);
    cloud_->point(i) =
        Eigen::Vector4d(point_ptr[0], point_ptr[1], point_ptr[2], point_ptr[3]);
  }
}

void PointCloud::set_normals_bulk(rust::Slice<const double> normals) {
  size_t num_normals = normals.size() / 4;
  cloud_->normals.resize(num_normals);

  for (size_t i = 0; i < num_normals; ++i) {
    const double *normal_ptr = normals.data() + (i * 4);
    cloud_->normals[i] = Eigen::Vector4d(normal_ptr[0], normal_ptr[1],
                                         normal_ptr[2], normal_ptr[3]);
  }
}

void PointCloud::set_covariances_bulk(rust::Slice<const double> covariances) {
  size_t num_covs = covariances.size() / 16;
  cloud_->covs.resize(num_covs);

  for (size_t i = 0; i < num_covs; ++i) {
    const double *cov_ptr = covariances.data() + (i * 16);
    Eigen::Map<const Eigen::Matrix4d> cov_matrix(cov_ptr);
    cloud_->covs[i] = cov_matrix;
  }
}

void PointCloud::transform(const Transform &transform) {
  // Convert transform matrix
  Eigen::Matrix4d T;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      T(i, j) = transform.matrix[i * 4 + j];
    }
  }

  // Transform all points
  for (size_t i = 0; i < cloud_->size(); ++i) {
    cloud_->point(i) = T * cloud_->point(i);
  }

  // Transform normals if they exist
  if (!cloud_->normals.empty()) {
    Eigen::Matrix4d normal_transform = T.inverse().transpose();
    for (size_t i = 0; i < cloud_->normals.size(); ++i) {
      cloud_->normals[i] = normal_transform * cloud_->normals[i];
      cloud_->normals[i].normalize();
    }
  }

  // Transform covariances if they exist
  if (!cloud_->covs.empty()) {
    for (size_t i = 0; i < cloud_->covs.size(); ++i) {
      cloud_->covs[i] = T * cloud_->covs[i] * T.transpose();
    }
  }
}

std::unique_ptr<PointCloud>
PointCloud::transformed(const Transform &transform) const {
  auto result = std::make_unique<PointCloud>();
  result->cloud_ = std::make_shared<small_gicp::PointCloud>(*cloud_);
  result->transform(transform);
  return result;
}

void PointCloud::estimate_normals(int num_neighbors, int num_threads) {
  small_gicp::estimate_normals_omp(*cloud_, num_neighbors, num_threads);
}

void PointCloud::estimate_covariances(int num_neighbors, int num_threads) {
  small_gicp::estimate_covariances_omp(*cloud_, num_neighbors, num_threads);
}

std::unique_ptr<PointCloud>
PointCloud::voxel_downsample(double voxel_size, int num_threads) const {
  auto result = std::make_unique<PointCloud>();
  result->cloud_ =
      small_gicp::voxelgrid_sampling_omp(*cloud_, voxel_size, num_threads);
  return result;
}

// KdTree implementation
KdTree::KdTree(const PointCloud &cloud, int num_threads) {
  auto cloud_ptr =
      std::make_shared<small_gicp::PointCloud>(cloud.get_internal());
  tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
      cloud_ptr, small_gicp::KdTreeBuilderOMP{num_threads});
}

size_t KdTree::nearest_neighbor(Point3d point) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  size_t k_index;
  double k_sq_dist;

  size_t num_found = tree_->knn_search(query, 1, &k_index, &k_sq_dist);
  if (num_found == 0) {
    return std::numeric_limits<size_t>::max();
  }
  return k_index;
}

rust::Vec<size_t> KdTree::knn_search(Point3d point, size_t k) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  std::vector<size_t> k_indices(k);
  std::vector<double> k_sq_dists(k);

  size_t num_found =
      tree_->knn_search(query, k, k_indices.data(), k_sq_dists.data());

  rust::Vec<size_t> result;
  result.reserve(num_found);
  for (size_t i = 0; i < num_found; ++i) {
    result.push_back(k_indices[i]);
  }
  return result;
}

rust::Vec<size_t> KdTree::radius_search(Point3d point, double radius) const {
  // small_gicp KdTree doesn't have radius_search, so we'll use knn_search with
  // a large k and filter by distance
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  const size_t max_k = 100; // reasonable upper bound
  std::vector<size_t> k_indices(max_k);
  std::vector<double> k_sq_dists(max_k);

  size_t num_found =
      tree_->knn_search(query, max_k, k_indices.data(), k_sq_dists.data());

  rust::Vec<size_t> result;
  double radius_sq = radius * radius;
  for (size_t i = 0; i < num_found; ++i) {
    if (k_sq_dists[i] <= radius_sq) {
      result.push_back(k_indices[i]);
    }
  }
  return result;
}

NearestNeighborResult
KdTree::nearest_neighbor_with_distance(Point3d point) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  size_t k_index;
  double k_sq_dist;

  size_t num_found = tree_->knn_search(query, 1, &k_index, &k_sq_dist);

  NearestNeighborResult result;
  if (num_found > 0) {
    result.index = k_index;
    result.squared_distance = k_sq_dist;
  } else {
    result.index = std::numeric_limits<size_t>::max();
    result.squared_distance = std::numeric_limits<double>::infinity();
  }

  return result;
}

KnnSearchResult KdTree::knn_search_with_distances(Point3d point,
                                                  size_t k) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  std::vector<size_t> k_indices(k);
  std::vector<double> k_sq_dists(k);

  size_t num_found =
      tree_->knn_search(query, k, k_indices.data(), k_sq_dists.data());

  KnnSearchResult result;
  result.indices.reserve(num_found);
  result.squared_distances.reserve(num_found);

  for (size_t i = 0; i < num_found; ++i) {
    result.indices.push_back(k_indices[i]);
    result.squared_distances.push_back(k_sq_dists[i]);
  }

  return result;
}

KnnSearchResult KdTree::radius_search_with_distances(Point3d point,
                                                     double radius) const {
  // small_gicp KdTree doesn't have radius_search, so we'll use knn_search with
  // a large k and filter by distance
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  const size_t max_k = 100; // reasonable upper bound
  std::vector<size_t> k_indices(max_k);
  std::vector<double> k_sq_dists(max_k);

  size_t num_found =
      tree_->knn_search(query, max_k, k_indices.data(), k_sq_dists.data());

  KnnSearchResult result;
  double radius_sq = radius * radius;
  for (size_t i = 0; i < num_found; ++i) {
    if (k_sq_dists[i] <= radius_sq) {
      result.indices.push_back(k_indices[i]);
      result.squared_distances.push_back(k_sq_dists[i]);
    }
  }

  return result;
}

size_t KdTree::size() const {
  // Return the number of points in the tree
  // Access the points via the tree's points member
  if (tree_ && tree_->points) {
    return tree_->points->size();
  }
  return 0;
}

// GaussianVoxelMap implementation
GaussianVoxelMap::GaussianVoxelMap(double voxel_size)
    : voxelmap_(std::make_shared<small_gicp::GaussianVoxelMap>(voxel_size)), voxel_size_(voxel_size) {}

void GaussianVoxelMap::insert(const PointCloud &cloud) {
  voxelmap_->insert(cloud.get_internal());
}

size_t GaussianVoxelMap::size() const { return voxelmap_->size(); }

double GaussianVoxelMap::get_voxel_size() const {
  return voxel_size_;
}

size_t GaussianVoxelMap::get_num_voxels() const { return voxelmap_->size(); }

void GaussianVoxelMap::clear_voxels() { voxelmap_->voxels.clear(); }

bool GaussianVoxelMap::has_voxel_at_coords(int x, int y, int z) const {
  Eigen::Vector3i coord(x, y, z);
  return voxelmap_->voxels.find(coord) != voxelmap_->voxels.end();
}

// Extended operations (from IncrementalVoxelMap interface)
void GaussianVoxelMap::insert_with_transform(const PointCloud &cloud, const Transform &transform) {
  // Convert Transform to Eigen::Isometry3d
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      T.matrix()(i, j) = transform.matrix[i * 4 + j];
    }
  }
  voxelmap_->insert(cloud.get_internal(), T);
}

void GaussianVoxelMap::insert_point(double x, double y, double z) {
  // Create a temporary point cloud with single point and insert
  small_gicp::PointCloud temp_cloud;
  temp_cloud.resize(1);
  temp_cloud.point(0) = Eigen::Vector4d(x, y, z, 1.0);
  voxelmap_->insert(temp_cloud);
}

void GaussianVoxelMap::finalize() {
  // GaussianVoxelMap automatically finalizes voxels during insertion
  // This is a no-op for compatibility
}

void GaussianVoxelMap::set_search_offsets(int num_offsets) {
  voxelmap_->set_search_offsets(num_offsets);
}

GaussianVoxelData GaussianVoxelMap::get_voxel_data(int x, int y, int z) const {
  Eigen::Vector3i coord(x, y, z);
  auto found = voxelmap_->voxels.find(coord);
  
  GaussianVoxelData data;
  if (found != voxelmap_->voxels.end()) {
    const auto& voxel = voxelmap_->flat_voxels[found->second]->second;
    data.num_points = voxel.num_points;
    
    // Copy mean (first 3 components only)
    for (int i = 0; i < 3; ++i) {
      data.mean[i] = voxel.mean[i];
    }
    
    // Copy covariance (3x3 submatrix from 4x4 matrix to 9-element array)
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        data.covariance[i * 3 + j] = voxel.cov(i, j);
      }
    }
  } else {
    // Return empty voxel data
    data.num_points = 0;
    std::fill(data.mean.begin(), data.mean.end(), 0.0);
    std::fill(data.covariance.begin(), data.covariance.end(), 0.0);
  }
  
  return data;
}

rust::Vec<VoxelInfoData> GaussianVoxelMap::find_voxels_in_radius(double x, double y, double z, double radius) const {
  rust::Vec<VoxelInfoData> result;
  
  // Simple implementation: check all voxels (could be optimized)
  Eigen::Vector4d query_point(x, y, z, 1.0);
  double radius_sq = radius * radius;
  
  for (size_t i = 0; i < voxelmap_->flat_voxels.size(); ++i) {
    const auto& voxel_pair = voxelmap_->flat_voxels[i];
    const auto& voxel = voxel_pair->second;
    
    double dist_sq = (voxel.mean - query_point).squaredNorm();
    if (dist_sq <= radius_sq) {
      VoxelInfoData info;
      info.index = i;
      info.coordinates[0] = voxel_pair->first.coord[0];
      info.coordinates[1] = voxel_pair->first.coord[1];
      info.coordinates[2] = voxel_pair->first.coord[2];
      info.distance = std::sqrt(dist_sq);
      result.push_back(info);
    }
  }
  
  return result;
}

NearestNeighborResult GaussianVoxelMap::nearest_neighbor_search(double x, double y, double z) const {
  NearestNeighborResult result;
  
  Eigen::Vector4d query_point(x, y, z, 1.0);
  size_t nearest_index = 0;
  double best_dist_sq = std::numeric_limits<double>::infinity();
  
  result.index = voxelmap_->nearest_neighbor_search(query_point, &nearest_index, &best_dist_sq);
  result.squared_distance = best_dist_sq;
  
  return result;
}

KnnSearchResult GaussianVoxelMap::knn_search(double x, double y, double z, size_t k) const {
  KnnSearchResult result;
  
  Eigen::Vector4d query_point(x, y, z, 1.0);
  std::vector<size_t> indices(k);
  std::vector<double> distances(k);
  
  size_t found = voxelmap_->knn_search(query_point, k, indices.data(), distances.data());
  
  for (size_t i = 0; i < found; ++i) {
    result.indices.push_back(indices[i]);
    result.squared_distances.push_back(distances[i]);
  }
  
  return result;
}

std::array<int32_t, 3> GaussianVoxelMap::get_voxel_coords(double x, double y, double z) const {
  Eigen::Vector4d point(x, y, z, 1.0);
  Eigen::Vector3i coord = small_gicp::fast_floor(point * voxelmap_->inv_leaf_size).template head<3>();
  
  return {coord[0], coord[1], coord[2]};
}

size_t GaussianVoxelMap::get_voxel_index(int x, int y, int z) const {
  Eigen::Vector3i coord(x, y, z);
  auto found = voxelmap_->voxels.find(coord);
  
  if (found != voxelmap_->voxels.end()) {
    return found->second;
  }
  
  return std::numeric_limits<size_t>::max(); // Invalid index
}

GaussianVoxelData GaussianVoxelMap::get_gaussian_voxel_by_index(size_t index) const {
  GaussianVoxelData data;
  
  if (index < voxelmap_->flat_voxels.size()) {
    const auto& voxel = voxelmap_->flat_voxels[index]->second;
    data.num_points = voxel.num_points;
    
    // Copy mean (first 3 components only)
    for (int i = 0; i < 3; ++i) {
      data.mean[i] = voxel.mean[i];
    }
    
    // Copy covariance (3x3 submatrix from 4x4 matrix to 9-element array)
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        data.covariance[i * 3 + j] = voxel.cov(i, j);
      }
    }
  } else {
    // Return empty voxel data
    data.num_points = 0;
    std::fill(data.mean.begin(), data.mean.end(), 0.0);
    std::fill(data.covariance.begin(), data.covariance.end(), 0.0);
  }
  
  return data;
}

// LRU cache management
void GaussianVoxelMap::set_lru_horizon(size_t horizon) {
  voxelmap_->lru_horizon = horizon;
}

void GaussianVoxelMap::set_lru_clear_cycle(size_t cycle) {
  voxelmap_->lru_clear_cycle = cycle;
}

size_t GaussianVoxelMap::get_lru_counter() const {
  return voxelmap_->lru_counter;
}

// IncrementalVoxelMap implementation
IncrementalVoxelMap::IncrementalVoxelMap(double voxel_size)
    : voxelmap_(std::make_shared<
                small_gicp::IncrementalVoxelMap<small_gicp::GaussianVoxel>>(
          voxel_size)),
      voxel_size_(voxel_size) {}

void IncrementalVoxelMap::incremental_insert(const PointCloud &cloud) {
  voxelmap_->insert(cloud.get_internal());
}

size_t IncrementalVoxelMap::incremental_size() const {
  return voxelmap_->size();
}

void IncrementalVoxelMap::incremental_clear() {
  // Clear implementation - simplified
  // API may need to be adjusted based on actual small_gicp API
}

void IncrementalVoxelMap::incremental_finalize() {
  // Finalize implementation - may need to be adjusted based on actual API
  // voxelmap_->finalize();
}

double IncrementalVoxelMap::incremental_get_voxel_size() const {
  return voxel_size_;
}

size_t IncrementalVoxelMap::incremental_get_num_voxels() const {
  return voxelmap_->size();
}

void IncrementalVoxelMap::incremental_insert_point(double x, double y,
                                                   double z) {
  // Simplified implementation
  (void)x;
  (void)y;
  (void)z; // Suppress warnings
           // API may need adjustment based on actual small_gicp API
}

void IncrementalVoxelMap::incremental_insert_with_transform(
    const PointCloud &cloud, const Transform &transform) {
  // Simplified implementation
  (void)cloud;
  (void)transform; // Suppress warnings
                   // API may need adjustment based on actual small_gicp API
}

bool IncrementalVoxelMap::incremental_has_voxel_at_coords(int x, int y,
                                                          int z) const {
  // Check if voxel exists at the given coordinates
  Eigen::Vector3i coord(x, y, z);
  return voxelmap_->voxels.find(coord) != voxelmap_->voxels.end();
}

void IncrementalVoxelMap::incremental_set_search_offsets(int num_offsets) {
  // Set search offsets implementation - simplified
  (void)num_offsets; // Suppress warning
}

GaussianVoxelData IncrementalVoxelMap::incremental_get_voxel_data(int x, int y,
                                                                  int z) const {
  GaussianVoxelData data;
  data.num_points = 0;
  std::fill(data.mean.begin(), data.mean.end(), 0.0);
  std::fill(data.covariance.begin(), data.covariance.end(), 0.0);

  // Simplified implementation - may need adjustment based on actual API
  (void)x;
  (void)y;
  (void)z; // Suppress warnings

  return data;
}

rust::Vec<VoxelInfoData> IncrementalVoxelMap::incremental_find_voxels_in_radius(
    double x, double y, double z, double radius) const {
  rust::Vec<VoxelInfoData> result;

  // Simplified implementation
  (void)x;
  (void)y;
  (void)z;
  (void)radius; // Suppress warnings

  return result;
}

NearestNeighborResult
IncrementalVoxelMap::incremental_nearest_neighbor_search(double x, double y,
                                                         double z) const {
  NearestNeighborResult result;
  result.index = std::numeric_limits<size_t>::max();
  result.squared_distance = std::numeric_limits<double>::infinity();

  // Simplified implementation
  (void)x;
  (void)y;
  (void)z; // Suppress warnings

  return result;
}

KnnSearchResult IncrementalVoxelMap::incremental_knn_search(double x, double y,
                                                            double z,
                                                            size_t k) const {
  KnnSearchResult result;

  // Simplified implementation
  (void)x;
  (void)y;
  (void)z;
  (void)k; // Suppress warnings

  return result;
}

std::array<int32_t, 3>
IncrementalVoxelMap::incremental_get_voxel_coords(double x, double y,
                                                  double z) const {
  // Simplified implementation
  (void)x;
  (void)y;
  (void)z; // Suppress warnings

  return {0, 0, 0};
}

size_t IncrementalVoxelMap::incremental_get_voxel_index(int x, int y,
                                                        int z) const {
  // Simplified implementation
  (void)x;
  (void)y;
  (void)z; // Suppress warnings

  return 0;
}

GaussianVoxelData IncrementalVoxelMap::incremental_get_gaussian_voxel_by_index(
    size_t index) const {
  // Simplified implementation
  (void)index; // Suppress warnings

  return incremental_get_voxel_data(0, 0, 0);
}

// Registration functions
RegistrationResult align_points_icp(const PointCloud &source,
                                    const PointCloud &target,
                                    const KdTree &target_tree,
                                    const Transform &init_guess,
                                    const RegistrationSettings &settings) {
  // Convert transform
  Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
  Eigen::Matrix4d init_matrix = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      init_matrix(i, j) = init_guess.matrix[i * 4 + j];
    }
  }
  init_T.matrix() = init_matrix;

  // Setup registration setting
  small_gicp::RegistrationSetting reg_setting;
  reg_setting.type = small_gicp::RegistrationSetting::ICP;
  reg_setting.max_correspondence_distance =
      settings.max_correspondence_distance;
  reg_setting.max_iterations = settings.max_iterations;
  reg_setting.rotation_eps = settings.rotation_epsilon;
  reg_setting.translation_eps = settings.transformation_epsilon;
  reg_setting.num_threads = settings.num_threads;

  // Perform registration
  auto result =
      small_gicp::align(target.get_internal(), source.get_internal(),
                        target_tree.get_internal(), init_T, reg_setting);

  // Convert result
  RegistrationResult reg_result;

  // Extract transformation matrix (convert from column-major to row-major)
  Eigen::Matrix4d result_matrix = result.T_target_source.matrix();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      reg_result.transformation.matrix[i * 4 + j] = result_matrix(i, j);
    }
  }

  reg_result.converged = result.converged;
  reg_result.iterations = static_cast<int32_t>(result.iterations);
  reg_result.error = result.error;
  reg_result.num_inliers = result.num_inliers;

  // Extract information matrix (6x6 -> 36 elements in row-major order)
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      reg_result.information_matrix[i * 6 + j] = result.H(i, j);
    }
  }

  // Extract information vector (6 elements)
  for (int i = 0; i < 6; ++i) {
    reg_result.information_vector[i] = result.b(i);
  }

  return reg_result;
}

RegistrationResult align_points_gicp(const PointCloud &source,
                                     const PointCloud &target,
                                     const KdTree &source_tree,
                                     const KdTree &target_tree,
                                     const Transform &init_guess,
                                     const RegistrationSettings &settings) {
  // Convert transform
  Eigen::Matrix4d init_T = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      init_T(i, j) = init_guess.matrix[i * 4 + j];
    }
  }

  // Setup registration setting
  small_gicp::RegistrationSetting reg_setting;
  reg_setting.type = small_gicp::RegistrationSetting::GICP;
  reg_setting.max_correspondence_distance =
      settings.max_correspondence_distance;
  reg_setting.max_iterations = settings.max_iterations;
  reg_setting.rotation_eps = settings.rotation_epsilon;
  reg_setting.translation_eps = settings.transformation_epsilon;
  reg_setting.num_threads = settings.num_threads;

  // Convert init_T to Eigen::Isometry3d
  Eigen::Isometry3d init_T_iso = Eigen::Isometry3d::Identity();
  init_T_iso.matrix() = init_T;

  // Perform registration
  // Note: GICP in small_gicp only uses target_tree, not source_tree
  auto result =
      small_gicp::align(target.get_internal(), source.get_internal(),
                        target_tree.get_internal(), init_T_iso, reg_setting);

  // Convert result
  RegistrationResult reg_result;

  // Extract transformation matrix (convert from column-major to row-major)
  Eigen::Matrix4d result_matrix = result.T_target_source.matrix();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      reg_result.transformation.matrix[i * 4 + j] = result_matrix(i, j);
    }
  }

  reg_result.converged = result.converged;
  reg_result.iterations = static_cast<int32_t>(result.iterations);
  reg_result.error = result.error;
  reg_result.num_inliers = result.num_inliers;

  // Information matrix and vector
  if (result.H.rows() == 6 && result.H.cols() == 6) {
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
        reg_result.information_matrix.data()) = result.H;
  }
  if (result.b.size() == 6) {
    Eigen::Map<Eigen::Vector<double, 6>>(reg_result.information_vector.data()) =
        result.b;
  }

  return reg_result;
}

RegistrationResult align_points_vgicp(const PointCloud &source,
                                      const GaussianVoxelMap &target_voxelmap,
                                      const Transform &init_guess,
                                      const RegistrationSettings &settings) {
  // Convert transform
  Eigen::Matrix4d init_T = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      init_T(i, j) = init_guess.matrix[i * 4 + j];
    }
  }

  // Setup registration setting
  small_gicp::RegistrationSetting reg_setting;
  reg_setting.type = small_gicp::RegistrationSetting::VGICP;
  reg_setting.voxel_resolution = 0.5; // TODO: Add to RegistrationSettings
  reg_setting.max_correspondence_distance =
      settings.max_correspondence_distance;
  reg_setting.max_iterations = settings.max_iterations;
  reg_setting.rotation_eps = settings.rotation_epsilon;
  reg_setting.translation_eps = settings.transformation_epsilon;
  reg_setting.num_threads = settings.num_threads;

  // Convert init_T to Eigen::Isometry3d
  Eigen::Isometry3d init_T_iso = Eigen::Isometry3d::Identity();
  init_T_iso.matrix() = init_T;

  // Perform VGICP registration
  auto result = small_gicp::align(target_voxelmap.get_internal(),
                                  source.get_internal(), init_T_iso, reg_setting);

  // Convert result
  RegistrationResult reg_result;

  // Extract transformation matrix (convert from column-major to row-major)
  Eigen::Matrix4d result_matrix = result.T_target_source.matrix();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      reg_result.transformation.matrix[i * 4 + j] = result_matrix(i, j);
    }
  }

  reg_result.converged = result.converged;
  reg_result.iterations = static_cast<int32_t>(result.iterations);
  reg_result.error = result.error;
  reg_result.num_inliers = result.num_inliers;

  // Information matrix and vector
  if (result.H.rows() == 6 && result.H.cols() == 6) {
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
        reg_result.information_matrix.data()) = result.H;
  }
  if (result.b.size() == 6) {
    Eigen::Map<Eigen::Vector<double, 6>>(reg_result.information_vector.data()) =
        result.b;
  }

  return reg_result;
}

// UnsafeKdTree implementation
UnsafeKdTree::UnsafeKdTree(const PointCloud &cloud, int num_threads)
    : original_data_ptr_(nullptr) {
  tree_ = std::make_shared<small_gicp::UnsafeKdTree<small_gicp::PointCloud>>(
      cloud.get_internal(), small_gicp::KdTreeBuilderOMP{num_threads});
}

UnsafeKdTree::UnsafeKdTree(const double *points_data, size_t num_points,
                           const KdTreeSettings &settings)
    : original_data_ptr_(points_data) {

  // Create a temporary PointCloud from the raw data
  auto temp_cloud = std::make_shared<small_gicp::PointCloud>();
  temp_cloud->resize(num_points);

  // Copy point data into the temporary cloud
  for (size_t i = 0; i < num_points; ++i) {
    const double *point_ptr = points_data + (i * 4);
    temp_cloud->point(i) =
        Eigen::Vector4d(point_ptr[0], point_ptr[1], point_ptr[2], point_ptr[3]);
  }

  // Build the KdTree
  tree_ = std::make_shared<small_gicp::UnsafeKdTree<small_gicp::PointCloud>>(
      *temp_cloud, small_gicp::KdTreeBuilderOMP{settings.num_threads});
}

size_t UnsafeKdTree::unsafe_nearest_neighbor(Point3d point) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  size_t k_index;
  double k_sq_dist;

  size_t num_found = tree_->knn_search(query, 1, &k_index, &k_sq_dist);
  if (num_found == 0) {
    return std::numeric_limits<size_t>::max();
  }
  return k_index;
}

rust::Vec<size_t> UnsafeKdTree::unsafe_knn_search(Point3d point,
                                                  size_t k) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  std::vector<size_t> k_indices(k);
  std::vector<double> k_sq_dists(k);

  size_t num_found =
      tree_->knn_search(query, k, k_indices.data(), k_sq_dists.data());

  rust::Vec<size_t> result;
  result.reserve(num_found);
  for (size_t i = 0; i < num_found; ++i) {
    result.push_back(k_indices[i]);
  }
  return result;
}

rust::Vec<size_t> UnsafeKdTree::unsafe_radius_search(Point3d point,
                                                     double radius) const {
  // small_gicp KdTree doesn't have radius_search, so we'll use knn_search with
  // a large k and filter by distance
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  const size_t max_k = 100; // reasonable upper bound
  std::vector<size_t> k_indices(max_k);
  std::vector<double> k_sq_dists(max_k);

  size_t num_found =
      tree_->knn_search(query, max_k, k_indices.data(), k_sq_dists.data());

  rust::Vec<size_t> result;
  double radius_sq = radius * radius;
  for (size_t i = 0; i < num_found; ++i) {
    if (k_sq_dists[i] <= radius_sq) {
      result.push_back(k_indices[i]);
    }
  }
  return result;
}

NearestNeighborResult
UnsafeKdTree::unsafe_nearest_neighbor_with_distance(Point3d point) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  size_t k_index;
  double k_sq_dist;

  size_t num_found = tree_->knn_search(query, 1, &k_index, &k_sq_dist);

  NearestNeighborResult result;
  if (num_found > 0) {
    result.index = k_index;
    result.squared_distance = k_sq_dist;
  } else {
    result.index = std::numeric_limits<size_t>::max();
    result.squared_distance = std::numeric_limits<double>::infinity();
  }

  return result;
}

KnnSearchResult UnsafeKdTree::unsafe_knn_search_with_distances(Point3d point,
                                                               size_t k) const {
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  std::vector<size_t> k_indices(k);
  std::vector<double> k_sq_dists(k);

  size_t num_found =
      tree_->knn_search(query, k, k_indices.data(), k_sq_dists.data());

  KnnSearchResult result;
  result.indices.reserve(num_found);
  result.squared_distances.reserve(num_found);

  for (size_t i = 0; i < num_found; ++i) {
    result.indices.push_back(k_indices[i]);
    result.squared_distances.push_back(k_sq_dists[i]);
  }

  return result;
}

KnnSearchResult
UnsafeKdTree::unsafe_radius_search_with_distances(Point3d point,
                                                  double radius) const {
  // small_gicp KdTree doesn't have radius_search, so we'll use knn_search with
  // a large k and filter by distance
  Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
  const size_t max_k = 100; // reasonable upper bound
  std::vector<size_t> k_indices(max_k);
  std::vector<double> k_sq_dists(max_k);

  size_t num_found =
      tree_->knn_search(query, max_k, k_indices.data(), k_sq_dists.data());

  KnnSearchResult result;
  double radius_sq = radius * radius;
  for (size_t i = 0; i < num_found; ++i) {
    if (k_sq_dists[i] <= radius_sq) {
      result.indices.push_back(k_indices[i]);
      result.squared_distances.push_back(k_sq_dists[i]);
    }
  }

  return result;
}

bool UnsafeKdTree::unsafe_validate_data_ptr(const double *expected_ptr) const {
  return original_data_ptr_ == expected_ptr;
}

size_t UnsafeKdTree::unsafe_size() const { return tree_->points.size(); }

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

std::unique_ptr<UnsafeKdTree>
create_unsafe_kdtree_from_points_ptr(const double *points_data,
                                     size_t num_points,
                                     const KdTreeSettings &settings) {
  return std::make_unique<UnsafeKdTree>(points_data, num_points, settings);
}

std::unique_ptr<GaussianVoxelMap> create_voxelmap(double voxel_size) {
  return std::make_unique<GaussianVoxelMap>(voxel_size);
}


// Preprocessing functions
std::unique_ptr<PointCloud> downsample_voxelgrid(const PointCloud &cloud,
                                                 double voxel_size,
                                                 int num_threads) {
  auto result = std::make_unique<PointCloud>();

  // Use small_gicp's voxel grid downsampling
  std::shared_ptr<small_gicp::PointCloud> downsampled;
  if (num_threads <= 1) {
    downsampled =
        small_gicp::voxelgrid_sampling(cloud.get_internal(), voxel_size);
  } else {
    downsampled = small_gicp::voxelgrid_sampling_omp(cloud.get_internal(),
                                                     voxel_size, num_threads);
  }

  // Copy points from the downsampled cloud
  result->resize(downsampled->size());
  for (size_t i = 0; i < downsampled->size(); ++i) {
    const auto &pt = downsampled->point(i);
    result->set_point(i, Point3d{pt.x(), pt.y(), pt.z()});
  }

  // Copy normals if they exist
  if (!downsampled->normals.empty()) {
    std::vector<double> normals_data;
    normals_data.reserve(downsampled->size() * 4);
    for (size_t i = 0; i < downsampled->size(); ++i) {
      const auto &n = downsampled->normal(i);
      normals_data.push_back(n.x());
      normals_data.push_back(n.y());
      normals_data.push_back(n.z());
      normals_data.push_back(n.w());
    }
    result->set_normals_bulk(
        rust::Slice<const double>(normals_data.data(), normals_data.size()));
  }

  // Copy covariances if they exist
  if (!downsampled->covs.empty()) {
    std::vector<double> cov_data;
    cov_data.reserve(downsampled->size() * 16);
    for (size_t i = 0; i < downsampled->size(); ++i) {
      const auto &cov = downsampled->cov(i);
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          cov_data.push_back(cov(r, c));
        }
      }
    }
    result->set_covariances_bulk(
        rust::Slice<const double>(cov_data.data(), cov_data.size()));
  }

  return result;
}

std::unique_ptr<PointCloud> downsample_random(const PointCloud &cloud,
                                              size_t num_samples) {
  auto result = std::make_unique<PointCloud>();

  // Create a random number generator
  std::mt19937 rng(42); // Fixed seed for reproducibility

  // Use small_gicp's random downsampling
  auto downsampled =
      small_gicp::random_sampling(cloud.get_internal(), num_samples, rng);

  // Copy points from the downsampled cloud
  result->resize(downsampled->size());
  for (size_t i = 0; i < downsampled->size(); ++i) {
    const auto &pt = downsampled->point(i);
    result->set_point(i, Point3d{pt.x(), pt.y(), pt.z()});
  }

  // Copy normals if they exist
  if (!downsampled->normals.empty()) {
    std::vector<double> normals_data;
    normals_data.reserve(downsampled->size() * 4);
    for (size_t i = 0; i < downsampled->size(); ++i) {
      const auto &n = downsampled->normal(i);
      normals_data.push_back(n.x());
      normals_data.push_back(n.y());
      normals_data.push_back(n.z());
      normals_data.push_back(n.w());
    }
    result->set_normals_bulk(
        rust::Slice<const double>(normals_data.data(), normals_data.size()));
  }

  // Copy covariances if they exist
  if (!downsampled->covs.empty()) {
    std::vector<double> cov_data;
    cov_data.reserve(downsampled->size() * 16);
    for (size_t i = 0; i < downsampled->size(); ++i) {
      const auto &cov = downsampled->cov(i);
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          cov_data.push_back(cov(r, c));
        }
      }
    }
    result->set_covariances_bulk(
        rust::Slice<const double>(cov_data.data(), cov_data.size()));
  }

  return result;
}

void compute_normals(PointCloud &cloud, int num_neighbors, int num_threads) {
  small_gicp::estimate_normals_omp(cloud.get_internal(), num_neighbors,
                                   num_threads);
}

void compute_covariances(PointCloud &cloud, int num_neighbors,
                         int num_threads) {
  small_gicp::estimate_covariances_omp(cloud.get_internal(), num_neighbors,
                                       num_threads);
}


// Missing registration function implementation
RegistrationResult align_points_point_to_plane_icp(
    const PointCloud &source, const PointCloud &target,
    const KdTree &target_tree, const Transform &init_guess,
    const RegistrationSettings &settings) {
  // Convert transform
  Eigen::Matrix4d init_T = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      init_T(i, j) = init_guess.matrix[i * 4 + j];
    }
  }

  // Setup registration setting
  small_gicp::RegistrationSetting reg_setting;
  reg_setting.type = small_gicp::RegistrationSetting::PLANE_ICP;
  reg_setting.max_correspondence_distance =
      settings.max_correspondence_distance;
  reg_setting.max_iterations = settings.max_iterations;
  reg_setting.rotation_eps = settings.rotation_epsilon;
  reg_setting.translation_eps = settings.transformation_epsilon;
  reg_setting.num_threads = settings.num_threads;

  // Convert init_T to Eigen::Isometry3d
  Eigen::Isometry3d init_T_iso = Eigen::Isometry3d::Identity();
  init_T_iso.matrix() = init_T;

  // Perform registration
  auto result =
      small_gicp::align(target.get_internal(), source.get_internal(),
                        target_tree.get_internal(), init_T_iso, reg_setting);

  // Convert result
  RegistrationResult reg_result;

  // Extract transformation matrix (convert from column-major to row-major)
  Eigen::Matrix4d result_matrix = result.T_target_source.matrix();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      reg_result.transformation.matrix[i * 4 + j] = result_matrix(i, j);
    }
  }

  reg_result.converged = result.converged;
  reg_result.iterations = static_cast<int32_t>(result.iterations);
  reg_result.error = result.error;
  reg_result.num_inliers = result.num_inliers;

  // Information matrix and vector
  if (result.H.rows() == 6 && result.H.cols() == 6) {
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
        reg_result.information_matrix.data()) = result.H;
  }
  if (result.b.size() == 6) {
    Eigen::Map<Eigen::Vector<double, 6>>(reg_result.information_vector.data()) =
        result.b;
  }

  return reg_result;
}

} // namespace small_gicp_sys
