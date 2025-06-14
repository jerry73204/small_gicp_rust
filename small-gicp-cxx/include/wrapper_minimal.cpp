#include "small-gicp-cxx/src/ffi.rs.h"
#include "wrapper.h"

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
  return rust::Slice<const double>(); // Simplified for now
}

rust::Slice<const double> PointCloud::covs_data() const {
  return rust::Slice<const double>(); // Simplified for now
}

void PointCloud::estimate_normals(int num_neighbors, int num_threads) {
  // No-op for now
}

void PointCloud::estimate_covariances(int num_neighbors, int num_threads) {
  // No-op for now
}

std::unique_ptr<PointCloud>
PointCloud::voxel_downsample(double voxel_size, int num_threads) const {
  // Return copy for now
  auto result = std::make_unique<PointCloud>();
  result->cloud_ = std::make_shared<small_gicp::PointCloud>(*cloud_);
  return result;
}

// Minimal KdTree - no actual functionality for now
KdTree::KdTree(const PointCloud &cloud, int num_threads) {
  // Create a minimal kdtree without OMP
  auto cloud_ptr =
      std::make_shared<small_gicp::PointCloud>(cloud.get_internal());
  tree_ =
      std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(cloud_ptr);
}

size_t KdTree::nearest_neighbor(Point3d point) const {
  return 0; // Placeholder
}

rust::Vec<size_t> KdTree::knn_search(Point3d point, size_t k) const {
  rust::Vec<size_t> result;
  for (size_t i = 0; i < std::min(k, size_t(10)); ++i) {
    result.push_back(i);
  }
  return result;
}

rust::Vec<size_t> KdTree::radius_search(Point3d point, double radius) const {
  rust::Vec<size_t> result;
  return result;
}

// Minimal GaussianVoxelMap
GaussianVoxelMap::GaussianVoxelMap(double voxel_size)
    : voxelmap_(std::make_shared<small_gicp::GaussianVoxelMap>(voxel_size)) {}

void GaussianVoxelMap::insert(const PointCloud &cloud) {
  voxelmap_->insert(cloud.get_internal());
}

size_t GaussianVoxelMap::size() const { return voxelmap_->size(); }

// Simple registration functions - return identity for now
RegistrationResult align_points_icp(const PointCloud &source,
                                    const PointCloud &target,
                                    const KdTree &target_tree,
                                    const Transform &init_guess,
                                    const RegistrationSettings &settings) {
  RegistrationResult reg_result;
  reg_result.transformation = init_guess;
  reg_result.converged = true;
  reg_result.iterations = 1;
  reg_result.error = 0.0;
  return reg_result;
}

RegistrationResult align_points_gicp(const PointCloud &source,
                                     const PointCloud &target,
                                     const KdTree &source_tree,
                                     const KdTree &target_tree,
                                     const Transform &init_guess,
                                     const RegistrationSettings &settings) {
  RegistrationResult reg_result;
  reg_result.transformation = init_guess;
  reg_result.converged = true;
  reg_result.iterations = 1;
  reg_result.error = 0.0;
  return reg_result;
}

RegistrationResult align_points_vgicp(const PointCloud &source,
                                      const GaussianVoxelMap &target_voxelmap,
                                      const Transform &init_guess,
                                      const RegistrationSettings &settings) {
  RegistrationResult reg_result;
  reg_result.transformation = init_guess;
  reg_result.converged = true;
  reg_result.iterations = 1;
  reg_result.error = 0.0;
  return reg_result;
}

// Factory functions
std::unique_ptr<PointCloud> create_point_cloud() {
  return std::make_unique<PointCloud>();
}

std::unique_ptr<KdTree> create_kdtree(const PointCloud &cloud,
                                      int num_threads) {
  return std::make_unique<KdTree>(cloud, num_threads);
}

std::unique_ptr<GaussianVoxelMap> create_voxelmap(double voxel_size) {
  return std::make_unique<GaussianVoxelMap>(voxel_size);
}

} // namespace small_gicp_cxx
