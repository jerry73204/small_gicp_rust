#include "small-gicp-cxx/src/ffi.rs.h"
#include "wrapper.h"
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>

namespace small_gicp_cxx {

// PointCloud implementation
PointCloud::PointCloud() : cloud_(std::make_shared<small_gicp::PointCloud>()) {}

size_t PointCloud::size() const {
    return cloud_->size();
}

void PointCloud::reserve(size_t n) {
    // small_gicp doesn't have reserve, so we'll just ensure capacity via resize if needed
    if (cloud_->size() < n) {
        size_t current_size = cloud_->size();
        cloud_->resize(n);
        cloud_->resize(current_size); // Resize back to original size
    }
}

void PointCloud::resize(size_t n) {
    cloud_->resize(n);
}

void PointCloud::clear() {
    cloud_->resize(0);
}

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
    const auto& p = cloud_->point(index);
    return Point3d{p.x(), p.y(), p.z()};
}

rust::Slice<const double> PointCloud::points_data() const {
    if (cloud_->empty()) {
        return rust::Slice<const double>();
    }
    return rust::Slice<const double>(
        cloud_->points.data()->data(), 
        cloud_->points.size() * 4
    );
}

rust::Slice<const double> PointCloud::normals_data() const {
    if (cloud_->normals.empty()) {
        return rust::Slice<const double>();
    }
    return rust::Slice<const double>(
        cloud_->normals.data()->data(), 
        cloud_->normals.size() * 4
    );
}

rust::Slice<const double> PointCloud::covs_data() const {
    if (cloud_->covs.empty()) {
        return rust::Slice<const double>();
    }
    return rust::Slice<const double>(
        cloud_->covs.data()->data(), 
        cloud_->covs.size() * 16
    );
}

void PointCloud::estimate_normals(int num_neighbors, int num_threads) {
    small_gicp::estimate_normals_omp(*cloud_, num_neighbors, num_threads);
}

void PointCloud::estimate_covariances(int num_neighbors, int num_threads) {
    small_gicp::estimate_covariances_omp(*cloud_, num_neighbors, num_threads);
}

std::unique_ptr<PointCloud> PointCloud::voxel_downsample(double voxel_size, int num_threads) const {
    auto result = std::make_unique<PointCloud>();
    result->cloud_ = small_gicp::voxelgrid_sampling_omp(*cloud_, voxel_size, num_threads);
    return result;
}

// KdTree implementation
KdTree::KdTree(const PointCloud& cloud, int num_threads) {
    tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
        cloud.get_internal(), small_gicp::KdTreeBuilderOMP{num_threads}
    );
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
    
    size_t num_found = tree_->knn_search(query, k, k_indices.data(), k_sq_dists.data());
    
    rust::Vec<size_t> result;
    result.reserve(num_found);
    for (size_t i = 0; i < num_found; ++i) {
        result.push_back(k_indices[i]);
    }
    return result;
}

rust::Vec<size_t> KdTree::radius_search(Point3d point, double radius) const {
    // small_gicp KdTree doesn't have radius_search, so we'll use knn_search with a large k
    // and filter by distance
    Eigen::Vector4d query(point.x, point.y, point.z, 1.0);
    const size_t max_k = 100; // reasonable upper bound
    std::vector<size_t> k_indices(max_k);
    std::vector<double> k_sq_dists(max_k);
    
    size_t num_found = tree_->knn_search(query, max_k, k_indices.data(), k_sq_dists.data());
    
    rust::Vec<size_t> result;
    double radius_sq = radius * radius;
    for (size_t i = 0; i < num_found; ++i) {
        if (k_sq_dists[i] <= radius_sq) {
            result.push_back(k_indices[i]);
        }
    }
    return result;
}

// GaussianVoxelMap implementation
GaussianVoxelMap::GaussianVoxelMap(double voxel_size) 
    : voxelmap_(std::make_shared<small_gicp::GaussianVoxelMap>(voxel_size)) {}

void GaussianVoxelMap::insert(const PointCloud& cloud) {
    voxelmap_->insert(cloud.get_internal());
}

size_t GaussianVoxelMap::size() const {
    return voxelmap_->size();
}

// Registration functions
RegistrationResult align_points_icp(
    const PointCloud& source,
    const PointCloud& target,
    const KdTree& target_tree,
    const Transform& init_guess,
    const RegistrationSettings& settings
) {
    // Convert transform
    Eigen::Matrix4d init_T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            init_T(i, j) = init_guess.matrix[i * 4 + j];
        }
    }
    
    // Use registration helper for ICP
    auto result = small_gicp::align(
        source.get_internal(), 
        target.get_internal(), 
        target_tree.get_internal(),
        init_T,
        small_gicp::RegistrationSetting()
            .type(\"ICP\")
            .max_iterations(settings.max_iterations)
            .num_threads(settings.num_threads)
    );
    
    // Convert result
    RegistrationResult reg_result;
    for (int i = 0; i < 16; ++i) {
        reg_result.transformation.matrix[i] = result.T.data()[i];
    }
    reg_result.converged = result.converged;
    reg_result.iterations = result.iterations;
    reg_result.error = result.error;
    
    return reg_result;
}

RegistrationResult align_points_gicp(
    const PointCloud& source,
    const PointCloud& target,
    const KdTree& source_tree,
    const KdTree& target_tree,
    const Transform& init_guess,
    const RegistrationSettings& settings
) {
    // Convert transform
    Eigen::Matrix4d init_T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            init_T(i, j) = init_guess.matrix[i * 4 + j];
        }
    }
    
    // Setup registration
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
    registration.reduction.num_threads = settings.num_threads;
    registration.optimizer.max_iterations = settings.max_iterations;
    registration.criteria.rotation_epsilon = settings.rotation_epsilon;
    registration.criteria.translation_epsilon = settings.transformation_epsilon;
    registration.rejector.max_dist_sq = settings.max_correspondence_distance * 
                                        settings.max_correspondence_distance;
    
    // Perform registration
    auto result = registration.align(source.get_internal(), target.get_internal(), 
                                     source_tree.get_internal(), target_tree.get_internal(), init_T);
    
    // Convert result
    RegistrationResult reg_result;
    for (int i = 0; i < 16; ++i) {
        reg_result.transformation.matrix[i] = result.T.data()[i];
    }
    reg_result.converged = result.converged;
    reg_result.iterations = result.iterations;
    reg_result.error = result.error;
    
    return reg_result;
}

RegistrationResult align_points_vgicp(
    const PointCloud& source,
    const GaussianVoxelMap& target_voxelmap,
    const Transform& init_guess,
    const RegistrationSettings& settings
) {
    // Convert transform
    Eigen::Matrix4d init_T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            init_T(i, j) = init_guess.matrix[i * 4 + j];
        }
    }
    
    // Create source voxelmap
    small_gicp::GaussianVoxelMap source_voxelmap(1.0);  // Use default resolution
    source_voxelmap.insert(source.get_internal());
    
    // Setup registration
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
    registration.reduction.num_threads = settings.num_threads;
    registration.optimizer.max_iterations = settings.max_iterations;
    registration.criteria.rotation_epsilon = settings.rotation_epsilon;
    registration.criteria.translation_epsilon = settings.transformation_epsilon;
    registration.rejector.max_dist_sq = settings.max_correspondence_distance * 
                                        settings.max_correspondence_distance;
    
    // Perform registration
    auto result = registration.align(source_voxelmap, target_voxelmap.get_internal(), init_T);
    
    // Convert result
    RegistrationResult reg_result;
    for (int i = 0; i < 16; ++i) {
        reg_result.transformation.matrix[i] = result.T.data()[i];
    }
    reg_result.converged = result.converged;
    reg_result.iterations = result.iterations;
    reg_result.error = result.error;
    
    return reg_result;
}

// Factory functions
std::unique_ptr<PointCloud> create_point_cloud() {
    return std::make_unique<PointCloud>();
}

std::unique_ptr<KdTree> create_kdtree(const PointCloud& cloud, int num_threads) {
    return std::make_unique<KdTree>(cloud, num_threads);
}

std::unique_ptr<GaussianVoxelMap> create_voxelmap(double voxel_size) {
    return std::make_unique<GaussianVoxelMap>(voxel_size);
}

} // namespace small_gicp_cxx