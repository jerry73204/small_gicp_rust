#include "small_gicp_c.h"

#include <memory> // Must be included before small_gicp headers

#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/benchmark/read_points.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/util/downsampling.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <random>

// Wrapper structs to hold C++ objects
struct small_gicp_point_cloud {
  std::shared_ptr<small_gicp::PointCloud> cloud;
};

struct small_gicp_kdtree {
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> tree;
};

struct small_gicp_gaussian_voxelmap {
  small_gicp::GaussianVoxelMap::Ptr voxelmap;
};

// Helper macros for error handling
#define TRY_CATCH_BEGIN try {
#define TRY_CATCH_END                                                          \
  }                                                                            \
  catch (const std::bad_alloc &) {                                             \
    return SMALL_GICP_ERROR_OUT_OF_MEMORY;                                     \
  }                                                                            \
  catch (const std::exception &e) {                                            \
    std::cerr << "small_gicp_c error: " << e.what() << std::endl;              \
    return SMALL_GICP_ERROR_EXCEPTION;                                         \
  }                                                                            \
  catch (...) {                                                                \
    return SMALL_GICP_ERROR_EXCEPTION;                                         \
  }

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

// Point cloud creation and destruction
small_gicp_error_t
small_gicp_point_cloud_create(small_gicp_point_cloud_t **cloud) {
  CHECK_NULL(cloud);

  TRY_CATCH_BEGIN
  *cloud = new small_gicp_point_cloud;
  (*cloud)->cloud = std::make_shared<small_gicp::PointCloud>();
  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_point_cloud_destroy(small_gicp_point_cloud_t *cloud) {
  if (cloud) {
    delete cloud;
  }
  return SMALL_GICP_SUCCESS;
}

// Point cloud data access
small_gicp_error_t
small_gicp_point_cloud_resize(small_gicp_point_cloud_t *cloud, size_t n) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);

  TRY_CATCH_BEGIN
  cloud->cloud->resize(n);
  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_point_cloud_size(const small_gicp_point_cloud_t *cloud,
                            size_t *size) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(size);

  *size = cloud->cloud->size();
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_point_cloud_set_point(small_gicp_point_cloud_t *cloud, size_t index,
                                 double x, double y, double z) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);

  if (index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  cloud->cloud->point(index) = Eigen::Vector4d(x, y, z, 1.0);
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_point_cloud_get_point(const small_gicp_point_cloud_t *cloud,
                                 size_t index, double *x, double *y,
                                 double *z) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(x);
  CHECK_NULL(y);
  CHECK_NULL(z);

  if (index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  const auto &point = cloud->cloud->point(index);
  *x = point.x();
  *y = point.y();
  *z = point.z();
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_point_cloud_set_normal(small_gicp_point_cloud_t *cloud, size_t index,
                                  double nx, double ny, double nz) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);

  if (index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  cloud->cloud->normal(index) = Eigen::Vector4d(nx, ny, nz, 0.0);
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_point_cloud_get_normal(const small_gicp_point_cloud_t *cloud,
                                  size_t index, double *nx, double *ny,
                                  double *nz) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(nx);
  CHECK_NULL(ny);
  CHECK_NULL(nz);

  if (index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  const auto &normal = cloud->cloud->normal(index);
  *nx = normal.x();
  *ny = normal.y();
  *nz = normal.z();
  return SMALL_GICP_SUCCESS;
}

// Point cloud I/O
small_gicp_error_t small_gicp_load_ply(const char *filename,
                                       small_gicp_point_cloud_t **cloud) {
  CHECK_NULL(filename);
  CHECK_NULL(cloud);

  TRY_CATCH_BEGIN
  auto points = small_gicp::read_ply(filename);
  if (points.empty()) {
    return SMALL_GICP_ERROR_FILE_NOT_FOUND;
  }

  *cloud = new small_gicp_point_cloud;
  (*cloud)->cloud = std::make_shared<small_gicp::PointCloud>();

  // Convert from vector of Vector4f to PointCloud
  (*cloud)->cloud->resize(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    (*cloud)->cloud->point(i) = points[i].cast<double>();
  }

  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_save_ply(const char *filename,
                                       const small_gicp_point_cloud_t *cloud) {
  CHECK_NULL(filename);
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);

  TRY_CATCH_BEGIN
  // Simple PLY writer
  std::ofstream ofs(filename);
  if (!ofs) {
    return SMALL_GICP_ERROR_IO_ERROR;
  }

  const auto &pc = *cloud->cloud;
  ofs << "ply\n";
  ofs << "format ascii 1.0\n";
  ofs << "element vertex " << pc.size() << "\n";
  ofs << "property float x\n";
  ofs << "property float y\n";
  ofs << "property float z\n";
  if (!pc.normals.empty()) {
    ofs << "property float nx\n";
    ofs << "property float ny\n";
    ofs << "property float nz\n";
  }
  ofs << "end_header\n";

  for (size_t i = 0; i < pc.size(); ++i) {
    const auto &p = pc.point(i);
    ofs << p.x() << " " << p.y() << " " << p.z();
    if (!pc.normals.empty()) {
      const auto &n = pc.normal(i);
      ofs << " " << n.x() << " " << n.y() << " " << n.z();
    }
    ofs << "\n";
  }

  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_load_points_from_array(const float *points, size_t num_points,
                                  small_gicp_point_cloud_t **cloud) {
  CHECK_NULL(points);
  CHECK_NULL(cloud);

  TRY_CATCH_BEGIN
  *cloud = new small_gicp_point_cloud;
  (*cloud)->cloud = std::make_shared<small_gicp::PointCloud>();
  (*cloud)->cloud->resize(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    (*cloud)->cloud->point(i) = Eigen::Vector4d(
        points[i * 3 + 0], points[i * 3 + 1], points[i * 3 + 2], 1.0);
  }

  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

// Preprocessing
small_gicp_error_t
small_gicp_preprocess_points(const small_gicp_point_cloud_t *cloud,
                             double downsampling_resolution, int num_neighbors,
                             int num_threads,
                             small_gicp_point_cloud_t **preprocessed_cloud,
                             small_gicp_kdtree_t **kdtree) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(preprocessed_cloud);
  CHECK_NULL(kdtree);

  TRY_CATCH_BEGIN
  auto [downsampled, tree] = small_gicp::preprocess_points(
      *cloud->cloud, downsampling_resolution, num_neighbors, num_threads);

  *preprocessed_cloud = new small_gicp_point_cloud;
  (*preprocessed_cloud)->cloud = downsampled;

  *kdtree = new small_gicp_kdtree;
  (*kdtree)->tree = tree;

  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

// Downsampling
small_gicp_error_t
small_gicp_voxelgrid_sampling(const small_gicp_point_cloud_t *cloud,
                              double leaf_size, int num_threads,
                              small_gicp_point_cloud_t **downsampled) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(downsampled);

  TRY_CATCH_BEGIN
  auto result = (num_threads > 1)
                    ? small_gicp::voxelgrid_sampling_omp(*cloud->cloud,
                                                         leaf_size, num_threads)
                    : small_gicp::voxelgrid_sampling(*cloud->cloud, leaf_size);

  *downsampled = new small_gicp_point_cloud;
  (*downsampled)->cloud = result;

  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_random_sampling(const small_gicp_point_cloud_t *cloud,
                           size_t num_samples,
                           small_gicp_point_cloud_t **downsampled) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(downsampled);

  TRY_CATCH_BEGIN
  std::mt19937 rng(42); // Fixed seed for reproducibility
  auto result = small_gicp::random_sampling(*cloud->cloud, num_samples, rng);

  *downsampled = new small_gicp_point_cloud;
  (*downsampled)->cloud = result;

  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

// Normal and covariance estimation
small_gicp_error_t
small_gicp_estimate_normals(small_gicp_point_cloud_t *cloud,
                            const small_gicp_kdtree_t *kdtree,
                            int num_neighbors, int num_threads) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);

  TRY_CATCH_BEGIN
  if (num_threads > 1) {
    small_gicp::estimate_normals_omp(*cloud->cloud, *kdtree->tree,
                                     num_neighbors, num_threads);
  } else {
    small_gicp::estimate_normals(*cloud->cloud, *kdtree->tree, num_neighbors);
  }
  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_estimate_covariances(small_gicp_point_cloud_t *cloud,
                                const small_gicp_kdtree_t *kdtree,
                                int num_neighbors, int num_threads) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);

  TRY_CATCH_BEGIN
  if (num_threads > 1) {
    small_gicp::estimate_covariances_omp(*cloud->cloud, *kdtree->tree,
                                         num_neighbors, num_threads);
  } else {
    small_gicp::estimate_covariances(*cloud->cloud, *kdtree->tree,
                                     num_neighbors);
  }
  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_estimate_normals_covariances(small_gicp_point_cloud_t *cloud,
                                        const small_gicp_kdtree_t *kdtree,
                                        int num_neighbors, int num_threads) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);

  TRY_CATCH_BEGIN
  if (num_threads > 1) {
    small_gicp::estimate_normals_covariances_omp(*cloud->cloud, *kdtree->tree,
                                                 num_neighbors, num_threads);
  } else {
    small_gicp::estimate_normals_covariances(*cloud->cloud, *kdtree->tree,
                                             num_neighbors);
  }
  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

// KdTree creation and search
small_gicp_error_t
small_gicp_kdtree_create(const small_gicp_point_cloud_t *cloud, int num_threads,
                         small_gicp_kdtree_t **kdtree) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);

  TRY_CATCH_BEGIN
  *kdtree = new small_gicp_kdtree;

  if (num_threads > 1) {
    // Use the parallel KdTree builder for multi-threaded construction
    small_gicp::KdTreeBuilderOMP builder(num_threads);
    (*kdtree)->tree =
        std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
            cloud->cloud, builder);
  } else {
    // Use default single-threaded builder
    (*kdtree)->tree =
        std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
            cloud->cloud);
  }

  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_kdtree_destroy(small_gicp_kdtree_t *kdtree) {
  if (kdtree) {
    delete kdtree;
  }
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_kdtree_nearest_neighbor_search(const small_gicp_kdtree_t *kdtree,
                                          double x, double y, double z,
                                          size_t *index, double *sq_dist) {
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);
  CHECK_NULL(index);
  CHECK_NULL(sq_dist);

  TRY_CATCH_BEGIN
  Eigen::Vector4d query(x, y, z, 1.0);
  size_t num_found =
      kdtree->tree->nearest_neighbor_search(query, index, sq_dist);

  if (num_found == 0) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_kdtree_knn_search(const small_gicp_kdtree_t *kdtree, double x,
                             double y, double z, int k, size_t *indices,
                             double *sq_dists) {
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);
  CHECK_NULL(indices);
  CHECK_NULL(sq_dists);

  TRY_CATCH_BEGIN
  Eigen::Vector4d query(x, y, z, 1.0);
  size_t num_found = kdtree->tree->knn_search(query, k, indices, sq_dists);

  if (num_found == 0) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

// Gaussian voxel map
small_gicp_error_t
small_gicp_gaussian_voxelmap_create(const small_gicp_point_cloud_t *cloud,
                                    double voxel_resolution, int num_threads,
                                    small_gicp_gaussian_voxelmap_t **voxelmap) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(voxelmap);

  TRY_CATCH_BEGIN
  *voxelmap = new small_gicp_gaussian_voxelmap;
  (*voxelmap)->voxelmap =
      small_gicp::create_gaussian_voxelmap(*cloud->cloud, voxel_resolution);
  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_gaussian_voxelmap_destroy(small_gicp_gaussian_voxelmap_t *voxelmap) {
  if (voxelmap) {
    delete voxelmap;
  }
  return SMALL_GICP_SUCCESS;
}

// Registration functions
small_gicp_error_t small_gicp_align(const small_gicp_point_cloud_t *target,
                                    const small_gicp_point_cloud_t *source,
                                    small_gicp_registration_type_t type,
                                    const double *initial_guess,
                                    int num_threads,
                                    small_gicp_registration_result_t *result) {
  CHECK_NULL(target);
  CHECK_NULL(target->cloud);
  CHECK_NULL(source);
  CHECK_NULL(source->cloud);
  CHECK_NULL(result);

  TRY_CATCH_BEGIN
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
  if (initial_guess) {
    Eigen::Matrix4d mat;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        mat(i, j) = initial_guess[i * 4 + j];
      }
    }
    init_guess.matrix() = mat;
  }

  small_gicp::RegistrationSetting setting;
  setting.num_threads = num_threads;
  setting.type =
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(type);

  // For direct alignment, we need to preprocess first
  auto [target_preprocessed, target_tree_temp] = small_gicp::preprocess_points(
      *target->cloud, setting.downsampling_resolution, 10, setting.num_threads);
  auto [source_preprocessed, source_tree_temp] = small_gicp::preprocess_points(
      *source->cloud, setting.downsampling_resolution, 10, setting.num_threads);

  small_gicp::RegistrationResult reg_result;
  if (setting.type == small_gicp::RegistrationSetting::VGICP) {
    auto target_voxelmap_temp = small_gicp::create_gaussian_voxelmap(
        *target_preprocessed, setting.voxel_resolution);
    reg_result = small_gicp::align(*target_voxelmap_temp, *source_preprocessed,
                                   init_guess, setting);
  } else {
    reg_result = small_gicp::align(*target_preprocessed, *source_preprocessed,
                                   *target_tree_temp, init_guess, setting);
  }

  // Copy transformation matrix
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result->T_target_source[i * 4 + j] = reg_result.T_target_source(i, j);
    }
  }

  result->converged = reg_result.converged;
  result->iterations = reg_result.iterations;
  result->num_inliers = reg_result.num_inliers;
  result->error = reg_result.error;

  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_align_preprocessed(const small_gicp_point_cloud_t *target,
                              const small_gicp_point_cloud_t *source,
                              const small_gicp_kdtree_t *target_tree,
                              small_gicp_registration_type_t type,
                              const double *initial_guess, int num_threads,
                              small_gicp_registration_result_t *result) {
  CHECK_NULL(target);
  CHECK_NULL(target->cloud);
  CHECK_NULL(source);
  CHECK_NULL(source->cloud);
  CHECK_NULL(target_tree);
  CHECK_NULL(target_tree->tree);
  CHECK_NULL(result);

  TRY_CATCH_BEGIN
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
  if (initial_guess) {
    Eigen::Matrix4d mat;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        mat(i, j) = initial_guess[i * 4 + j];
      }
    }
    init_guess.matrix() = mat;
  }

  small_gicp::RegistrationSetting setting;
  setting.num_threads = num_threads;
  setting.type =
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(type);

  auto reg_result = small_gicp::align(*target->cloud, *source->cloud,
                                      *target_tree->tree, init_guess, setting);

  // Copy transformation matrix
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result->T_target_source[i * 4 + j] = reg_result.T_target_source(i, j);
    }
  }

  result->converged = reg_result.converged;
  result->iterations = reg_result.iterations;
  result->num_inliers = reg_result.num_inliers;
  result->error = reg_result.error;

  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_align_vgicp(const small_gicp_gaussian_voxelmap_t *target_voxelmap,
                       const small_gicp_point_cloud_t *source,
                       const double *initial_guess, int num_threads,
                       small_gicp_registration_result_t *result) {
  CHECK_NULL(target_voxelmap);
  CHECK_NULL(target_voxelmap->voxelmap);
  CHECK_NULL(source);
  CHECK_NULL(source->cloud);
  CHECK_NULL(result);

  TRY_CATCH_BEGIN
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
  if (initial_guess) {
    Eigen::Matrix4d mat;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        mat(i, j) = initial_guess[i * 4 + j];
      }
    }
    init_guess.matrix() = mat;
  }

  small_gicp::RegistrationSetting setting;
  setting.num_threads = num_threads;

  auto reg_result = small_gicp::align(*target_voxelmap->voxelmap,
                                      *source->cloud, init_guess, setting);

  // Copy transformation matrix
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result->T_target_source[i * 4 + j] = reg_result.T_target_source(i, j);
    }
  }

  result->converged = reg_result.converged;
  result->iterations = reg_result.iterations;
  result->num_inliers = reg_result.num_inliers;
  result->error = reg_result.error;

  return SMALL_GICP_SUCCESS;
  TRY_CATCH_END
}

// Utility functions
small_gicp_error_t small_gicp_get_version(char *buffer, size_t buffer_size) {
  CHECK_NULL(buffer);

  const char *version = "1.0.0";
  size_t len = strlen(version);

  if (buffer_size <= len) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  strcpy(buffer, version);
  return SMALL_GICP_SUCCESS;
}

const char *small_gicp_error_string(small_gicp_error_t error) {
  switch (error) {
  case SMALL_GICP_SUCCESS:
    return "Success";
  case SMALL_GICP_ERROR_INVALID_ARGUMENT:
    return "Invalid argument";
  case SMALL_GICP_ERROR_OUT_OF_MEMORY:
    return "Out of memory";
  case SMALL_GICP_ERROR_FILE_NOT_FOUND:
    return "File not found";
  case SMALL_GICP_ERROR_IO_ERROR:
    return "I/O error";
  case SMALL_GICP_ERROR_EXCEPTION:
    return "Exception caught";
  default:
    return "Unknown error";
  }
}
