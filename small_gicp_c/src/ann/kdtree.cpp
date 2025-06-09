#include <small_gicp_c/ann/kdtree.h>
#include "../common.h"

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/ann/kdtree_tbb.hpp>

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

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
  TRY_CATCH_END
}

// KdTree creation with specific builder type
small_gicp_error_t
small_gicp_kdtree_create_with_builder(const small_gicp_point_cloud_t *cloud,
                                      small_gicp_kdtree_builder_type_t builder_type,
                                      int num_threads,
                                      small_gicp_kdtree_t **kdtree) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);

  TRY_CATCH_BEGIN
  *kdtree = new small_gicp_kdtree;

  switch (builder_type) {
    case SMALL_GICP_KDTREE_BUILDER_DEFAULT: {
      (*kdtree)->tree = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(cloud->cloud);
      break;
    }
    case SMALL_GICP_KDTREE_BUILDER_OPENMP: {
      if (num_threads <= 0) num_threads = 4;
      small_gicp::KdTreeBuilderOMP builder(num_threads);
      (*kdtree)->tree = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(cloud->cloud, builder);
      break;
    }
    case SMALL_GICP_KDTREE_BUILDER_TBB: {
      small_gicp::KdTreeBuilderTBB builder;
      (*kdtree)->tree = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(cloud->cloud, builder);
      break;
    }
    default:
      delete *kdtree;
      *kdtree = nullptr;
      return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}

// KdTree creation with full configuration
small_gicp_error_t
small_gicp_kdtree_create_with_config(const small_gicp_point_cloud_t *cloud,
                                     const small_gicp_kdtree_config_t *config,
                                     small_gicp_kdtree_t **kdtree) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(config);
  CHECK_NULL(kdtree);

  TRY_CATCH_BEGIN
  *kdtree = new small_gicp_kdtree;

  switch (config->builder_type) {
    case SMALL_GICP_KDTREE_BUILDER_DEFAULT: {
      (*kdtree)->tree = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(cloud->cloud);
      break;
    }
    case SMALL_GICP_KDTREE_BUILDER_OPENMP: {
      int num_threads = config->num_threads > 0 ? config->num_threads : 4;
      small_gicp::KdTreeBuilderOMP builder(num_threads);
      builder.max_leaf_size = config->max_leaf_size > 0 ? config->max_leaf_size : 20;
      (*kdtree)->tree = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(cloud->cloud, builder);
      break;
    }
    case SMALL_GICP_KDTREE_BUILDER_TBB: {
      small_gicp::KdTreeBuilderTBB builder;
      builder.max_leaf_size = config->max_leaf_size > 0 ? config->max_leaf_size : 20;
      (*kdtree)->tree = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(cloud->cloud, builder);
      break;
    }
    default:
      delete *kdtree;
      *kdtree = nullptr;
      return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}

// KdTree configuration helpers
small_gicp_error_t
small_gicp_kdtree_config_create_default(small_gicp_kdtree_config_t **config) {
  CHECK_NULL(config);

  TRY_CATCH_BEGIN
  *config = new small_gicp_kdtree_config_t;
  (*config)->builder_type = SMALL_GICP_KDTREE_BUILDER_DEFAULT;
  (*config)->num_threads = 4;
  (*config)->max_leaf_size = 20;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_kdtree_config_create(small_gicp_kdtree_builder_type_t builder_type,
                               int num_threads, int max_leaf_size,
                               small_gicp_kdtree_config_t **config) {
  CHECK_NULL(config);

  TRY_CATCH_BEGIN
  *config = new small_gicp_kdtree_config_t;
  (*config)->builder_type = builder_type;
  (*config)->num_threads = num_threads > 0 ? num_threads : 4;
  (*config)->max_leaf_size = max_leaf_size > 0 ? max_leaf_size : 20;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_kdtree_config_destroy(small_gicp_kdtree_config_t *config) {
  if (config) {
    delete config;
  }
  return SMALL_GICP_SUCCESS;
}