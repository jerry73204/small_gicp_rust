#include "../common.h"
#include <small_gicp_c/ann/kdtree.h>
#include <small_gicp_c/ann/unsafe_kdtree.h>
#include <small_gicp_c/points/point_cloud.h>

#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/ann/knn_result.hpp>
#include <small_gicp/ann/projection.hpp>
#include <small_gicp/points/point_cloud.hpp>

#include <memory>
#include <variant>

// Type aliases for different projection types
using AxisAlignedProjection = small_gicp::AxisAlignedProjection;
using NormalProjection = small_gicp::NormalProjection;

// Variant type to hold different UnsafeKdTree types
using UnsafeKdTreeVariant = std::variant<
    std::unique_ptr<small_gicp::UnsafeKdTree<small_gicp::PointCloud,
                                             AxisAlignedProjection>>,
    std::unique_ptr<
        small_gicp::UnsafeKdTree<small_gicp::PointCloud, NormalProjection>>>;

struct small_gicp_unsafe_kdtree {
  UnsafeKdTreeVariant kdtree;
  small_gicp_projection_type_t projection_type;
  const small_gicp_point_cloud_t
      *cloud_ref; // Keep reference to ensure lifetime
};

template <typename UnsafeKdTree>
UnsafeKdTree *get_unsafe_kdtree_ptr(const small_gicp_unsafe_kdtree_t *kdtree) {
  if (auto ptr = std::get_if<std::unique_ptr<UnsafeKdTree>>(&kdtree->kdtree)) {
    return ptr->get();
  }
  return nullptr;
}

small_gicp_error_t
small_gicp_create_default_knn_setting(small_gicp_knn_setting_t *setting) {
  if (!setting) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  setting->epsilon = 0.0; // Exact search by default
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t small_gicp_create_default_projection_config(
    small_gicp_projection_config_t *config) {
  if (!config) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  config->type = SMALL_GICP_PROJECTION_AXIS_ALIGNED;
  config->max_scan_count = 128;
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t small_gicp_create_default_kdtree_config_extended(
    small_gicp_kdtree_config_extended_t *config) {
  if (!config) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  config->builder_type = SMALL_GICP_KDTREE_BUILDER_DEFAULT;
  config->num_threads = 1;
  config->max_leaf_size = 20;

  small_gicp_create_default_projection_config(&config->projection);

  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_unsafe_kdtree_create(const small_gicp_point_cloud_t *cloud,
                                small_gicp_unsafe_kdtree_t **kdtree) {
  small_gicp_kdtree_config_t config;
  config.builder_type = SMALL_GICP_KDTREE_BUILDER_DEFAULT;
  config.num_threads = 1;
  config.max_leaf_size = 20;

  return small_gicp_unsafe_kdtree_create_with_config(cloud, &config, kdtree);
}

small_gicp_error_t small_gicp_unsafe_kdtree_create_with_config(
    const small_gicp_point_cloud_t *cloud,
    const small_gicp_kdtree_config_t *config,
    small_gicp_unsafe_kdtree_t **kdtree) {
  // Convert basic config to extended config
  small_gicp_kdtree_config_extended_t extended_config;
  extended_config.builder_type = config->builder_type;
  extended_config.num_threads = config->num_threads;
  extended_config.max_leaf_size = config->max_leaf_size;
  small_gicp_create_default_projection_config(&extended_config.projection);

  return small_gicp_unsafe_kdtree_create_with_extended_config(
      cloud, &extended_config, kdtree);
}

small_gicp_error_t small_gicp_unsafe_kdtree_create_with_extended_config(
    const small_gicp_point_cloud_t *cloud,
    const small_gicp_kdtree_config_extended_t *config,
    small_gicp_unsafe_kdtree_t **kdtree) {
  if (!cloud || !cloud->cloud || !config || !kdtree) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto wrapper = std::make_unique<small_gicp_unsafe_kdtree_t>();
    wrapper->projection_type = config->projection.type;
    wrapper->cloud_ref = cloud;

    // Create projection setting
    small_gicp::ProjectionSetting proj_setting;
    proj_setting.max_scan_count = config->projection.max_scan_count;

    // Create KdTree builder
    small_gicp::KdTreeBuilder builder;
    builder.max_leaf_size = config->max_leaf_size;
    builder.projection_setting = proj_setting;

    switch (config->projection.type) {
    case SMALL_GICP_PROJECTION_AXIS_ALIGNED: {
      switch (config->builder_type) {
#ifdef SMALL_GICP_HAS_OPENMP
      case SMALL_GICP_KDTREE_BUILDER_OPENMP: {
        auto kdtree_ptr =
            std::make_unique<small_gicp::UnsafeKdTree<small_gicp::PointCloud,
                                                      AxisAlignedProjection>>(
                *cloud->cloud,
                small_gicp::KdTreeBuilderOMP(config->num_threads, builder));
        wrapper->kdtree = std::move(kdtree_ptr);
        break;
      }
#endif
#ifdef SMALL_GICP_HAS_TBB
      case SMALL_GICP_KDTREE_BUILDER_TBB: {
        auto kdtree_ptr =
            std::make_unique<small_gicp::UnsafeKdTree<small_gicp::PointCloud,
                                                      AxisAlignedProjection>>(
                *cloud->cloud,
                small_gicp::KdTreeBuilderTBB(config->num_threads, builder));
        wrapper->kdtree = std::move(kdtree_ptr);
        break;
      }
#endif
      default: {
        auto kdtree_ptr =
            std::make_unique<small_gicp::UnsafeKdTree<small_gicp::PointCloud,
                                                      AxisAlignedProjection>>(
                *cloud->cloud, builder);
        wrapper->kdtree = std::move(kdtree_ptr);
        break;
      }
      }
      break;
    }

    case SMALL_GICP_PROJECTION_NORMAL: {
      switch (config->builder_type) {
#ifdef SMALL_GICP_HAS_OPENMP
      case SMALL_GICP_KDTREE_BUILDER_OPENMP: {
        auto kdtree_ptr = std::make_unique<
            small_gicp::UnsafeKdTree<small_gicp::PointCloud, NormalProjection>>(
            *cloud->cloud,
            small_gicp::KdTreeBuilderOMP(config->num_threads, builder));
        wrapper->kdtree = std::move(kdtree_ptr);
        break;
      }
#endif
#ifdef SMALL_GICP_HAS_TBB
      case SMALL_GICP_KDTREE_BUILDER_TBB: {
        auto kdtree_ptr = std::make_unique<
            small_gicp::UnsafeKdTree<small_gicp::PointCloud, NormalProjection>>(
            *cloud->cloud,
            small_gicp::KdTreeBuilderTBB(config->num_threads, builder));
        wrapper->kdtree = std::move(kdtree_ptr);
        break;
      }
#endif
      default: {
        auto kdtree_ptr = std::make_unique<
            small_gicp::UnsafeKdTree<small_gicp::PointCloud, NormalProjection>>(
            *cloud->cloud, builder);
        wrapper->kdtree = std::move(kdtree_ptr);
        break;
      }
      }
      break;
    }

    default:
      return SMALL_GICP_ERROR_INVALID_ARGUMENT;
    }

    *kdtree = wrapper.release();
    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t
small_gicp_unsafe_kdtree_destroy(small_gicp_unsafe_kdtree_t *kdtree) {
  if (!kdtree) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  delete kdtree;
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t small_gicp_unsafe_kdtree_nearest_neighbor_search(
    const small_gicp_unsafe_kdtree_t *kdtree, double query_x, double query_y,
    double query_z, size_t *index, double *distance) {
  small_gicp_knn_setting_t setting;
  small_gicp_create_default_knn_setting(&setting);

  return small_gicp_unsafe_kdtree_nearest_neighbor_search_with_setting(
      kdtree, query_x, query_y, query_z, &setting, index, distance);
}

small_gicp_error_t
small_gicp_unsafe_kdtree_nearest_neighbor_search_with_setting(
    const small_gicp_unsafe_kdtree_t *kdtree, double query_x, double query_y,
    double query_z, const small_gicp_knn_setting_t *setting, size_t *index,
    double *distance) {
  if (!kdtree || !setting || !index || !distance) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    Eigen::Vector4d query(query_x, query_y, query_z, 1.0);
    small_gicp::KnnSetting knn_setting;
    knn_setting.epsilon = setting->epsilon;

    // Initialize with defaults in case no neighbor is found
    *index = SIZE_MAX;
    *distance = std::numeric_limits<double>::infinity();

    std::visit(
        [&](const auto &tree_ptr) {
          double sq_dist = 0.0;
          size_t found = tree_ptr->nearest_neighbor_search(
              query, index, &sq_dist, knn_setting);
          if (found > 0) {
            *distance = std::sqrt(sq_dist);
          } else {
            *index = SIZE_MAX;
            *distance = std::numeric_limits<double>::infinity();
          }
        },
        kdtree->kdtree);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_unsafe_kdtree_knn_search(
    const small_gicp_unsafe_kdtree_t *kdtree, double query_x, double query_y,
    double query_z, int k, size_t *indices, double *distances) {
  small_gicp_knn_setting_t setting;
  small_gicp_create_default_knn_setting(&setting);

  return small_gicp_unsafe_kdtree_knn_search_with_setting(
      kdtree, query_x, query_y, query_z, k, &setting, indices, distances);
}

small_gicp_error_t small_gicp_unsafe_kdtree_knn_search_with_setting(
    const small_gicp_unsafe_kdtree_t *kdtree, double query_x, double query_y,
    double query_z, int k, const small_gicp_knn_setting_t *setting,
    size_t *indices, double *distances) {
  if (!kdtree || !setting || k <= 0 || !indices || !distances) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    Eigen::Vector4d query(query_x, query_y, query_z, 1.0);
    small_gicp::KnnSetting knn_setting;
    knn_setting.epsilon = setting->epsilon;

    // Create temporary arrays for squared distances
    std::vector<double> sq_distances(k);

    std::visit(
        [&](const auto &tree_ptr) {
          size_t found = tree_ptr->knn_search(query, k, indices,
                                              sq_distances.data(), knn_setting);

          // Convert squared distances to distances and fill remaining slots
          for (int i = 0; i < k; i++) {
            if (i < static_cast<int>(found)) {
              distances[i] = std::sqrt(sq_distances[i]);
            } else {
              indices[i] = SIZE_MAX;
              distances[i] = std::numeric_limits<double>::infinity();
            }
          }
        },
        kdtree->kdtree);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

// Enhanced versions of existing KdTree functions with KNN settings
small_gicp_error_t small_gicp_kdtree_nearest_neighbor_search_with_setting(
    const small_gicp_kdtree_t *kdtree, double query_x, double query_y,
    double query_z, const small_gicp_knn_setting_t *setting, size_t *index,
    double *distance) {
  if (!kdtree || !setting || !index || !distance) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    Eigen::Vector4d query(query_x, query_y, query_z, 1.0);
    small_gicp::KnnSetting knn_setting;
    knn_setting.epsilon = setting->epsilon;

    // Initialize with defaults in case no neighbor is found
    *index = SIZE_MAX;
    *distance = std::numeric_limits<double>::infinity();

    double sq_dist = 0.0;
    size_t found = kdtree->tree->nearest_neighbor_search(query, index, &sq_dist,
                                                         knn_setting);
    if (found > 0) {
      *distance = std::sqrt(sq_dist);
    }

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_kdtree_knn_search_with_setting(
    const small_gicp_kdtree_t *kdtree, double query_x, double query_y,
    double query_z, int k, const small_gicp_knn_setting_t *setting,
    size_t *indices, double *distances) {
  if (!kdtree || !setting || k <= 0 || !indices || !distances) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    Eigen::Vector4d query(query_x, query_y, query_z, 1.0);
    small_gicp::KnnSetting knn_setting;
    knn_setting.epsilon = setting->epsilon;

    // Create temporary arrays for squared distances
    std::vector<double> sq_distances(k);

    size_t found = kdtree->tree->knn_search(query, k, indices,
                                            sq_distances.data(), knn_setting);

    // Convert squared distances to distances and fill remaining slots
    for (int i = 0; i < k; i++) {
      if (i < static_cast<int>(found)) {
        distances[i] = std::sqrt(sq_distances[i]);
      } else {
        indices[i] = SIZE_MAX;
        distances[i] = std::numeric_limits<double>::infinity();
      }
    }

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_kdtree_create_with_extended_config(
    const small_gicp_point_cloud_t *cloud,
    const small_gicp_kdtree_config_extended_t *config,
    small_gicp_kdtree_t **kdtree) {
  if (!cloud || !cloud->cloud || !config || !kdtree) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto wrapper = std::make_unique<small_gicp_kdtree>();

    // Create projection setting
    small_gicp::ProjectionSetting proj_setting;
    proj_setting.max_scan_count = config->projection.max_scan_count;

    // Create KdTree builder
    small_gicp::KdTreeBuilder builder;
    builder.max_leaf_size = config->max_leaf_size;
    builder.projection_setting = proj_setting;

    // For regular KdTree, we'll use AxisAlignedProjection for simplicity
    // The projection type mainly affects the UnsafeKdTree construction
    switch (config->builder_type) {
#ifdef SMALL_GICP_HAS_OPENMP
    case SMALL_GICP_KDTREE_BUILDER_OPENMP: {
      wrapper->tree =
          std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
              cloud->cloud,
              small_gicp::KdTreeBuilderOMP(config->num_threads, builder));
      break;
    }
#endif
#ifdef SMALL_GICP_HAS_TBB
    case SMALL_GICP_KDTREE_BUILDER_TBB: {
      wrapper->tree =
          std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
              cloud->cloud,
              small_gicp::KdTreeBuilderTBB(config->num_threads, builder));
      break;
    }
#endif
    default: {
      wrapper->tree =
          std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
              cloud->cloud, builder);
      break;
    }
    }

    *kdtree = wrapper.release();
    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}
