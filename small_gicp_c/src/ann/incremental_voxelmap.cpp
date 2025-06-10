#include "../common.h"
#include <small_gicp_c/ann/gaussian_voxelmap.h>
#include <small_gicp_c/ann/incremental_voxelmap.h>
#include <small_gicp_c/points/point_cloud.h>

#include <small_gicp/ann/flat_container.hpp>
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/incremental_voxelmap.hpp>
#include <small_gicp/points/point_cloud.hpp>

#include <memory>
#include <variant>

// Type aliases for different voxel container types
using FlatContainerPoints = small_gicp::FlatContainer<false, false>;
using FlatContainerNormal = small_gicp::FlatContainer<true, false>;
using FlatContainerCov = small_gicp::FlatContainer<false, true>;
using FlatContainerNormalCov = small_gicp::FlatContainer<true, true>;
using GaussianVoxel = small_gicp::GaussianVoxel;

// Variant type to hold different voxelmap types
using VoxelMapVariant = std::variant<
    std::shared_ptr<small_gicp::IncrementalVoxelMap<FlatContainerPoints>>,
    std::shared_ptr<small_gicp::IncrementalVoxelMap<FlatContainerNormal>>,
    std::shared_ptr<small_gicp::IncrementalVoxelMap<FlatContainerCov>>,
    std::shared_ptr<small_gicp::IncrementalVoxelMap<FlatContainerNormalCov>>,
    std::shared_ptr<small_gicp::IncrementalVoxelMap<GaussianVoxel>>>;

struct small_gicp_incremental_voxelmap {
  VoxelMapVariant voxelmap;
  small_gicp_voxel_container_type_t container_type;
  double leaf_size;
  small_gicp_flat_container_config_t config;
};

template <typename VoxelMap>
VoxelMap *get_voxelmap_ptr(const small_gicp_incremental_voxelmap_t *voxelmap) {
  if (auto ptr = std::get_if<std::shared_ptr<VoxelMap>>(&voxelmap->voxelmap)) {
    return ptr->get();
  }
  return nullptr;
}

small_gicp_error_t small_gicp_create_default_flat_container_config(
    small_gicp_flat_container_config_t *config) {
  if (!config) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  config->min_sq_dist_in_cell = 0.01;  // 10cm minimum distance
  config->max_num_points_in_cell = 20; // 20 points maximum per cell
  config->lru_horizon = 100.0;         // Keep voxels for 100 insertions
  config->lru_clear_cycle = 10;        // Clear every 10 insertions

  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t small_gicp_incremental_voxelmap_create(
    double leaf_size, small_gicp_voxel_container_type_t container_type,
    small_gicp_incremental_voxelmap_t **voxelmap) {
  small_gicp_flat_container_config_t config;
  small_gicp_create_default_flat_container_config(&config);

  return small_gicp_incremental_voxelmap_create_with_config(
      leaf_size, container_type, &config, voxelmap);
}

small_gicp_error_t small_gicp_incremental_voxelmap_create_with_config(
    double leaf_size, small_gicp_voxel_container_type_t container_type,
    const small_gicp_flat_container_config_t *config,
    small_gicp_incremental_voxelmap_t **voxelmap) {
  if (leaf_size <= 0.0 || !config || !voxelmap) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto wrapper = std::make_unique<small_gicp_incremental_voxelmap_t>();
    wrapper->container_type = container_type;
    wrapper->leaf_size = leaf_size;
    wrapper->config = *config;

    switch (container_type) {
    case SMALL_GICP_VOXEL_FLAT_POINTS: {
      auto vmap = std::make_shared<
          small_gicp::IncrementalVoxelMap<FlatContainerPoints>>(leaf_size);
      if (config->lru_horizon > 0) {
        vmap->lru_horizon = config->lru_horizon;
      }
      if (config->lru_clear_cycle > 0) {
        vmap->lru_clear_cycle = config->lru_clear_cycle;
      }
      wrapper->voxelmap = vmap;
      break;
    }
    case SMALL_GICP_VOXEL_FLAT_NORMAL: {
      auto vmap = std::make_shared<
          small_gicp::IncrementalVoxelMap<FlatContainerNormal>>(leaf_size);
      if (config->lru_horizon > 0) {
        vmap->lru_horizon = config->lru_horizon;
      }
      if (config->lru_clear_cycle > 0) {
        vmap->lru_clear_cycle = config->lru_clear_cycle;
      }
      wrapper->voxelmap = vmap;
      break;
    }
    case SMALL_GICP_VOXEL_FLAT_COV: {
      auto vmap =
          std::make_shared<small_gicp::IncrementalVoxelMap<FlatContainerCov>>(
              leaf_size);
      if (config->lru_horizon > 0) {
        vmap->lru_horizon = config->lru_horizon;
      }
      if (config->lru_clear_cycle > 0) {
        vmap->lru_clear_cycle = config->lru_clear_cycle;
      }
      wrapper->voxelmap = vmap;
      break;
    }
    case SMALL_GICP_VOXEL_FLAT_NORMAL_COV: {
      auto vmap = std::make_shared<
          small_gicp::IncrementalVoxelMap<FlatContainerNormalCov>>(leaf_size);
      if (config->lru_horizon > 0) {
        vmap->lru_horizon = config->lru_horizon;
      }
      if (config->lru_clear_cycle > 0) {
        vmap->lru_clear_cycle = config->lru_clear_cycle;
      }
      wrapper->voxelmap = vmap;
      break;
    }
    case SMALL_GICP_VOXEL_GAUSSIAN: {
      auto vmap =
          std::make_shared<small_gicp::IncrementalVoxelMap<GaussianVoxel>>(
              leaf_size);
      if (config->lru_horizon > 0) {
        vmap->lru_horizon = config->lru_horizon;
      }
      if (config->lru_clear_cycle > 0) {
        vmap->lru_clear_cycle = config->lru_clear_cycle;
      }
      wrapper->voxelmap = vmap;
      break;
    }
    default:
      return SMALL_GICP_ERROR_INVALID_ARGUMENT;
    }

    *voxelmap = wrapper.release();
    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_destroy(
    small_gicp_incremental_voxelmap_t *voxelmap) {
  if (!voxelmap) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  delete voxelmap;
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t small_gicp_incremental_voxelmap_size(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t *size) {
  if (!voxelmap || !size) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    std::visit([size](const auto &vmap) { *size = vmap->size(); },
               voxelmap->voxelmap);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_num_voxels(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t *num_voxels) {
  if (!voxelmap || !num_voxels) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    std::visit(
        [num_voxels](const auto &vmap) {
          *num_voxels = vmap->flat_voxels.size();
        },
        voxelmap->voxelmap);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_insert(
    small_gicp_incremental_voxelmap_t *voxelmap,
    const small_gicp_point_cloud_t *points) {
  if (!voxelmap || !points || !points->cloud) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

    std::visit(
        [&](const auto &vmap) { vmap->insert(*points->cloud, identity); },
        voxelmap->voxelmap);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_insert_with_transform(
    small_gicp_incremental_voxelmap_t *voxelmap,
    const small_gicp_point_cloud_t *points,
    const double *transformation_matrix) {
  if (!voxelmap || !points || !points->cloud || !transformation_matrix) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    // Convert 4x4 transformation matrix to Eigen::Isometry3d
    Eigen::Matrix4d transform_mat;
    for (int i = 0; i < 16; i++) {
      transform_mat(i / 4, i % 4) = transformation_matrix[i];
    }
    Eigen::Isometry3d transform(transform_mat);

    std::visit(
        [&](const auto &vmap) { vmap->insert(*points->cloud, transform); },
        voxelmap->voxelmap);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_insert_point(
    small_gicp_incremental_voxelmap_t *voxelmap, double x, double y, double z) {
  if (!voxelmap) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    // Create a temporary point cloud with single point
    small_gicp::PointCloud single_point;
    single_point.resize(1);
    single_point.point(0) = Eigen::Vector4d(x, y, z, 1.0);

    Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

    std::visit([&](const auto &vmap) { vmap->insert(single_point, identity); },
               voxelmap->voxelmap);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_insert_point_with_normal(
    small_gicp_incremental_voxelmap_t *voxelmap, double x, double y, double z,
    double nx, double ny, double nz) {
  if (!voxelmap) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  // Check if this voxelmap supports normals
  if (voxelmap->container_type != SMALL_GICP_VOXEL_FLAT_NORMAL &&
      voxelmap->container_type != SMALL_GICP_VOXEL_FLAT_NORMAL_COV) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    // Create a temporary point cloud with single point and normal
    small_gicp::PointCloud single_point;
    single_point.resize(1);
    single_point.point(0) = Eigen::Vector4d(x, y, z, 1.0);
    single_point.normal(0) = Eigen::Vector4d(nx, ny, nz, 0.0);

    Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

    std::visit([&](const auto &vmap) { vmap->insert(single_point, identity); },
               voxelmap->voxelmap);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_insert_point_with_covariance(
    small_gicp_incremental_voxelmap_t *voxelmap, double x, double y, double z,
    const double *covariance_matrix) {
  if (!voxelmap || !covariance_matrix) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  // Check if this voxelmap supports covariances
  if (voxelmap->container_type != SMALL_GICP_VOXEL_FLAT_COV &&
      voxelmap->container_type != SMALL_GICP_VOXEL_FLAT_NORMAL_COV) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    // Create a temporary point cloud with single point and covariance
    small_gicp::PointCloud single_point;
    single_point.resize(1);
    single_point.point(0) = Eigen::Vector4d(x, y, z, 1.0);

    Eigen::Matrix4d cov;
    for (int i = 0; i < 16; i++) {
      cov(i / 4, i % 4) = covariance_matrix[i];
    }
    single_point.cov(0) = cov;

    Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();

    std::visit([&](const auto &vmap) { vmap->insert(single_point, identity); },
               voxelmap->voxelmap);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_set_search_offsets(
    small_gicp_incremental_voxelmap_t *voxelmap,
    small_gicp_search_offset_pattern_t pattern) {
  if (!voxelmap) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    std::visit(
        [pattern](const auto &vmap) {
          vmap->set_search_offsets(static_cast<int>(pattern));
        },
        voxelmap->voxelmap);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_nearest_neighbor_search(
    const small_gicp_incremental_voxelmap_t *voxelmap, double query_x,
    double query_y, double query_z, size_t *index, double *distance) {
  if (!voxelmap || !index || !distance) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    Eigen::Vector4d query(query_x, query_y, query_z, 1.0);

    // Initialize with defaults in case no neighbor is found
    *index = SIZE_MAX;
    *distance = std::numeric_limits<double>::infinity();

    std::visit(
        [&](const auto &vmap) {
          double sq_dist = 0.0;
          size_t found = vmap->nearest_neighbor_search(query, index, &sq_dist);
          if (found > 0) {
            *distance = std::sqrt(sq_dist);
          } else {
            *index = SIZE_MAX;
            *distance = std::numeric_limits<double>::infinity();
          }
        },
        voxelmap->voxelmap);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_knn_search(
    const small_gicp_incremental_voxelmap_t *voxelmap, double query_x,
    double query_y, double query_z, int k, size_t *indices, double *distances) {
  if (!voxelmap || k <= 0 || !indices || !distances) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    Eigen::Vector4d query(query_x, query_y, query_z, 1.0);

    // Create temporary arrays for squared distances
    std::vector<double> sq_distances(k);

    std::visit(
        [&](const auto &vmap) {
          size_t found =
              vmap->knn_search(query, k, indices, sq_distances.data());

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
        voxelmap->voxelmap);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_lru_cleanup(
    small_gicp_incremental_voxelmap_t *voxelmap) {
  if (!voxelmap) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  // Note: IncrementalVoxelMap doesn't have a manual lru_cleanup method.
  // LRU cleanup is performed automatically during insert() operations
  // based on lru_clear_cycle. This function is provided for API completeness
  // but has no effect since cleanup is automatic.
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t small_gicp_incremental_voxelmap_get_voxel_info(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t voxel_index,
    small_gicp_voxel_info_t *info) {
  if (!voxelmap || !info) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    std::visit(
        [&](const auto &vmap) {
          if (voxel_index >= vmap->flat_voxels.size()) {
            throw std::out_of_range("Voxel index out of range");
          }

          const auto &voxel_pair = *vmap->flat_voxels[voxel_index];
          const auto &voxel_info = voxel_pair.first;
          const auto &voxel_contents = voxel_pair.second;

          // Calculate voxel center from coordinates using inv_leaf_size
          double leaf_size = 1.0 / vmap->inv_leaf_size;
          info->x = (voxel_info.coord[0] + 0.5) * leaf_size;
          info->y = (voxel_info.coord[1] + 0.5) * leaf_size;
          info->z = (voxel_info.coord[2] + 0.5) * leaf_size;
          info->coord_x = voxel_info.coord[0];
          info->coord_y = voxel_info.coord[1];
          info->coord_z = voxel_info.coord[2];
          info->lru_counter = voxel_info.lru;
          info->num_points = voxel_contents.size();
        },
        voxelmap->voxelmap);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_finalize(
    small_gicp_incremental_voxelmap_t *voxelmap) {
  if (!voxelmap) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    std::visit(
        [](const auto &vmap) {
          for (auto &voxel_pair : vmap->flat_voxels) {
            voxel_pair->second.finalize();
          }
        },
        voxelmap->voxelmap);

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_clear(
    small_gicp_incremental_voxelmap_t *voxelmap) {
  if (!voxelmap) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  // Note: IncrementalVoxelMap doesn't provide a public clear() method.
  // The voxels and flat_voxels are private members.
  // To clear the voxelmap, create a new one instead.
  return SMALL_GICP_NOT_IMPLEMENTED;
}

small_gicp_error_t small_gicp_incremental_voxelmap_get_voxel_coords(
    const small_gicp_incremental_voxelmap_t *voxelmap, double x, double y,
    double z, int *coord_x, int *coord_y, int *coord_z) {
  if (!voxelmap || !coord_x || !coord_y || !coord_z) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    double inv_leaf_size = 1.0 / voxelmap->leaf_size;
    *coord_x = static_cast<int>(std::floor(x * inv_leaf_size));
    *coord_y = static_cast<int>(std::floor(y * inv_leaf_size));
    *coord_z = static_cast<int>(std::floor(z * inv_leaf_size));

    return SMALL_GICP_SUCCESS;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}

small_gicp_error_t small_gicp_incremental_voxelmap_has_voxel(
    const small_gicp_incremental_voxelmap_t *voxelmap, int coord_x, int coord_y,
    int coord_z, bool *exists) {
  if (!voxelmap || !exists) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  // Note: IncrementalVoxelMap doesn't provide public access to the voxels hash
  // map. This function cannot be implemented without access to private members.
  *exists = false;
  return SMALL_GICP_NOT_IMPLEMENTED;
}

small_gicp_error_t small_gicp_incremental_voxelmap_get_voxel_index(
    const small_gicp_incremental_voxelmap_t *voxelmap, int coord_x, int coord_y,
    int coord_z, size_t *voxel_index) {
  if (!voxelmap || !voxel_index) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  // Note: IncrementalVoxelMap doesn't provide public access to the voxels hash
  // map. This function cannot be implemented without access to private members.
  *voxel_index = 0;
  return SMALL_GICP_NOT_IMPLEMENTED;
}

small_gicp_error_t small_gicp_incremental_voxelmap_update_config(
    small_gicp_incremental_voxelmap_t *voxelmap,
    const small_gicp_flat_container_config_t *config) {
  if (!voxelmap || !config) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  // Note: IncrementalVoxelMap's LRU parameters are set at construction time
  // and cannot be modified after creation. The lru_horizon and lru_clear_cycle
  // are private members with no public setters.
  voxelmap->config = *config;
  return SMALL_GICP_NOT_IMPLEMENTED;
}

small_gicp_error_t small_gicp_incremental_voxelmap_get_config(
    const small_gicp_incremental_voxelmap_t *voxelmap,
    small_gicp_flat_container_config_t *config) {
  if (!voxelmap || !config) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  *config = voxelmap->config;
  return SMALL_GICP_SUCCESS;
}

// Note: The following functions for extracting points/normals/covariances from
// specific voxels and Gaussian voxel statistics are complex to implement
// generically across all container types. They would require template
// specialization for each container type. For now, we provide basic stubs that
// return appropriate errors.

small_gicp_error_t small_gicp_incremental_voxelmap_get_voxel_points(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t voxel_index,
    double *points, size_t *num_points) {
  if (!voxelmap || !points || !num_points) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  // This function would require significant template specialization
  // to extract points from different container types
  return SMALL_GICP_NOT_IMPLEMENTED;
}

small_gicp_error_t small_gicp_incremental_voxelmap_get_voxel_normals(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t voxel_index,
    double *normals, size_t *num_normals) {
  if (!voxelmap || !normals || !num_normals) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  // Only applicable to containers with normals
  if (voxelmap->container_type != SMALL_GICP_VOXEL_FLAT_NORMAL &&
      voxelmap->container_type != SMALL_GICP_VOXEL_FLAT_NORMAL_COV) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  return SMALL_GICP_NOT_IMPLEMENTED;
}

small_gicp_error_t small_gicp_incremental_voxelmap_get_voxel_covariances(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t voxel_index,
    double *covariances, size_t *num_covariances) {
  if (!voxelmap || !covariances || !num_covariances) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  // Only applicable to containers with covariances
  if (voxelmap->container_type != SMALL_GICP_VOXEL_FLAT_COV &&
      voxelmap->container_type != SMALL_GICP_VOXEL_FLAT_NORMAL_COV) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  return SMALL_GICP_NOT_IMPLEMENTED;
}

small_gicp_error_t small_gicp_incremental_voxelmap_get_gaussian_voxel(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t voxel_index,
    double *mean, double *covariance, size_t *num_points) {
  if (!voxelmap || !mean || !covariance || !num_points) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  // Only applicable to Gaussian voxels
  if (voxelmap->container_type != SMALL_GICP_VOXEL_GAUSSIAN) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  try {
    if (auto gaussian_vmap =
            get_voxelmap_ptr<small_gicp::IncrementalVoxelMap<GaussianVoxel>>(
                voxelmap)) {
      if (voxel_index >= gaussian_vmap->flat_voxels.size()) {
        return SMALL_GICP_ERROR_INVALID_ARGUMENT;
      }

      const auto &voxel = gaussian_vmap->flat_voxels[voxel_index]->second;

      // Copy mean (4 components)
      for (int i = 0; i < 4; i++) {
        mean[i] = voxel.mean[i];
      }

      // Copy covariance (16 components)
      for (int i = 0; i < 16; i++) {
        covariance[i] = voxel.cov(i / 4, i % 4);
      }

      *num_points = voxel.num_points;

      return SMALL_GICP_SUCCESS;
    }

    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  } catch (const std::exception &e) {
    return SMALL_GICP_ERROR_EXCEPTION;
  }
}
