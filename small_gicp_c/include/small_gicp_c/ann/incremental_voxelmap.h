#ifndef SMALL_GICP_C_ANN_INCREMENTAL_VOXELMAP_H
#define SMALL_GICP_C_ANN_INCREMENTAL_VOXELMAP_H

#include <small_gicp_c/points/point_cloud.h>
#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
typedef struct small_gicp_incremental_voxelmap
    small_gicp_incremental_voxelmap_t;

// Voxel container types
typedef enum {
  SMALL_GICP_VOXEL_FLAT_POINTS = 0,     // Points only
  SMALL_GICP_VOXEL_FLAT_NORMAL = 1,     // Points + normals
  SMALL_GICP_VOXEL_FLAT_COV = 2,        // Points + covariances
  SMALL_GICP_VOXEL_FLAT_NORMAL_COV = 3, // Points + normals + covariances
  SMALL_GICP_VOXEL_GAUSSIAN = 4         // Gaussian statistics
} small_gicp_voxel_container_type_t;

// Search offset patterns
typedef enum {
  SMALL_GICP_SEARCH_OFFSET_1 = 1,  // Center voxel only
  SMALL_GICP_SEARCH_OFFSET_7 = 7,  // Center + 6 face neighbors
  SMALL_GICP_SEARCH_OFFSET_27 = 27 // Center + 26 neighbors (full 3x3x3)
} small_gicp_search_offset_pattern_t;

// Configuration structure for flat containers
typedef struct {
  double
      min_sq_dist_in_cell; // Minimum squared distance between points in a cell
  int max_num_points_in_cell; // Maximum number of points per cell
  double lru_horizon;         // LRU horizon (0 = disabled)
  int lru_clear_cycle;        // LRU clear cycle (0 = disabled)
} small_gicp_flat_container_config_t;

// Voxel information structure
typedef struct {
  double x, y, z;                // Voxel center coordinates
  size_t num_points;             // Number of points in voxel
  size_t lru_counter;            // LRU access counter
  int coord_x, coord_y, coord_z; // Voxel grid coordinates
} small_gicp_voxel_info_t;

// Create incremental voxelmap
small_gicp_error_t small_gicp_incremental_voxelmap_create(
    double leaf_size, small_gicp_voxel_container_type_t container_type,
    small_gicp_incremental_voxelmap_t **voxelmap);

// Create incremental voxelmap with flat container configuration
small_gicp_error_t small_gicp_incremental_voxelmap_create_with_config(
    double leaf_size, small_gicp_voxel_container_type_t container_type,
    const small_gicp_flat_container_config_t *config,
    small_gicp_incremental_voxelmap_t **voxelmap);

// Destroy incremental voxelmap
small_gicp_error_t small_gicp_incremental_voxelmap_destroy(
    small_gicp_incremental_voxelmap_t *voxelmap);

// Get voxelmap size (number of points)
small_gicp_error_t small_gicp_incremental_voxelmap_size(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t *size);

// Get number of voxels
small_gicp_error_t small_gicp_incremental_voxelmap_num_voxels(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t *num_voxels);

// Insert point cloud into voxelmap
small_gicp_error_t small_gicp_incremental_voxelmap_insert(
    small_gicp_incremental_voxelmap_t *voxelmap,
    const small_gicp_point_cloud_t *points);

// Insert point cloud with transformation
small_gicp_error_t small_gicp_incremental_voxelmap_insert_with_transform(
    small_gicp_incremental_voxelmap_t *voxelmap,
    const small_gicp_point_cloud_t *points,
    const double *transformation_matrix);

// Insert single point
small_gicp_error_t small_gicp_incremental_voxelmap_insert_point(
    small_gicp_incremental_voxelmap_t *voxelmap, double x, double y, double z);

// Insert single point with normal
small_gicp_error_t small_gicp_incremental_voxelmap_insert_point_with_normal(
    small_gicp_incremental_voxelmap_t *voxelmap, double x, double y, double z,
    double nx, double ny, double nz);

// Insert single point with covariance
small_gicp_error_t small_gicp_incremental_voxelmap_insert_point_with_covariance(
    small_gicp_incremental_voxelmap_t *voxelmap, double x, double y, double z,
    const double *covariance_matrix);

// Set search offset pattern
small_gicp_error_t small_gicp_incremental_voxelmap_set_search_offsets(
    small_gicp_incremental_voxelmap_t *voxelmap,
    small_gicp_search_offset_pattern_t pattern);

// Nearest neighbor search
small_gicp_error_t small_gicp_incremental_voxelmap_nearest_neighbor_search(
    const small_gicp_incremental_voxelmap_t *voxelmap, double query_x,
    double query_y, double query_z, size_t *index, double *distance);

// K-nearest neighbor search
small_gicp_error_t small_gicp_incremental_voxelmap_knn_search(
    const small_gicp_incremental_voxelmap_t *voxelmap, double query_x,
    double query_y, double query_z, int k, size_t *indices, double *distances);

// Trigger LRU cleanup manually
small_gicp_error_t small_gicp_incremental_voxelmap_lru_cleanup(
    small_gicp_incremental_voxelmap_t *voxelmap);

// Get voxel information
small_gicp_error_t small_gicp_incremental_voxelmap_get_voxel_info(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t voxel_index,
    small_gicp_voxel_info_t *info);

// Extract points from a specific voxel
small_gicp_error_t small_gicp_incremental_voxelmap_get_voxel_points(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t voxel_index,
    double *points, size_t *num_points);

// Extract normals from a specific voxel (if available)
small_gicp_error_t small_gicp_incremental_voxelmap_get_voxel_normals(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t voxel_index,
    double *normals, size_t *num_normals);

// Extract covariances from a specific voxel (if available)
small_gicp_error_t small_gicp_incremental_voxelmap_get_voxel_covariances(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t voxel_index,
    double *covariances, size_t *num_covariances);

// Get Gaussian voxel statistics (for Gaussian voxels only)
small_gicp_error_t small_gicp_incremental_voxelmap_get_gaussian_voxel(
    const small_gicp_incremental_voxelmap_t *voxelmap, size_t voxel_index,
    double *mean, double *covariance, size_t *num_points);

// Update flat container configuration
small_gicp_error_t small_gicp_incremental_voxelmap_update_config(
    small_gicp_incremental_voxelmap_t *voxelmap,
    const small_gicp_flat_container_config_t *config);

// Get current flat container configuration
small_gicp_error_t small_gicp_incremental_voxelmap_get_config(
    const small_gicp_incremental_voxelmap_t *voxelmap,
    small_gicp_flat_container_config_t *config);

// Clear all voxels
small_gicp_error_t small_gicp_incremental_voxelmap_clear(
    small_gicp_incremental_voxelmap_t *voxelmap);

// Get voxel grid coordinates for a point
small_gicp_error_t small_gicp_incremental_voxelmap_get_voxel_coords(
    const small_gicp_incremental_voxelmap_t *voxelmap, double x, double y,
    double z, int *coord_x, int *coord_y, int *coord_z);

// Check if voxel exists at coordinates
small_gicp_error_t small_gicp_incremental_voxelmap_has_voxel(
    const small_gicp_incremental_voxelmap_t *voxelmap, int coord_x, int coord_y,
    int coord_z, bool *exists);

// Get voxel index from coordinates
small_gicp_error_t small_gicp_incremental_voxelmap_get_voxel_index(
    const small_gicp_incremental_voxelmap_t *voxelmap, int coord_x, int coord_y,
    int coord_z, size_t *voxel_index);

// Finalize all voxels (for Gaussian voxels)
small_gicp_error_t small_gicp_incremental_voxelmap_finalize(
    small_gicp_incremental_voxelmap_t *voxelmap);

// Create default flat container configuration
small_gicp_error_t small_gicp_create_default_flat_container_config(
    small_gicp_flat_container_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_ANN_INCREMENTAL_VOXELMAP_H
