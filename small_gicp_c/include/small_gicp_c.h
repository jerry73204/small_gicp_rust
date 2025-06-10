#ifndef SMALL_GICP_C_H
#define SMALL_GICP_C_H

#ifdef __cplusplus
extern "C" {
#endif

// Include common types first
#include <small_gicp_c/types.h>

// Include all module headers
#include <small_gicp_c/ann/gaussian_voxelmap.h>
#include <small_gicp_c/ann/incremental_voxelmap.h>
#include <small_gicp_c/ann/kdtree.h>
#include <small_gicp_c/io/io.h>
#include <small_gicp_c/points/point_cloud.h>
#include <small_gicp_c/registration/registration.h>
#include <small_gicp_c/util/downsampling.h>
#include <small_gicp_c/util/local_features.h>
#include <small_gicp_c/util/normal_estimation.h>
#include <small_gicp_c/util/sorting.h>

// Version and utility functions
#include <small_gicp_c/util/version.h>

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_H
