#ifndef SMALL_GICP_C_COMMON_H
#define SMALL_GICP_C_COMMON_H

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
  }                                                                            \
  return SMALL_GICP_SUCCESS;

#endif // SMALL_GICP_C_COMMON_H