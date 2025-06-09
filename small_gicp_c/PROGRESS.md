# small_gicp C Wrapper API Coverage Progress

This document tracks the progress of wrapping the C++ small_gicp public API with C bindings.

## Overview

The small_gicp library is a header-only C++ library for efficient point cloud registration. This analysis compares the C++ public API with the current C wrapper implementation to identify coverage gaps.

**Total Coverage: ~75%** (Core functionality is well covered, missing advanced features and parallel variants)

## Module Analysis

### 1. Points Module

**Coverage: ~90%**

#### ✅ **Covered in C Wrapper**
- `PointCloud` struct basic operations
  - Creation, destruction, resize
  - Point get/set operations  
  - Normal get/set operations
  - Size queries
- Loading from arrays
- Basic traits functionality

#### ❌ **Missing from C Wrapper**
- Covariance get/set operations (`cov()` accessors)
- Constructor from Eigen vector arrays
- `empty()` check
- Direct access to internal vectors (points, normals, covs)
- Template support for different point types

#### **C++ API (from `points/point_cloud.hpp`)**
```cpp
struct PointCloud {
  size_t size() const;
  bool empty() const;
  void resize(size_t n);
  Eigen::Vector4d& point(size_t i);
  Eigen::Vector4d& normal(size_t i);
  Eigen::Matrix4d& cov(size_t i);
  // + const versions
};
```

### 2. ANN (Nearest Neighbor Search) Module

**Coverage: ~70%**

#### ✅ **Covered in C Wrapper**
- `KdTree` creation and destruction
- Basic k-nearest neighbor search
- Nearest neighbor search
- Thread control for tree construction
- `GaussianVoxelMap` creation and destruction

#### ❌ **Missing from C Wrapper**
- `UnsafeKdTree` (raw pointer version)
- Custom projection types (`AxisAlignedProjection`)
- `KdTreeBuilder` configuration (max_leaf_size, projection_setting)
- KNN search with custom settings (`KnnSetting`)
- Advanced voxelmap operations:
  - Point insertion
  - LRU-based voxel management
  - Search offsets configuration
  - Incremental updates
- Parallel variants (`kdtree_omp.hpp`, `kdtree_tbb.hpp`)
- Advanced voxel features from `incremental_voxelmap.hpp`

#### **C++ API (from `ann/kdtree.hpp`, `ann/gaussian_voxelmap.hpp`)**
```cpp
template<typename PointCloud, typename Projection = AxisAlignedProjection>
struct UnsafeKdTree {
  size_t nearest_neighbor_search(const Eigen::Vector4d& query, ...);
  size_t knn_search(const Eigen::Vector4d& query, int k, ...);
};

template<typename PointCloud>
struct KdTree {
  // Safe version with shared_ptr ownership
};

struct GaussianVoxel;
using GaussianVoxelMap = IncrementalVoxelMap<GaussianVoxel>;
```

### 3. Registration Module

**Coverage: ~80%**

#### ✅ **Covered in C Wrapper**
- Basic registration functions (ICP, Plane-ICP, GICP, VGICP)
- Registration with automatic preprocessing
- Registration with preprocessed point clouds
- VGICP registration with voxelmaps
- Advanced registration with full configuration
- Registration settings and results
- Termination criteria configuration
- Optimizer settings (Gauss-Newton, Levenberg-Marquardt)
- Correspondence rejection (distance-based)
- Robust kernels (Huber, Cauchy)
- DOF restrictions
- Extended results with information matrix

#### ❌ **Missing from C Wrapper**
- Direct access to `Registration` template class
- Custom factor types beyond built-in ones
- `RegistrationHelper` preprocessing functions
- Template-based registration with custom types
- Manual factor linearization
- Custom reduction strategies beyond thread control
- Integration with custom point cloud types

#### **C++ API (from `registration/registration.hpp`, `registration/registration_helper.hpp`)**
```cpp
template<typename PointFactor, typename Reduction, ...>
struct Registration {
  RegistrationResult align(const TargetPointCloud& target, ...);
};

// Helper functions
std::pair<PointCloud::Ptr, KdTree<PointCloud>::Ptr>
preprocess_points(const PointCloud& points, ...);

RegistrationResult align(const std::vector<Eigen::Vector<T,D,1>>& target, ...);
```

### 4. Utilities Module

**Coverage: ~65%**

#### ✅ **Covered in C Wrapper**
- Voxel grid downsampling
- Random downsampling
- Normal estimation
- Covariance estimation  
- Combined normal and covariance estimation

#### ❌ **Missing from C Wrapper**
- Parallel variants of all utilities:
  - `voxelgrid_sampling_omp()`, `voxelgrid_sampling_tbb()`
  - `estimate_normals_omp()`, `estimate_normals_tbb()`
  - `estimate_covariances_omp()`, `estimate_covariances_tbb()`
- Custom RNG for random sampling
- Template-based downsampling with custom point types
- Direct access to `NormalSetter`, `CovarianceSetter` classes
- Fine-grained local feature estimation control
- Sorting utilities (`sort_omp.hpp`, `sort_tbb.hpp`)

#### **C++ API (from `util/downsampling.hpp`, `util/normal_estimation.hpp`)**
```cpp
template<typename InputPointCloud, typename OutputPointCloud = InputPointCloud>
std::shared_ptr<OutputPointCloud> voxelgrid_sampling(const InputPointCloud& points, double leaf_size);

template<typename InputPointCloud, typename OutputPointCloud = InputPointCloud, typename RNG = std::mt19937>
std::shared_ptr<OutputPointCloud> random_sampling(const InputPointCloud& points, size_t num_samples, RNG& rng);

template<typename PointCloud>
void estimate_normals(PointCloud& cloud, int num_neighbors = 20);

template<typename PointCloud>
void estimate_covariances(PointCloud& cloud, int num_neighbors = 20);
```

### 5. Factors Module

**Coverage: ~50%**

#### ✅ **Covered in C Wrapper**
- ICP factor (implicitly through registration types)
- Plane-ICP factor (implicitly through registration types)
- GICP factor (implicitly through registration types)
- Robust kernel configuration (Huber, Cauchy)

#### ❌ **Missing from C Wrapper**
- Direct access to factor classes:
  - `ICPFactor`
  - `PointToPlaneICPFactor` 
  - `GICPFactor`
- `RobustFactor` template wrapper
- Custom factor development interface
- `GeneralFactor` for constraints
- Factor linearization interface
- Error evaluation methods

#### **C++ API (from `factors/*.hpp`)**
```cpp
struct ICPFactor {
  struct Setting {};
  bool linearize(...);
  double error(...);
  bool inlier() const;
};

struct GICPFactor {
  struct Setting {};
  bool linearize(...);
  double error(...);
  bool inlier() const;
};

template<typename Kernel, typename Factor>
struct RobustFactor {
  // Wraps any factor with robust kernel
};
```

### 6. I/O Module

**Coverage: ~40%**

#### ✅ **Covered in C Wrapper**
- PLY file loading (`small_gicp_load_ply`)
- PLY file saving (`small_gicp_save_ply`)

#### ❌ **Missing from C Wrapper**
- Support for other point cloud formats
- Advanced I/O options and configurations
- Stream-based I/O
- Metadata preservation during I/O

### 7. PCL Integration Module

**Coverage: ~0%**

#### ❌ **Missing from C Wrapper**
- All PCL integration features:
  - `RegistrationPCL` template class
  - PCL point type support
  - Drop-in replacement for PCL registration
  - PCL-compatible interfaces

#### **C++ API (from `pcl/*.hpp`)**
```cpp
template<typename PointSource, typename PointTarget>
class RegistrationPCL : public pcl::Registration<PointSource, PointTarget, float> {
  // PCL-compatible registration interface
};
```

### 8. Parallel Processing Support

**Coverage: ~20%**

#### ✅ **Covered in C Wrapper**
- Thread count configuration for basic operations
- Parallel reduction strategy selection

#### ❌ **Missing from C Wrapper**
- OpenMP-specific variants (`*_omp.hpp`)
- TBB-specific variants (`*_tbb.hpp`)
- Custom parallel execution policies
- Fine-grained parallelism control
- NUMA-aware processing

## Priority Recommendations

### High Priority (Essential for Completeness)
1. **Add covariance support to point cloud API** - Required for GICP functionality
2. **Expose parallel utility variants** - Performance critical
3. **Add missing kdtree configuration options** - Important for fine-tuning
4. **Implement incremental voxelmap operations** - Needed for advanced VGICP usage

### Medium Priority (Enhanced Functionality)  
1. **Expose factor classes directly** - For advanced users needing custom registration
2. **Add template support for custom point types** - Flexibility for integration
3. **Implement more I/O formats** - Broader compatibility
4. **Add custom RNG support for sampling** - Reproducibility control

### Low Priority (Advanced Features)
1. **PCL integration module** - Only needed if PCL compatibility required
2. **Custom projection types** - Specialized use cases
3. **Manual factor linearization interface** - Research/development use cases

## Implementation Effort Estimates

- **High Priority items**: ~2-3 weeks of development
- **Medium Priority items**: ~3-4 weeks of development  
- **Low Priority items**: ~4-6 weeks of development

The current C wrapper provides solid coverage of the core functionality needed for most point cloud registration tasks. The missing features are primarily advanced configuration options, parallel processing variants, and specialized use cases.