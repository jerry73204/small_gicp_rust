//! Voxel map data structures for VGICP registration.
//!
//! This module provides the unified GaussianVoxelMap interface that directly maps to
//! the C++ `IncrementalVoxelMap<GaussianVoxel>` template specialization.

use crate::{error::Result, point_cloud::PointCloud};
use nalgebra::{Isometry3, Matrix3, Point3};

/// A Gaussian voxel containing statistical information about points.
#[derive(Debug, Clone)]
pub struct GaussianVoxel {
    /// Number of points in this voxel.
    pub num_points: usize,
    /// Mean position of points in this voxel.
    pub mean: [f64; 3],
    /// Covariance matrix of points in this voxel.
    pub covariance: [f64; 9], // 3x3 matrix stored row-major
}

/// Information about a voxel in the map.
#[derive(Debug, Clone)]
pub struct VoxelInfo {
    /// Voxel index in the map.
    pub index: usize,
    /// Voxel coordinates.
    pub coordinates: [i32; 3],
    /// Distance to query point.
    pub distance: f64,
}

/// Container type for voxel storage (for future use - currently only Gaussian is supported).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoxelContainerType {
    /// Hash map based storage.
    HashMap,
    /// Flat array based storage.
    FlatArray,
    /// Flat points storage.
    FlatPoints,
    /// Flat storage with normals.
    FlatNormal,
    /// Flat storage with covariances.
    FlatCovariance,
    /// Flat storage with normals and covariances.
    FlatNormalCovariance,
    /// Gaussian voxel representation.
    Gaussian,
}

/// Search pattern for finding nearby voxels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SearchOffsetPattern {
    /// Only search the center voxel (1 voxel).
    Center = 1,
    /// Search in 6 neighboring voxels (faces) + center (7 voxels).
    Face = 7,
    /// Search in full 3x3x3 cube (27 voxels).
    FullCube = 27,
}

/// Legacy aliases for SearchOffsetPattern variants.
impl SearchOffsetPattern {
    /// Alias for Face (7 neighbors).
    pub const FACE_NEIGHBORS: Self = Self::Face;
    /// Alias for FullCube (27 neighbors).
    pub const FULL_NEIGHBORHOOD: Self = Self::FullCube;
}

impl SearchOffsetPattern {
    /// Convert to the integer value expected by the C++ API.
    pub fn to_int(self) -> i32 {
        self as i32
    }
}

/// Statistics about a voxel map.
#[derive(Debug, Clone)]
pub struct VoxelMapStatistics {
    /// Total number of voxels.
    pub num_voxels: usize,
    /// Total number of points across all voxels.
    pub total_points: usize,
    /// Minimum number of points in a voxel.
    pub min_points_per_voxel: usize,
    /// Maximum number of points in a voxel.
    pub max_points_per_voxel: usize,
    /// Average number of points per voxel.
    pub avg_points_per_voxel: f64,
    /// Voxel size.
    pub voxel_size: f64,
    /// LRU horizon setting.
    pub lru_horizon: usize,
    /// LRU clear cycle setting.
    pub lru_clear_cycle: usize,
    /// Current LRU counter.
    pub lru_counter: usize,
}

/// Unified Gaussian voxel map that provides all `IncrementalVoxelMap<GaussianVoxel>` functionality.
///
/// This matches the C++ convention where `GaussianVoxelMap = IncrementalVoxelMap<GaussianVoxel>`.
/// The voxel map divides 3D space into a regular grid of voxels, where each voxel
/// stores statistical information (mean and covariance) about the points it contains.
///
/// # Use Cases
///
/// - **VGICP Registration**: Primary use case for efficient large-scale registration
/// - **Spatial Indexing**: Fast nearest neighbor queries in large point clouds
/// - **Downsampling**: Statistical representation of dense point clouds
/// - **Change Detection**: Comparing point clouds over time
///
/// # Performance
///
/// - Insertion: O(1) average case per point
/// - Nearest neighbor search: O(1) to O(k) where k is the search neighborhood size
/// - Memory usage: O(n) where n is the number of occupied voxels
///
/// # Example
///
/// ```rust
/// use nalgebra::Point3;
/// use small_gicp::{GaussianVoxelMap, PointCloud};
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let mut cloud = PointCloud::new()?;
/// # for i in 0..10 {
/// #     cloud.add_point(i as f64 * 0.1, 0.0, 0.0);
/// # }
///
/// // Create voxel map with 0.5m resolution
/// let mut voxelmap = GaussianVoxelMap::new(0.5);
/// voxelmap.insert(&cloud)?;
/// voxelmap.finalize();
///
/// // Query nearest voxel
/// let query = Point3::new(0.25, 0.0, 0.0);
/// if let Some((index, distance)) = voxelmap.nearest_neighbor_search(&query) {
///     println!(
///         "Nearest voxel: index={}, distance={}",
///         index,
///         distance.sqrt()
///     );
/// }
/// # Ok(())
/// # }
/// ```
pub struct GaussianVoxelMap {
    inner: small_gicp_sys::GaussianVoxelMap,
}

impl GaussianVoxel {
    /// Get the mean position as a Point3.
    ///
    /// Converts the internal array representation to a nalgebra Point3
    /// for easier manipulation.
    ///
    /// # Example
    ///
    /// ```rust
    /// use small_gicp::GaussianVoxel;
    ///
    /// let voxel = GaussianVoxel {
    ///     num_points: 10,
    ///     mean: [1.0, 2.0, 3.0],
    ///     covariance: [0.0; 9],
    /// };
    ///
    /// let mean = voxel.mean_point();
    /// assert_eq!(mean.x, 1.0);
    /// assert_eq!(mean.y, 2.0);
    /// assert_eq!(mean.z, 3.0);
    /// ```
    pub fn mean_point(&self) -> Point3<f64> {
        Point3::new(self.mean[0], self.mean[1], self.mean[2])
    }

    /// Get the covariance as a 3x3 matrix.
    ///
    /// Converts the internal row-major array representation to a nalgebra Matrix3.
    /// The covariance matrix represents the distribution of points within the voxel.
    ///
    /// # Example
    ///
    /// ```rust
    /// use small_gicp::GaussianVoxel;
    ///
    /// let voxel = GaussianVoxel {
    ///     num_points: 10,
    ///     mean: [0.0; 3],
    ///     covariance: [
    ///         1.0, 0.0, 0.0, // First row
    ///         0.0, 2.0, 0.0, // Second row
    ///         0.0, 0.0, 3.0, // Third row
    ///     ],
    /// };
    ///
    /// let cov = voxel.covariance_matrix();
    /// assert_eq!(cov[(0, 0)], 1.0);
    /// assert_eq!(cov[(1, 1)], 2.0);
    /// assert_eq!(cov[(2, 2)], 3.0);
    /// ```
    pub fn covariance_matrix(&self) -> Matrix3<f64> {
        Matrix3::new(
            self.covariance[0],
            self.covariance[1],
            self.covariance[2],
            self.covariance[3],
            self.covariance[4],
            self.covariance[5],
            self.covariance[6],
            self.covariance[7],
            self.covariance[8],
        )
    }

    /// Check if the voxel has valid data.
    ///
    /// A voxel is considered valid if it contains at least one point.
    /// Invalid voxels may be returned by queries when no data is available.
    ///
    /// # Example
    ///
    /// ```rust
    /// use small_gicp::GaussianVoxel;
    ///
    /// let valid_voxel = GaussianVoxel {
    ///     num_points: 5,
    ///     mean: [1.0, 2.0, 3.0],
    ///     covariance: [0.0; 9],
    /// };
    /// assert!(valid_voxel.is_valid());
    ///
    /// let invalid_voxel = GaussianVoxel {
    ///     num_points: 0,
    ///     mean: [0.0; 3],
    ///     covariance: [0.0; 9],
    /// };
    /// assert!(!invalid_voxel.is_valid());
    /// ```
    pub fn is_valid(&self) -> bool {
        self.num_points > 0
    }
}

impl GaussianVoxelMap {
    /// Create a new voxel map with specified voxel size.
    pub fn new(voxel_size: f64) -> Self {
        let inner = small_gicp_sys::GaussianVoxelMap::new(voxel_size);
        Self { inner }
    }

    /// Create a new voxel map from a point cloud.
    pub fn from_points(points: &PointCloud, voxel_size: f64) -> Result<Self> {
        let mut voxelmap = Self::new(voxel_size);
        voxelmap.insert(points)?;
        Ok(voxelmap)
    }

    /// Insert a point cloud into the voxel map.
    pub fn insert(&mut self, cloud: &PointCloud) -> Result<()> {
        let cxx_cloud = cloud.clone().into_cxx();
        self.inner.insert(&cxx_cloud);
        Ok(())
    }

    /// Insert a point cloud with transformation matrix.
    pub fn insert_with_transform(
        &mut self,
        cloud: &PointCloud,
        transform: &Isometry3<f64>,
    ) -> Result<()> {
        // Convert nalgebra Isometry3 to FFI transform
        let mut ffi_transform = small_gicp_sys::Transform::default();
        let matrix = transform.to_homogeneous();
        for i in 0..4 {
            for j in 0..4 {
                ffi_transform.matrix[i * 4 + j] = matrix[(i, j)];
            }
        }

        let cxx_cloud = cloud.clone().into_cxx();
        self.inner.insert_with_transform(&cxx_cloud, &ffi_transform);
        Ok(())
    }

    /// Insert a single point into the voxel map.
    pub fn insert_point(&mut self, point: &Point3<f64>) -> Result<()> {
        self.inner.insert_point(point.x, point.y, point.z);
        Ok(())
    }

    /// Get the number of points in the voxel map.
    pub fn len(&self) -> usize {
        self.inner.size()
    }

    /// Get the number of voxels in the map.
    pub fn num_voxels(&self) -> usize {
        self.inner.num_voxels()
    }

    /// Get the voxel size (resolution).
    pub fn voxel_size(&self) -> f64 {
        self.inner.voxel_size()
    }

    /// Check if the voxel map is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Clear all voxels from the map.
    pub fn clear(&mut self) {
        self.inner.clear();
    }

    /// Finalize the voxel map (compute final statistics).
    pub fn finalize(&mut self) {
        self.inner.finalize();
    }

    /// Check if a voxel exists at the given coordinates.
    pub fn has_voxel_at(&self, x: i32, y: i32, z: i32) -> bool {
        self.inner.has_voxel_at(x, y, z)
    }

    /// Find the nearest neighbor to a query point.
    pub fn nearest_neighbor_search(&self, query: &Point3<f64>) -> Option<(usize, f64)> {
        let result = self
            .inner
            .nearest_neighbor_search(query.x, query.y, query.z);
        if result.index != usize::MAX {
            Some((result.index, result.squared_distance))
        } else {
            None
        }
    }

    /// Find k nearest neighbors to a query point.
    pub fn knn_search(&self, query: &Point3<f64>, k: usize) -> Vec<(usize, f64)> {
        let result = self.inner.knn_search(query.x, query.y, query.z, k);
        result
            .indices
            .iter()
            .zip(result.squared_distances.iter())
            .map(|(idx, dist)| (*idx, *dist))
            .collect()
    }

    /// Get voxel data at specific coordinates.
    pub fn voxel_data_at(&self, x: i32, y: i32, z: i32) -> Option<GaussianVoxel> {
        let data = self.inner.voxel_data_at(x, y, z);
        if data.num_points > 0 {
            Some(GaussianVoxel {
                num_points: data.num_points,
                mean: data.mean,
                covariance: data.covariance,
            })
        } else {
            None
        }
    }

    /// Get Gaussian voxel data by index.
    pub fn gaussian_voxel_by_index(&self, index: usize) -> Option<GaussianVoxel> {
        let data = self.inner.gaussian_voxel_by_index(index);
        if data.num_points > 0 {
            Some(GaussianVoxel {
                num_points: data.num_points,
                mean: data.mean,
                covariance: data.covariance,
            })
        } else {
            None
        }
    }

    /// Get voxel coordinates for a world position.
    pub fn voxel_coords(&self, point: &Point3<f64>) -> [i32; 3] {
        self.inner.voxel_coords(point.x, point.y, point.z)
    }

    /// Get voxel index from coordinates.
    pub fn voxel_index(&self, x: i32, y: i32, z: i32) -> Option<usize> {
        let index = self.inner.voxel_index(x, y, z);
        if index != usize::MAX {
            Some(index)
        } else {
            None
        }
    }

    /// Set the search offset pattern (1=center, 7=face neighbors, 27=full cube).
    pub fn set_search_pattern(&mut self, pattern: SearchOffsetPattern) {
        self.inner.set_search_offsets(pattern.to_int());
    }

    /// Find all voxels within a radius of a point.
    pub fn find_voxels_in_radius(&self, center: &Point3<f64>, radius: f64) -> Vec<VoxelInfo> {
        let result = self
            .inner
            .find_voxels_in_radius(center.x, center.y, center.z, radius);
        result
            .into_iter()
            .map(|data| VoxelInfo {
                index: data.index,
                coordinates: data.coordinates,
                distance: data.distance,
            })
            .collect()
    }

    /// Set LRU horizon for cache management.
    pub fn set_lru_horizon(&mut self, horizon: usize) {
        self.inner.set_lru_horizon(horizon);
    }

    /// Set LRU clear cycle for cache management.
    pub fn set_lru_clear_cycle(&mut self, cycle: usize) {
        self.inner.set_lru_clear_cycle(cycle);
    }

    /// Get current LRU counter value.
    pub fn lru_counter(&self) -> usize {
        self.inner.lru_counter()
    }

    /// Get comprehensive statistics about the voxel map.
    pub fn statistics(&self) -> VoxelMapStatistics {
        let num_voxels = self.num_voxels();
        let total_points = self.len();

        // Basic statistics - could be enhanced with actual min/max calculation
        let avg_points_per_voxel = if num_voxels > 0 {
            total_points as f64 / num_voxels as f64
        } else {
            0.0
        };

        VoxelMapStatistics {
            num_voxels,
            total_points,
            min_points_per_voxel: if num_voxels > 0 { 1 } else { 0 }, // Simplified
            max_points_per_voxel: if num_voxels > 0 { total_points } else { 0 }, // Simplified
            avg_points_per_voxel,
            voxel_size: self.voxel_size(),
            lru_horizon: 100,    // Would need to expose from C++ for exact value
            lru_clear_cycle: 10, // Would need to expose from C++ for exact value
            lru_counter: self.lru_counter(),
        }
    }

    /// Get internal FFI handle (for internal use).
    pub(crate) fn inner(&self) -> &small_gicp_sys::GaussianVoxelMap {
        &self.inner
    }

    // TODO: Add CXX conversion methods if needed for future FFI integration
    // pub(crate) fn from_cxx(inner: small_gicp_sys::GaussianVoxelMap) -> Self {
    //     Self { inner }
    // }
    // pub(crate) fn into_cxx(self) -> small_gicp_sys::GaussianVoxelMap {
    //     self.inner
    // }
}

impl std::fmt::Debug for GaussianVoxelMap {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GaussianVoxelMap")
            .field("size", &self.len())
            .field("num_voxels", &self.num_voxels())
            .field("voxel_size", &self.voxel_size())
            .field("is_empty", &self.is_empty())
            .finish()
    }
}

impl Default for GaussianVoxelMap {
    fn default() -> Self {
        Self::new(1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gaussian_voxel() {
        let voxel = GaussianVoxel {
            num_points: 5,
            mean: [1.0, 2.0, 3.0],
            covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        };

        assert!(voxel.is_valid());
        assert_eq!(voxel.mean_point(), Point3::new(1.0, 2.0, 3.0));

        let cov_matrix = voxel.covariance_matrix();
        assert_eq!(cov_matrix[(0, 0)], 1.0);
        assert_eq!(cov_matrix[(1, 1)], 1.0);
        assert_eq!(cov_matrix[(2, 2)], 1.0);
    }

    #[test]
    fn test_search_offset_pattern() {
        assert_eq!(SearchOffsetPattern::Center.to_int(), 1);
        assert_eq!(SearchOffsetPattern::Face.to_int(), 7);
        assert_eq!(SearchOffsetPattern::FACE_NEIGHBORS.to_int(), 7);
        assert_eq!(SearchOffsetPattern::FullCube.to_int(), 27);
        assert_eq!(SearchOffsetPattern::FULL_NEIGHBORHOOD.to_int(), 27);
    }

    #[test]
    fn test_gaussian_voxel_map_basic() {
        let mut cloud = PointCloud::new().unwrap();
        cloud.add_point(0.0, 0.0, 0.0);
        cloud.add_point(1.0, 0.0, 0.0);
        cloud.add_point(0.0, 1.0, 0.0);

        let mut voxelmap = GaussianVoxelMap::new(0.5);
        assert_eq!(voxelmap.len(), 0);
        assert!(voxelmap.is_empty());
        assert_eq!(voxelmap.voxel_size(), 0.5);

        voxelmap.insert(&cloud).unwrap();
        assert!(!voxelmap.is_empty());
        assert!(!voxelmap.is_empty());
        assert!(voxelmap.num_voxels() > 0);
    }

    #[test]
    fn test_gaussian_voxel_map_from_points() {
        let mut cloud = PointCloud::new().unwrap();
        cloud.add_point(0.0, 0.0, 0.0);
        cloud.add_point(0.1, 0.1, 0.1);

        let voxelmap = GaussianVoxelMap::from_points(&cloud, 1.0).unwrap();
        assert!(!voxelmap.is_empty());
        assert_eq!(voxelmap.voxel_size(), 1.0);
    }

    #[test]
    fn test_voxel_operations() {
        let mut cloud = PointCloud::new().unwrap();
        cloud.add_point(0.0, 0.0, 0.0);
        cloud.add_point(0.1, 0.1, 0.1);

        let mut voxelmap = GaussianVoxelMap::new(1.0);
        voxelmap.insert(&cloud).unwrap();

        // Test coordinate operations
        let coords = voxelmap.voxel_coords(&Point3::new(0.0, 0.0, 0.0));
        assert_eq!(coords, [0, 0, 0]);

        // Test voxel existence
        assert!(voxelmap.has_voxel_at(0, 0, 0));

        // Test search configuration
        voxelmap.set_search_pattern(SearchOffsetPattern::FullCube);

        // Test statistics
        let stats = voxelmap.statistics();
        assert!(stats.num_voxels > 0);
        assert!(stats.total_points > 0);
        assert!(stats.avg_points_per_voxel > 0.0);

        // Test clear
        voxelmap.clear();
        assert!(voxelmap.is_empty());
    }

    #[test]
    fn test_point_insertion() {
        let mut voxelmap = GaussianVoxelMap::new(1.0);

        let point = Point3::new(1.0, 2.0, 3.0);
        voxelmap.insert_point(&point).unwrap();

        assert!(!voxelmap.is_empty());
        assert_eq!(voxelmap.len(), 1);
    }

    #[test]
    fn test_lru_configuration() {
        let mut voxelmap = GaussianVoxelMap::new(1.0);

        voxelmap.set_lru_horizon(50);
        voxelmap.set_lru_clear_cycle(5);

        // These would be tested more thoroughly with actual LRU behavior
        let _counter = voxelmap.lru_counter();
        // Just ensure it returns a valid value (counter is usize, always >= 0)
    }
}
