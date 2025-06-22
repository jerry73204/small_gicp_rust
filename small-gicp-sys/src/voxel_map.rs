use crate::{
    ffi::ffi::{
        create_voxelmap, GaussianVoxelData, GaussianVoxelMap as FfiGaussianVoxelMap,
        KnnSearchResult, NearestNeighborResult, Transform, VoxelInfoData,
    },
    PointCloud,
};
use cxx::UniquePtr;
use std::pin::Pin;

/// Gaussian voxel map for efficient VGICP registration.
/// This is the unified interface that provides all IncrementalVoxelMap<GaussianVoxel> functionality.
pub struct GaussianVoxelMap {
    inner: UniquePtr<FfiGaussianVoxelMap>,
}

impl GaussianVoxelMap {
    /// Create a new voxel map with specified voxel size
    pub fn new(voxel_size: f64) -> Self {
        GaussianVoxelMap {
            inner: create_voxelmap(voxel_size),
        }
    }

    /// Insert a point cloud into the voxel map
    pub fn insert(&mut self, cloud: &PointCloud) {
        self.inner.pin_mut().insert(cloud.as_ffi());
    }

    /// Insert a point cloud with transformation matrix
    pub fn insert_with_transform(&mut self, cloud: &PointCloud, transform: &Transform) {
        self.inner
            .pin_mut()
            .insert_with_transform(cloud.as_ffi(), transform);
    }

    /// Insert a single point into the voxel map
    pub fn insert_point(&mut self, x: f64, y: f64, z: f64) {
        self.inner.pin_mut().insert_point(x, y, z);
    }

    /// Get the number of points in the voxel map
    pub fn size(&self) -> usize {
        self.inner.size()
    }

    /// Get the number of voxels in the map
    pub fn num_voxels(&self) -> usize {
        self.inner.get_num_voxels()
    }

    /// Get the voxel size (resolution)
    pub fn voxel_size(&self) -> f64 {
        self.inner.get_voxel_size()
    }

    /// Check if the voxel map is empty
    pub fn is_empty(&self) -> bool {
        self.size() == 0
    }

    /// Clear all voxels from the map
    pub fn clear(&mut self) {
        self.inner.pin_mut().clear_voxels();
    }

    /// Finalize the voxel map (compute final statistics)
    pub fn finalize(&mut self) {
        self.inner.pin_mut().finalize();
    }

    /// Check if a voxel exists at the given coordinates
    pub fn has_voxel_at(&self, x: i32, y: i32, z: i32) -> bool {
        self.inner.has_voxel_at_coords(x, y, z)
    }

    /// Find the nearest neighbor to a query point
    pub fn nearest_neighbor_search(&self, x: f64, y: f64, z: f64) -> NearestNeighborResult {
        self.inner.nearest_neighbor_search(x, y, z)
    }

    /// Find k nearest neighbors to a query point
    pub fn knn_search(&self, x: f64, y: f64, z: f64, k: usize) -> KnnSearchResult {
        self.inner.knn_search(x, y, z, k)
    }

    /// Get voxel data at specific coordinates
    pub fn voxel_data_at(&self, x: i32, y: i32, z: i32) -> GaussianVoxelData {
        self.inner.get_voxel_data(x, y, z)
    }

    /// Get Gaussian voxel data by index
    pub fn gaussian_voxel_by_index(&self, index: usize) -> GaussianVoxelData {
        self.inner.get_gaussian_voxel_by_index(index)
    }

    /// Get voxel coordinates for a world position
    pub fn voxel_coords(&self, x: f64, y: f64, z: f64) -> [i32; 3] {
        self.inner.get_voxel_coords(x, y, z)
    }

    /// Get voxel index from coordinates
    pub fn voxel_index(&self, x: i32, y: i32, z: i32) -> usize {
        self.inner.get_voxel_index(x, y, z)
    }

    /// Set the search offset pattern (1=center, 7=face neighbors, 27=full cube)
    pub fn set_search_offsets(&mut self, num_offsets: i32) {
        self.inner.pin_mut().set_search_offsets(num_offsets);
    }

    /// Find all voxels within a radius of a point
    pub fn find_voxels_in_radius(&self, x: f64, y: f64, z: f64, radius: f64) -> Vec<VoxelInfoData> {
        self.inner.find_voxels_in_radius(x, y, z, radius)
    }

    /// Set LRU horizon for cache management
    pub fn set_lru_horizon(&mut self, horizon: usize) {
        self.inner.pin_mut().set_lru_horizon(horizon);
    }

    /// Set LRU clear cycle for cache management
    pub fn set_lru_clear_cycle(&mut self, cycle: usize) {
        self.inner.pin_mut().set_lru_clear_cycle(cycle);
    }

    /// Get current LRU counter value
    pub fn lru_counter(&self) -> usize {
        self.inner.get_lru_counter()
    }

    /// Get internal FFI handle
    pub(crate) fn as_ffi(&self) -> &FfiGaussianVoxelMap {
        &self.inner
    }

    /// Get mutable internal FFI handle
    pub(crate) fn as_ffi_mut(&mut self) -> Pin<&mut FfiGaussianVoxelMap> {
        self.inner.pin_mut()
    }
}

/// Builder for voxel map with configuration options
pub struct GaussianVoxelMapBuilder {
    voxel_size: f64,
    search_offsets: Option<i32>,
    lru_horizon: Option<usize>,
    lru_clear_cycle: Option<usize>,
}

impl GaussianVoxelMapBuilder {
    /// Create a new voxel map builder
    pub fn new() -> Self {
        Self {
            voxel_size: 1.0,
            search_offsets: None,
            lru_horizon: None,
            lru_clear_cycle: None,
        }
    }

    /// Set the voxel size
    pub fn voxel_size(mut self, size: f64) -> Self {
        self.voxel_size = size;
        self
    }

    /// Set search offsets pattern (1=center, 7=face neighbors, 27=full cube)
    pub fn search_offsets(mut self, offsets: i32) -> Self {
        self.search_offsets = Some(offsets);
        self
    }

    /// Set LRU horizon for cache management
    pub fn lru_horizon(mut self, horizon: usize) -> Self {
        self.lru_horizon = Some(horizon);
        self
    }

    /// Set LRU clear cycle for cache management
    pub fn lru_clear_cycle(mut self, cycle: usize) -> Self {
        self.lru_clear_cycle = Some(cycle);
        self
    }

    /// Build the voxel map
    pub fn build(self) -> GaussianVoxelMap {
        let mut voxelmap = GaussianVoxelMap::new(self.voxel_size);

        if let Some(offsets) = self.search_offsets {
            voxelmap.set_search_offsets(offsets);
        }

        if let Some(horizon) = self.lru_horizon {
            voxelmap.set_lru_horizon(horizon);
        }

        if let Some(cycle) = self.lru_clear_cycle {
            voxelmap.set_lru_clear_cycle(cycle);
        }

        voxelmap
    }

    /// Build and insert a point cloud
    pub fn build_from_cloud(self, cloud: &PointCloud) -> GaussianVoxelMap {
        let mut voxelmap = self.build();
        voxelmap.insert(cloud);
        voxelmap
    }
}

impl Default for GaussianVoxelMapBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gaussian_voxel_map() {
        let mut cloud = PointCloud::new();
        cloud.add_point(0.0, 0.0, 0.0);
        cloud.add_point(1.0, 0.0, 0.0);
        cloud.add_point(0.0, 1.0, 0.0);

        let mut voxelmap = GaussianVoxelMap::new(0.5);
        assert_eq!(voxelmap.size(), 0);
        assert!(voxelmap.is_empty());

        voxelmap.insert(&cloud);
        assert!(voxelmap.size() > 0);
        assert!(!voxelmap.is_empty());
        assert!(voxelmap.num_voxels() > 0);
        assert_eq!(voxelmap.voxel_size(), 0.5);
    }

    #[test]
    fn test_gaussian_voxel_map_builder() {
        let cloud = PointCloud::from_points(&[(0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0)]);

        let voxelmap = GaussianVoxelMapBuilder::new()
            .voxel_size(0.1)
            .search_offsets(7)
            .lru_horizon(50)
            .build_from_cloud(&cloud);

        assert!(!voxelmap.is_empty());
        assert_eq!(voxelmap.voxel_size(), 0.1);
    }

    #[test]
    fn test_voxel_operations() {
        let mut cloud = PointCloud::new();
        cloud.add_point(0.0, 0.0, 0.0);
        cloud.add_point(0.1, 0.1, 0.1);

        let mut voxelmap = GaussianVoxelMap::new(1.0);
        voxelmap.insert(&cloud);

        // Test coordinate operations
        let coords = voxelmap.voxel_coords(0.0, 0.0, 0.0);
        assert_eq!(coords, [0, 0, 0]);

        // Test voxel existence
        assert!(voxelmap.has_voxel_at(0, 0, 0));

        // Test search configuration
        voxelmap.set_search_offsets(27); // Full cube search

        // Test clear
        voxelmap.clear();
        assert!(voxelmap.is_empty());
    }
}
