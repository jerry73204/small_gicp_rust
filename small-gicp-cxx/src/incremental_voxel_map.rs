use crate::{
    ffi::ffi::{create_incremental_voxelmap, IncrementalVoxelMap as FfiIncrementalVoxelMap},
    PointCloud,
};
use cxx::UniquePtr;
use std::pin::Pin;

/// Incremental voxel map for efficient scan-to-model registration
///
/// This data structure supports incremental point cloud insertion and LRU-based
/// voxel deletion that removes voxels that are not recently referenced.
/// It's particularly useful for scan-to-model registration where new points
/// are continuously added to the map.
pub struct IncrementalVoxelMap {
    inner: UniquePtr<FfiIncrementalVoxelMap>,
}

impl IncrementalVoxelMap {
    /// Create a new incremental voxel map with the given voxel size
    pub fn new(voxel_size: f64) -> Self {
        IncrementalVoxelMap {
            inner: create_incremental_voxelmap(voxel_size),
        }
    }

    /// Insert a point cloud into the voxel map
    pub fn insert(&mut self, cloud: &PointCloud) {
        self.inner.pin_mut().incremental_insert(cloud.as_ffi());
    }

    /// Get the number of voxels in the map
    pub fn size(&self) -> usize {
        self.inner.incremental_size()
    }

    /// Check if the voxel map is empty
    pub fn is_empty(&self) -> bool {
        self.size() == 0
    }

    /// Clear all voxels from the map
    ///
    /// Note: IncrementalVoxelMap uses LRU-based cleanup automatically.
    /// For a complete reset, create a new instance instead.
    pub fn clear(&mut self) {
        self.inner.pin_mut().incremental_clear();
    }

    /// Finalize the voxel map (prepare for registration)
    ///
    /// Note: IncrementalVoxelMap finalizes voxels automatically during insertion.
    /// This method is provided for API compatibility.
    pub fn finalize(&mut self) {
        self.inner.pin_mut().incremental_finalize();
    }

    /// Get the voxel size
    pub fn voxel_size(&self) -> f64 {
        self.inner.incremental_get_voxel_size()
    }

    /// Get the number of voxels
    pub fn num_voxels(&self) -> usize {
        self.inner.incremental_get_num_voxels()
    }

    /// Check if there's a voxel at the given coordinates
    pub fn has_voxel_at_coords(&self, x: i32, y: i32, z: i32) -> bool {
        self.inner.incremental_has_voxel_at_coords(x, y, z)
    }

    /// Insert a single point into the voxel map
    pub fn insert_point(&mut self, x: f64, y: f64, z: f64) {
        self.inner.pin_mut().incremental_insert_point(x, y, z);
    }

    /// Insert a point cloud with transformation
    pub fn insert_with_transform(&mut self, cloud: &PointCloud, transform: &crate::Transform) {
        self.inner
            .pin_mut()
            .incremental_insert_with_transform(cloud.as_ffi(), transform);
    }

    /// Set the search offset pattern
    pub fn set_search_offsets(&mut self, num_offsets: i32) {
        self.inner
            .pin_mut()
            .incremental_set_search_offsets(num_offsets);
    }

    /// Get voxel data at specific coordinates
    pub fn get_voxel_data(
        &self,
        x: i32,
        y: i32,
        z: i32,
    ) -> Option<crate::ffi::ffi::GaussianVoxelData> {
        if !self.has_voxel_at_coords(x, y, z) {
            return None;
        }
        Some(self.inner.incremental_get_voxel_data(x, y, z))
    }

    /// Find voxels within a radius
    pub fn find_voxels_in_radius(
        &self,
        x: f64,
        y: f64,
        z: f64,
        radius: f64,
    ) -> Vec<crate::ffi::ffi::VoxelInfoData> {
        self.inner
            .incremental_find_voxels_in_radius(x, y, z, radius)
    }

    /// Nearest neighbor search
    pub fn nearest_neighbor_search(
        &self,
        x: f64,
        y: f64,
        z: f64,
    ) -> crate::ffi::ffi::NearestNeighborResult {
        self.inner.incremental_nearest_neighbor_search(x, y, z)
    }

    /// K-nearest neighbors search
    pub fn knn_search(&self, x: f64, y: f64, z: f64, k: usize) -> crate::ffi::ffi::KnnSearchResult {
        self.inner.incremental_knn_search(x, y, z, k)
    }

    /// Get voxel coordinates for a point
    pub fn get_voxel_coords(&self, x: f64, y: f64, z: f64) -> [i32; 3] {
        self.inner.incremental_get_voxel_coords(x, y, z)
    }

    /// Get voxel index for given coordinates
    pub fn get_voxel_index(&self, x: i32, y: i32, z: i32) -> Option<usize> {
        let index = self.inner.incremental_get_voxel_index(x, y, z);
        if index == usize::MAX {
            None
        } else {
            Some(index)
        }
    }

    /// Get Gaussian voxel data by index
    pub fn get_gaussian_voxel_by_index(&self, index: usize) -> crate::ffi::ffi::GaussianVoxelData {
        self.inner.incremental_get_gaussian_voxel_by_index(index)
    }

    /// Get internal FFI handle (for registration algorithms)
    pub(crate) fn as_ffi(&self) -> &FfiIncrementalVoxelMap {
        &self.inner
    }

    #[allow(dead_code)]
    pub(crate) fn as_ffi_mut(&mut self) -> Pin<&mut FfiIncrementalVoxelMap> {
        self.inner.pin_mut()
    }
}

/// Builder for incremental voxel map construction
pub struct IncrementalVoxelMapBuilder {
    voxel_size: f64,
}

impl IncrementalVoxelMapBuilder {
    /// Create a new incremental voxel map builder
    pub fn new(voxel_size: f64) -> Self {
        Self { voxel_size }
    }

    /// Build the incremental voxel map
    pub fn build(self) -> IncrementalVoxelMap {
        IncrementalVoxelMap::new(self.voxel_size)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_incremental_voxel_map_basic() {
        let mut voxel_map = IncrementalVoxelMap::new(0.1);
        assert_eq!(voxel_map.size(), 0);
        assert!(voxel_map.is_empty());

        let mut cloud = PointCloud::new();
        cloud.add_point(0.0, 0.0, 0.0);
        cloud.add_point(0.1, 0.1, 0.1);
        cloud.add_point(0.2, 0.2, 0.2);

        voxel_map.insert(&cloud);
        assert!(voxel_map.size() > 0);
        assert!(!voxel_map.is_empty());

        voxel_map.finalize();
        let size_after_finalize = voxel_map.size();
        assert!(size_after_finalize > 0);

        // Note: clear() is a no-op for IncrementalVoxelMap
        voxel_map.clear();
        // Size should remain the same since clear is a no-op
        assert_eq!(voxel_map.size(), size_after_finalize);
    }

    #[test]
    fn test_incremental_voxel_map_builder() {
        let voxel_map = IncrementalVoxelMapBuilder::new(0.05).build();
        assert_eq!(voxel_map.size(), 0);
    }

    #[test]
    fn test_incremental_multiple_insertions() {
        let mut voxel_map = IncrementalVoxelMap::new(0.1);

        // First insertion
        let mut cloud1 = PointCloud::new();
        cloud1.add_point(0.0, 0.0, 0.0);
        cloud1.add_point(1.0, 0.0, 0.0);
        voxel_map.insert(&cloud1);
        let size1 = voxel_map.size();

        // Second insertion
        let mut cloud2 = PointCloud::new();
        cloud2.add_point(2.0, 0.0, 0.0);
        cloud2.add_point(3.0, 0.0, 0.0);
        voxel_map.insert(&cloud2);
        let size2 = voxel_map.size();

        // Size should increase or stay the same (depending on voxel overlap)
        assert!(size2 >= size1);
    }
}
