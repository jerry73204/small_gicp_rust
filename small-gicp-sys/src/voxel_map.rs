use crate::{
    ffi::ffi::{create_voxelmap, GaussianVoxelMap as FfiGaussianVoxelMap},
    PointCloud,
};
use cxx::UniquePtr;
use std::pin::Pin;

/// Gaussian voxel map for efficient VGICP registration
pub struct VoxelMap {
    inner: UniquePtr<FfiGaussianVoxelMap>,
}

impl VoxelMap {
    /// Create a new voxel map with specified voxel size
    pub fn new(voxel_size: f64) -> Self {
        VoxelMap {
            inner: create_voxelmap(voxel_size),
        }
    }

    /// Insert a point cloud into the voxel map
    pub fn insert(&mut self, cloud: &PointCloud) {
        self.inner.pin_mut().insert(cloud.as_ffi());
    }

    /// Get the number of voxels in the map
    pub fn size(&self) -> usize {
        self.inner.size()
    }

    /// Check if the voxel map is empty
    pub fn is_empty(&self) -> bool {
        self.size() == 0
    }

    /// Get internal FFI handle
    pub(crate) fn as_ffi(&self) -> &FfiGaussianVoxelMap {
        &self.inner
    }

    #[allow(dead_code)]
    pub(crate) fn as_ffi_mut(&mut self) -> Pin<&mut FfiGaussianVoxelMap> {
        self.inner.pin_mut()
    }
}

/// Builder for voxel map with configuration options
pub struct VoxelMapBuilder {
    voxel_size: f64,
}

impl VoxelMapBuilder {
    /// Create a new voxel map builder
    pub fn new() -> Self {
        Self { voxel_size: 1.0 }
    }

    /// Set the voxel size
    pub fn voxel_size(mut self, size: f64) -> Self {
        self.voxel_size = size;
        self
    }

    /// Build the voxel map
    pub fn build(self) -> VoxelMap {
        VoxelMap::new(self.voxel_size)
    }

    /// Build and insert a point cloud
    pub fn build_from_cloud(self, cloud: &PointCloud) -> VoxelMap {
        let mut voxelmap = self.build();
        voxelmap.insert(cloud);
        voxelmap
    }
}

impl Default for VoxelMapBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_voxel_map() {
        let mut cloud = PointCloud::new();
        cloud.add_point(0.0, 0.0, 0.0);
        cloud.add_point(1.0, 0.0, 0.0);
        cloud.add_point(0.0, 1.0, 0.0);

        let mut voxelmap = VoxelMap::new(0.5);
        assert_eq!(voxelmap.size(), 0);
        assert!(voxelmap.is_empty());

        voxelmap.insert(&cloud);
        assert!(voxelmap.size() > 0);
        assert!(!voxelmap.is_empty());
    }

    #[test]
    fn test_voxel_map_builder() {
        let cloud = PointCloud::from_points(&[(0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0)]);

        let voxelmap = VoxelMapBuilder::new()
            .voxel_size(0.1)
            .build_from_cloud(&cloud);

        assert!(!voxelmap.is_empty());
    }
}
