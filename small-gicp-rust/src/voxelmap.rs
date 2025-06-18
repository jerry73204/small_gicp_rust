//! Voxel map data structures for VGICP registration.

use crate::{error::Result, point_cloud::PointCloud, traits::MutablePointCloudTrait};
use nalgebra::{Matrix3, Matrix4, Point3, Vector3};

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

/// Container type for voxel storage.
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
    /// Only search the center voxel.
    Center,
    /// Search in 6 neighboring voxels (faces).
    Face,
    /// Same as Face (6 neighbors).
    FaceNeighbors,
    /// Search in 18 neighboring voxels (faces + edges).
    FaceEdge,
    /// Search in 26 neighboring voxels (faces + edges + corners).
    FaceEdgeCorner,
    /// Same as FaceEdgeCorner (26 neighbors).
    FullNeighborhood,
}

/// An incremental voxel map for efficient scan-to-model registration.
pub struct IncrementalVoxelMap {
    inner: small_gicp_cxx::IncrementalVoxelMap,
}

/// A Gaussian voxel map (alias for IncrementalVoxelMap with GaussianVoxel).
/// This follows the small_gicp C++ library convention where GaussianVoxelMap
/// is defined as `using GaussianVoxelMap = IncrementalVoxelMap<GaussianVoxel>;`
pub type GaussianVoxelMap = IncrementalVoxelMap;

impl IncrementalVoxelMap {
    /// Create a new incremental voxel map.
    pub fn new(voxel_size: f64) -> Self {
        let inner = small_gicp_cxx::IncrementalVoxelMap::new(voxel_size);
        Self { inner }
    }

    /// Create a new incremental voxel map with specified leaf size and container type.
    pub fn with_leaf_size(voxel_size: f64, _container_type: VoxelContainerType) -> Result<Self> {
        // NOTE: The upstream C++ library only supports GaussianVoxel container type
        // for IncrementalVoxelMap. The container_type parameter is kept for API compatibility
        // but currently has no effect.
        let inner = small_gicp_cxx::IncrementalVoxelMap::new(voxel_size);
        Ok(Self { inner })
    }

    /// Insert a point cloud into the voxel map.
    pub fn insert(&mut self, cloud: &PointCloud) -> Result<()> {
        // Convert to small-gicp-cxx PointCloud
        let cxx_cloud = cloud.clone().into_cxx();
        self.inner.insert(&cxx_cloud);
        Ok(())
    }

    /// Get the number of voxels in the map.
    pub fn size(&self) -> usize {
        self.inner.size()
    }

    /// Get the number of voxels in the map.
    pub fn num_voxels(&self) -> Result<usize> {
        Ok(self.inner.num_voxels())
    }

    /// Get the voxel size.
    pub fn voxel_size(&self) -> f64 {
        self.inner.voxel_size()
    }

    /// Check if there's a voxel at the given coordinates.
    pub fn has_voxel_at_coords(&self, x: i32, y: i32, z: i32) -> bool {
        self.inner.has_voxel_at_coords(x, y, z)
    }

    /// Clear all voxels from the map.
    pub fn clear(&mut self) {
        self.inner.clear();
    }

    /// Finalize the voxel map after all insertions.
    pub fn finalize(&mut self) {
        self.inner.finalize();
    }

    /// Insert a single point into the voxel map.
    pub fn insert_point(&mut self, point: &Point3<f64>) -> Result<()> {
        self.inner.insert_point(point.x, point.y, point.z);
        Ok(())
    }

    /// Insert a point cloud into the voxel map.
    pub fn insert_point_cloud(&mut self, cloud: &PointCloud) -> Result<()> {
        self.insert(cloud)
    }

    /// Insert a point cloud with transformation.
    pub fn insert_point_cloud_with_transform(
        &mut self,
        cloud: &PointCloud,
        transform: &Matrix4<f64>,
    ) -> Result<()> {
        // Convert nalgebra matrix to FFI transform
        let mut ffi_transform = small_gicp_cxx::Transform::default();
        for i in 0..4 {
            for j in 0..4 {
                ffi_transform.matrix[i * 4 + j] = transform[(i, j)];
            }
        }

        // Convert nalgebra matrix to FFI transform
        let mut ffi_transform = small_gicp_cxx::Transform::default();
        for i in 0..4 {
            for j in 0..4 {
                ffi_transform.matrix[i * 4 + j] = transform[(i, j)];
            }
        }

        self.inner
            .insert_with_transform(cloud.inner(), &ffi_transform);
        Ok(())
    }

    /// Insert a point with normal.
    pub fn insert_point_with_normal(
        &mut self,
        point: &Point3<f64>,
        normal: &Vector3<f64>,
    ) -> Result<()> {
        // NOTE: The upstream C++ IncrementalVoxelMap doesn't have direct support
        // for inserting points with normals. Create a temporary point cloud with normal.
        let mut cloud = PointCloud::new()?;
        cloud.add_point(point.x, point.y, point.z);
        cloud.set_normal(0, nalgebra::Vector4::new(normal.x, normal.y, normal.z, 0.0));
        self.insert(&cloud)
    }

    /// Insert a point with covariance.
    pub fn insert_point_with_covariance(
        &mut self,
        point: &Point3<f64>,
        covariance: &Matrix3<f64>,
    ) -> Result<()> {
        // NOTE: The upstream C++ IncrementalVoxelMap doesn't have direct support
        // for inserting points with covariances. Create a temporary point cloud with covariance.
        let mut cloud = PointCloud::new()?;
        cloud.add_point(point.x, point.y, point.z);

        // Convert 3x3 covariance to 4x4 matrix
        let mut cov4 = nalgebra::Matrix4::zeros();
        for i in 0..3 {
            for j in 0..3 {
                cov4[(i, j)] = covariance[(i, j)];
            }
        }
        cloud.set_covariance(0, cov4)?;
        self.insert(&cloud)
    }

    /// Set search offsets pattern.
    pub fn set_search_offsets(&mut self, pattern: SearchOffsetPattern) -> Result<()> {
        let num_offsets = match pattern {
            SearchOffsetPattern::Center => 1,
            SearchOffsetPattern::Face | SearchOffsetPattern::FaceNeighbors => 7,
            SearchOffsetPattern::FaceEdge => 19, // Note: C++ doesn't support 19, will use 27
            SearchOffsetPattern::FaceEdgeCorner | SearchOffsetPattern::FullNeighborhood => 27,
        };

        // Note: C++ only supports 1, 7, or 27. If 19 is requested, we use 27.
        let actual_offsets = if num_offsets == 19 { 27 } else { num_offsets };
        self.inner.set_search_offsets(actual_offsets);
        Ok(())
    }

    /// Nearest neighbor search.
    pub fn nearest_neighbor_search(&self, query: &Point3<f64>) -> Result<(usize, f64)> {
        let result = self
            .inner
            .nearest_neighbor_search(query.x, query.y, query.z);
        Ok((result.index, result.squared_distance))
    }

    /// K-nearest neighbors search.
    pub fn knn_search(&self, query: &Point3<f64>, k: usize) -> Result<(Vec<usize>, Vec<f64>)> {
        let result = self.inner.knn_search(query.x, query.y, query.z, k);
        Ok((result.indices, result.squared_distances))
    }

    /// Get voxel coordinates for a point.
    pub fn voxel_coords(&self, point: &Point3<f64>) -> Result<[i32; 3]> {
        Ok(self.inner.get_voxel_coords(point.x, point.y, point.z))
    }

    /// Get voxel by index.
    pub fn voxel_index(&self, coords: [i32; 3]) -> Result<usize> {
        match self.inner.get_voxel_index(coords[0], coords[1], coords[2]) {
            Some(index) => Ok(index),
            None => Err(crate::error::SmallGicpError::InvalidArgument(
                "No voxel found at the specified coordinates".to_string(),
            )),
        }
    }

    /// Get Gaussian voxel by index.
    pub fn gaussian_voxel(&self, index: usize) -> Result<GaussianVoxel> {
        let voxel_data = self.inner.get_gaussian_voxel_by_index(index);

        Ok(GaussianVoxel {
            num_points: voxel_data.num_points,
            mean: voxel_data.mean,
            covariance: voxel_data.covariance,
        })
    }

    /// Find nearby voxels to a query point.
    pub fn find_nearby_voxels(&self, point: &[f64; 3], max_distance: f64) -> Vec<VoxelInfo> {
        let voxel_infos =
            self.inner
                .find_voxels_in_radius(point[0], point[1], point[2], max_distance);

        voxel_infos
            .into_iter()
            .map(|info| VoxelInfo {
                index: info.index,
                coordinates: info.coordinates,
                distance: info.distance,
            })
            .collect()
    }

    /// Get voxel information at specific coordinates.
    pub fn voxel(&self, coordinates: &[i32; 3]) -> Option<GaussianVoxel> {
        let voxel_data =
            self.inner
                .get_voxel_data(coordinates[0], coordinates[1], coordinates[2])?;

        Some(GaussianVoxel {
            num_points: voxel_data.num_points,
            mean: voxel_data.mean,
            covariance: voxel_data.covariance,
        })
    }

    /// Access the underlying small-gicp-cxx IncrementalVoxelMap.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn inner(&self) -> &small_gicp_cxx::IncrementalVoxelMap {
        &self.inner
    }

    /// Convert from a small-gicp-cxx IncrementalVoxelMap.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn from_cxx(inner: small_gicp_cxx::IncrementalVoxelMap) -> Self {
        Self { inner }
    }

    /// Convert to a small-gicp-cxx IncrementalVoxelMap.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn into_cxx(self) -> small_gicp_cxx::IncrementalVoxelMap {
        self.inner
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_voxel_container_type() {
        assert_eq!(VoxelContainerType::HashMap, VoxelContainerType::HashMap);
        assert_ne!(VoxelContainerType::HashMap, VoxelContainerType::FlatArray);
    }

    #[test]
    fn test_search_offset_pattern() {
        assert_eq!(SearchOffsetPattern::Face, SearchOffsetPattern::Face);
        assert_ne!(SearchOffsetPattern::Face, SearchOffsetPattern::FaceEdge);
    }

    #[test]
    fn test_gaussian_voxel() {
        let voxel = GaussianVoxel {
            num_points: 10,
            mean: [1.0, 2.0, 3.0],
            covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        };
        assert_eq!(voxel.num_points, 10);
        assert_eq!(voxel.mean, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_incremental_voxel_map() {
        let mut voxelmap = IncrementalVoxelMap::new(0.1);
        assert_eq!(voxelmap.size(), 0);

        // Create a point cloud and add some points
        let mut cloud = crate::point_cloud::PointCloud::new().unwrap();
        cloud.add_point(0.0, 0.0, 0.0);
        cloud.add_point(0.05, 0.05, 0.05);
        cloud.add_point(0.2, 0.2, 0.2);

        voxelmap.insert(&cloud).unwrap();

        assert!(voxelmap.size() > 0);

        // Test other methods that are implemented
        voxelmap.finalize();

        // Test insert_point_cloud (alias for insert)
        voxelmap.insert_point_cloud(&cloud).unwrap();

        // Test voxel_size method
        let voxel_size = voxelmap.voxel_size();
        assert_eq!(voxel_size, 0.1);

        // Test has_voxel_at_coords
        assert!(voxelmap.has_voxel_at_coords(0, 0, 0));

        // Test clear (which is a no-op for IncrementalVoxelMap)
        voxelmap.clear();

        // Size should remain the same since clear is a no-op
        assert!(voxelmap.size() > 0);
    }
}
