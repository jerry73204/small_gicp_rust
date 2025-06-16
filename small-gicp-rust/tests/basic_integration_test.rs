//! Basic integration tests for the small-gicp-rust preprocessing functionality.

use nalgebra::Point3;
use small_gicp_rust::{config::VoxelGridConfig, prelude::*};

/// Create a simple test point cloud.
fn create_test_cloud() -> PointCloud {
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
    ];
    PointCloud::from_points(&points).unwrap()
}

#[test]
fn test_voxel_downsampling() {
    let cloud = create_test_cloud();
    let config = VoxelGridConfig {
        leaf_size: 0.5,
        backend: DownsamplingBackend::Default,
        num_threads: 1,
    };

    let downsampled = cloud.voxelgrid_sampling(&config).unwrap();
    assert!(downsampled.len() <= cloud.len());
    assert!(downsampled.len() > 0);
}

#[test]
fn test_random_downsampling() {
    let cloud = create_test_cloud();

    let downsampled = cloud.random_sampling(4).unwrap();
    assert_eq!(downsampled.len(), 4);
}

#[test]
fn test_preprocessing_functionality() {
    let mut cloud = create_test_cloud();

    // Test that we can estimate normals
    cloud.estimate_normals(5).unwrap();

    // Test that we can estimate covariances
    cloud.estimate_covariances(5).unwrap();

    // Test that the cloud has the expected data
    assert!(cloud.has_normals());
    assert!(cloud.has_covariances());
}
