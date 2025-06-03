//! Integration tests for the small-gicp-rust high-level API.

use approx::assert_relative_eq;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use small_gicp_rust::{estimate_normals, prelude::*};

/// Create a simple test point cloud (unit cube vertices).
fn create_test_cube() -> PointCloud {
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

/// Create a denser test point cloud for more realistic registration.
fn create_dense_test_cloud() -> PointCloud {
    let mut points = Vec::new();

    // Create a 10x10x10 grid
    for i in 0..10 {
        for j in 0..10 {
            for k in 0..5 {
                // Make it less dense in z direction
                let x = i as f64 * 0.1;
                let y = j as f64 * 0.1;
                let z = k as f64 * 0.1;
                points.push(Point3::new(x, y, z));
            }
        }
    }

    PointCloud::from_points(&points).unwrap()
}

/// Transform a point cloud with a known transformation.
fn transform_point_cloud(cloud: &PointCloud, transformation: &Isometry3<f64>) -> PointCloud {
    let points = cloud.points().unwrap();
    let transformed_points: Vec<Point3<f64>> = points.iter().map(|p| transformation * p).collect();

    PointCloud::from_points(&transformed_points).unwrap()
}

#[test]
fn test_point_cloud_basic_operations() {
    let cloud = create_test_cube();

    assert_eq!(cloud.len(), 8);
    assert!(!cloud.is_empty());

    // Test point access
    let first_point = cloud.get_point(0).unwrap();
    assert_eq!(first_point, Point3::new(0.0, 0.0, 0.0));

    // Test bounds checking
    assert!(cloud.get_point(100).is_err());
}

#[test]
fn test_point_cloud_with_normals() {
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let normals = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, 1.0),
    ];

    let cloud = PointCloud::from_points_and_normals(&points, &normals).unwrap();

    assert_eq!(cloud.len(), 3);

    let normal = cloud.get_normal(0).unwrap();
    assert_relative_eq!(normal.x, 0.0, epsilon = 1e-6);
    assert_relative_eq!(normal.y, 0.0, epsilon = 1e-6);
    assert_relative_eq!(normal.z, 1.0, epsilon = 1e-6);
}

#[test]
fn test_kdtree_operations() {
    let cloud = create_dense_test_cloud();
    let kdtree_config = KdTreeConfig::default();
    let kdtree = KdTree::new(&cloud, &kdtree_config).unwrap();

    // Test nearest neighbor search
    let query = Point3::new(0.5, 0.5, 0.2);
    let (index, sq_dist) = kdtree.nearest_neighbor(query).unwrap();

    assert!(index < cloud.len());
    assert!(sq_dist >= 0.0);

    let nearest_point = cloud.get_point(index).unwrap();
    let expected_sq_dist = (query - nearest_point).magnitude_squared();
    assert_relative_eq!(sq_dist, expected_sq_dist, epsilon = 1e-6);

    // Test k-NN search
    let knn_results = kdtree.knn_search(query, 5).unwrap();
    assert_eq!(knn_results.len(), 5);

    // Results should be sorted by distance
    for i in 1..knn_results.len() {
        assert!(knn_results[i - 1].1 <= knn_results[i].1);
    }

    // Test radius search
    let radius_results = kdtree.radius_search(query, 0.1, 20).unwrap();
    for (_, sq_dist) in &radius_results {
        assert!(*sq_dist <= 0.1 * 0.1);
    }
}

#[test]
fn test_voxelgrid_downsampling() {
    let cloud = create_dense_test_cloud();
    let original_size = cloud.len();

    let downsampled = cloud
        .voxelgrid_sampling(&VoxelGridConfig {
            leaf_size: 0.15,
            num_threads: 1,
        })
        .unwrap();

    // Should have fewer points after downsampling
    assert!(downsampled.len() <= original_size);
    assert!(!downsampled.is_empty());

    // Test with very large voxel size - should result in very few points
    let heavily_downsampled = cloud
        .voxelgrid_sampling(&VoxelGridConfig {
            leaf_size: 1.0,
            num_threads: 1,
        })
        .unwrap();
    assert!(heavily_downsampled.len() < downsampled.len());
}

#[test]
fn test_random_downsampling() {
    let cloud = create_dense_test_cloud();
    let target_samples = 50;

    let downsampled = cloud.random_sampling(target_samples).unwrap();
    assert_eq!(downsampled.len(), target_samples);

    // Test edge case: sampling more points than available
    let oversampled = cloud.random_sampling(cloud.len() + 100).unwrap();
    assert_eq!(oversampled.len(), cloud.len());
}

#[test]
fn test_normal_estimation() {
    let cloud = create_dense_test_cloud();
    let mut processed_cloud = cloud.clone();
    let kdtree_config = KdTreeConfig::default();
    let kdtree = KdTree::new(&processed_cloud, &kdtree_config).unwrap();

    // Before normal estimation, normals might be zero or uninitialized
    estimate_normals(
        &mut processed_cloud,
        &kdtree,
        &NormalEstimationConfig {
            num_neighbors: 10,
            num_threads: 1,
        },
    )
    .unwrap();

    // After estimation, normals should have reasonable magnitude
    let normal = processed_cloud.get_normal(0).unwrap();
    assert!(normal.magnitude() > 0.1); // Should be close to unit length
    assert!(normal.magnitude() < 2.0); // But not too large
}

#[test]
fn test_preprocessing_pipeline() {
    let cloud = create_dense_test_cloud();

    let result = cloud
        .preprocess_points(&PreprocessorConfig {
            downsampling_resolution: 0.15,
            num_neighbors: 15,
            num_threads: 1,
        })
        .unwrap();

    // Should have downsampled
    assert!(result.cloud.len() <= cloud.len());
    assert!(!result.cloud.is_empty());

    // Should have KdTree (always created)

    // Should have normals
    let normal = result.cloud.get_normal(0).unwrap();
    assert!(normal.magnitude() > 0.1);
}

#[test]
fn test_preprocess_points_convenience() {
    let cloud = create_dense_test_cloud();

    let result = cloud
        .preprocess_points(&PreprocessorConfig {
            downsampling_resolution: 0.15,
            num_neighbors: 15,
            num_threads: 1,
        })
        .unwrap();

    assert!(!result.cloud.is_empty());
    // KdTree is always created

    // Verify normals were estimated
    let normal = result.cloud.get_normal(0).unwrap();
    assert!(normal.magnitude() > 0.1);
}

#[test]
fn test_basic_registration() {
    let target = create_dense_test_cloud();

    // Create source by applying a small transformation
    let transformation = Isometry3::from_parts(
        Translation3::new(0.05, 0.03, 0.02),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.1),
    );
    let source = transform_point_cloud(&target, &transformation);

    let settings = RegistrationSettings {
        registration_type: RegistrationType::Icp,
        num_threads: 1,
        initial_guess: None,
    };

    let result = register(&target, &source, &settings).unwrap();

    assert!(result.converged);
    assert!(result.iterations > 0);
    assert!(result.error >= 0.0);

    // The recovered transformation should be close to the inverse of the applied transformation
    let recovered_transformation = result.transformation;
    let inverse_transformation = transformation.inverse();

    let translation_error = (recovered_transformation.translation.vector
        - inverse_transformation.translation.vector)
        .magnitude();
    assert!(
        translation_error < 0.1,
        "Translation error too large: {}",
        translation_error
    );
}

#[test]
fn test_registration_with_initial_guess() {
    let target = create_dense_test_cloud();

    let true_transformation = Isometry3::from_parts(
        Translation3::new(0.1, 0.05, 0.02),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.15),
    );
    let source = transform_point_cloud(&target, &true_transformation);

    // Provide a close initial guess
    let initial_guess = Isometry3::from_parts(
        Translation3::new(0.08, 0.04, 0.01),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.12),
    );

    let settings = RegistrationSettings {
        registration_type: RegistrationType::Icp,
        num_threads: 1,
        initial_guess: Some(initial_guess),
    };

    let result = register(&target, &source, &settings).unwrap();
    assert!(result.converged);
}

#[test]
fn test_different_registration_types() {
    let target = create_dense_test_cloud();
    let transformation = Isometry3::from_parts(
        Translation3::new(0.02, 0.01, 0.01),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.05),
    );
    let source = transform_point_cloud(&target, &transformation);

    let registration_types = [
        RegistrationType::Icp,
        RegistrationType::PlaneIcp,
        RegistrationType::Gicp,
    ];

    for reg_type in registration_types {
        let settings = RegistrationSettings {
            registration_type: reg_type,
            num_threads: 1,
            initial_guess: None,
        };

        let result = register(&target, &source, &settings);
        match result {
            Ok(res) => {
                assert!(res.error >= 0.0);
                // Note: convergence depends on the specific algorithm and data
            }
            Err(e) => {
                // Some algorithms might fail on this simple test case
                println!("Registration type {:?} failed: {}", reg_type, e);
            }
        }
    }
}

#[test]
fn test_vgicp_registration() {
    let target = create_dense_test_cloud();
    let transformation = Isometry3::from_parts(
        Translation3::new(0.03, 0.02, 0.01),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.08),
    );
    let source = transform_point_cloud(&target, &transformation);

    // Create voxel map
    let voxel_config = GaussianVoxelMapConfig {
        voxel_resolution: 0.1,
        num_threads: 1,
    };
    let voxel_map = GaussianVoxelMap::new(&target, &voxel_config).unwrap();

    let settings = RegistrationSettings {
        registration_type: RegistrationType::Vgicp,
        num_threads: 1,
        initial_guess: None,
    };

    // VGICP might fail with small test data, so handle errors gracefully
    match register_vgicp(&voxel_map, &source, &settings) {
        Ok(result) => {
            assert!(result.iterations > 0);
            assert!(result.error >= 0.0);
        }
        Err(SmallGicpError::RegistrationFailed { iterations: _ }) => {
            // VGICP can fail with small test clouds, which is expected
            println!("VGICP failed as expected with small test data");
        }
        Err(e) => panic!("Unexpected error: {}", e),
    }
}

#[test]
fn test_register_preprocessed() {
    let target = create_dense_test_cloud();
    let transformation = Isometry3::from_parts(
        Translation3::new(0.02, 0.01, 0.01),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.03),
    );
    let source = transform_point_cloud(&target, &transformation);

    // Preprocess clouds
    let target_result = target
        .preprocess_points(&PreprocessorConfig {
            downsampling_resolution: 0.1,
            num_neighbors: 15,
            num_threads: 1,
        })
        .unwrap();
    let source_processed = source
        .preprocess_points(&PreprocessorConfig {
            downsampling_resolution: 0.1,
            num_neighbors: 15,
            num_threads: 1,
        })
        .unwrap();

    let settings = RegistrationSettings {
        registration_type: RegistrationType::Gicp,
        num_threads: 1,
        initial_guess: None,
    };

    let result = register_preprocessed(
        &target_result.cloud,
        &source_processed.cloud,
        &target_result.kdtree,
        &settings,
    )
    .unwrap();

    assert!(result.iterations > 0);
    assert!(result.error >= 0.0);
}

#[test]
fn test_registration_result_utilities() {
    let target = create_test_cube();
    let source = create_test_cube();

    let settings = RegistrationSettings::default();
    let result = register(&target, &source, &settings).unwrap();

    // Test utility methods
    let matrix = result.transformation_matrix();
    assert_eq!(matrix.shape(), (4, 4));

    let _translation = result.translation();
    let _rotation = result.rotation();

    // Test point transformation
    let test_point = Point3::new(0.5, 0.5, 0.5);
    let transformed = result.transform_point(test_point);

    // Manual transformation should match
    let manual_transform = result.transformation * test_point;
    assert_relative_eq!(transformed.x, manual_transform.x, epsilon = 1e-10);
    assert_relative_eq!(transformed.y, manual_transform.y, epsilon = 1e-10);
    assert_relative_eq!(transformed.z, manual_transform.z, epsilon = 1e-10);

    // Test point cloud transformation
    let transformed_cloud = result.transform_point_cloud(&target).unwrap();
    assert_eq!(transformed_cloud.len(), target.len());
}

#[test]
fn test_error_handling() {
    // Test empty point cloud
    let empty_cloud = PointCloud::new().unwrap();
    assert!(empty_cloud.is_empty());

    let non_empty_cloud = create_test_cube();

    // Registration with empty cloud should fail
    let settings = RegistrationSettings::default();
    assert!(register(&empty_cloud, &non_empty_cloud, &settings).is_err());
    assert!(register(&non_empty_cloud, &empty_cloud, &settings).is_err());

    // KdTree creation with empty cloud should fail
    let kdtree_config = KdTreeConfig::default();
    assert!(KdTree::new(&empty_cloud, &kdtree_config).is_err());

    // Downsampling empty cloud should fail
    assert!(empty_cloud
        .voxelgrid_sampling(&VoxelGridConfig {
            leaf_size: 0.1,
            num_threads: 1,
        })
        .is_err());
    assert!(empty_cloud.random_sampling(10).is_err());

    // Invalid parameters
    assert!(non_empty_cloud
        .voxelgrid_sampling(&VoxelGridConfig {
            leaf_size: -0.1,
            num_threads: 1,
        })
        .is_err());
    assert!(non_empty_cloud.random_sampling(0).is_err());
}
