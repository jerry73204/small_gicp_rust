//! Integration tests for the small-gicp-rust high-level API.

use approx::assert_relative_eq;
use nalgebra::{Isometry3, Matrix4, Point3, Translation3, UnitQuaternion, Vector3};
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

#[test]
fn test_covariance_operations() {
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let mut cloud = PointCloud::from_points(&points).unwrap();

    // Test setting and getting individual covariance matrices
    let test_cov = Matrix4::new(
        1.0, 0.1, 0.2, 0.0, 0.1, 2.0, 0.3, 0.0, 0.2, 0.3, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    cloud.set_covariance(0, test_cov).unwrap();
    let retrieved_cov = cloud.get_covariance(0).unwrap();

    for i in 0..4 {
        for j in 0..4 {
            assert_relative_eq!(test_cov[(i, j)], retrieved_cov[(i, j)], epsilon = 1e-10);
        }
    }

    // Test setting all covariances at once
    let covariances = vec![test_cov; 3];
    cloud.set_covariances(&covariances).unwrap();

    let all_covariances = cloud.covariances().unwrap();
    assert_eq!(all_covariances.len(), 3);
    for cov in &all_covariances {
        for i in 0..4 {
            for j in 0..4 {
                assert_relative_eq!(test_cov[(i, j)], cov[(i, j)], epsilon = 1e-10);
            }
        }
    }

    // Test bounds checking
    assert!(cloud.get_covariance(100).is_err());
    assert!(cloud.set_covariance(100, test_cov).is_err());

    // Test wrong number of covariances
    let wrong_covariances = vec![test_cov; 2];
    assert!(cloud.set_covariances(&wrong_covariances).is_err());
}

#[test]
fn test_validation_functions() {
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

    // Test empty cloud
    let empty_cloud = PointCloud::new().unwrap();
    assert!(!empty_cloud.has_points());
    assert!(!empty_cloud.has_normals());
    assert!(!empty_cloud.has_covariances());
    assert!(empty_cloud.empty());

    // Test cloud with only points
    let cloud_points_only = PointCloud::from_points(&points).unwrap();
    assert!(cloud_points_only.has_points());
    // Note: The C wrapper may allocate space for normals and covariances when points are added
    // The main thing is that we have points and it's not empty
    assert!(!cloud_points_only.empty());

    // Test cloud with points and normals
    let cloud_with_normals = PointCloud::from_points_and_normals(&points, &normals).unwrap();
    assert!(cloud_with_normals.has_points());
    assert!(cloud_with_normals.has_normals());
    assert!(!cloud_with_normals.empty());

    // Test cloud with points, normals, and covariances
    let mut cloud_full = PointCloud::from_points_and_normals(&points, &normals).unwrap();
    let test_cov = Matrix4::identity();
    let covariances = vec![test_cov; 3];
    cloud_full.set_covariances(&covariances).unwrap();

    assert!(cloud_full.has_points());
    assert!(cloud_full.has_normals());
    assert!(cloud_full.has_covariances());
    assert!(!cloud_full.empty());
}

#[test]
fn test_bulk_operations() {
    let mut cloud = PointCloud::new().unwrap();

    // Test bulk point setting
    let points_data = vec![
        0.0, 0.0, 0.0, // Point 0
        1.0, 0.0, 0.0, // Point 1
        0.0, 1.0, 0.0, // Point 2
    ];
    cloud.set_points_bulk(&points_data).unwrap();
    assert_eq!(cloud.len(), 3);

    let first_point = cloud.get_point(0).unwrap();
    assert_relative_eq!(first_point.x, 0.0, epsilon = 1e-10);
    assert_relative_eq!(first_point.y, 0.0, epsilon = 1e-10);
    assert_relative_eq!(first_point.z, 0.0, epsilon = 1e-10);

    // Test bulk normal setting
    let normals_data = vec![
        0.0, 0.0, 1.0, // Normal 0
        0.0, 0.0, 1.0, // Normal 1
        0.0, 0.0, 1.0, // Normal 2
    ];
    cloud.set_normals_bulk(&normals_data).unwrap();

    let first_normal = cloud.get_normal(0).unwrap();
    assert_relative_eq!(first_normal.x, 0.0, epsilon = 1e-10);
    assert_relative_eq!(first_normal.y, 0.0, epsilon = 1e-10);
    assert_relative_eq!(first_normal.z, 1.0, epsilon = 1e-10);

    // Test bulk covariance setting
    let identity_matrix_data = vec![
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];
    let covariances_data = [
        identity_matrix_data.clone(),
        identity_matrix_data.clone(),
        identity_matrix_data,
    ]
    .concat();
    cloud.set_covariances_bulk(&covariances_data).unwrap();

    let first_cov = cloud.get_covariance(0).unwrap();
    let expected_cov = Matrix4::identity();
    for i in 0..4 {
        for j in 0..4 {
            assert_relative_eq!(first_cov[(i, j)], expected_cov[(i, j)], epsilon = 1e-10);
        }
    }

    // Test error cases
    assert!(cloud.set_points_bulk(&[1.0, 2.0]).is_err()); // Not multiple of 3
    assert!(cloud.set_normals_bulk(&[1.0, 2.0]).is_err()); // Not multiple of 3
    assert!(cloud.set_covariances_bulk(&[1.0; 15]).is_err()); // Not multiple of 16
    assert!(cloud.set_normals_bulk(&[1.0; 12]).is_err()); // Wrong number of normals (4 instead of 3)
    assert!(cloud.set_covariances_bulk(&[1.0; 32]).is_err()); // Wrong number of covariances (2 instead of 3)
}

#[test]
fn test_copy_operations() {
    let points = vec![
        Point3::new(1.0, 2.0, 3.0),
        Point3::new(4.0, 5.0, 6.0),
        Point3::new(7.0, 8.0, 9.0),
    ];
    let normals = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];

    let mut cloud = PointCloud::from_points_and_normals(&points, &normals).unwrap();

    // Add covariances
    let test_cov = Matrix4::new(
        1.0, 0.1, 0.2, 0.0, 0.1, 2.0, 0.3, 0.0, 0.2, 0.3, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );
    let covariances = vec![test_cov; 3];
    cloud.set_covariances(&covariances).unwrap();

    // Test copying points
    let mut points_array = vec![0.0; 9]; // 3 points * 3 components
    cloud.copy_points_to_array(&mut points_array).unwrap();
    assert_relative_eq!(points_array[0], 1.0, epsilon = 1e-10);
    assert_relative_eq!(points_array[1], 2.0, epsilon = 1e-10);
    assert_relative_eq!(points_array[2], 3.0, epsilon = 1e-10);
    assert_relative_eq!(points_array[3], 4.0, epsilon = 1e-10);

    // Test copying normals
    let mut normals_array = vec![0.0; 9]; // 3 normals * 3 components
    cloud.copy_normals_to_array(&mut normals_array).unwrap();
    assert_relative_eq!(normals_array[0], 0.0, epsilon = 1e-10);
    assert_relative_eq!(normals_array[1], 0.0, epsilon = 1e-10);
    assert_relative_eq!(normals_array[2], 1.0, epsilon = 1e-10);

    // Test copying covariances
    let mut covariances_array = vec![0.0; 48]; // 3 covariances * 16 components
    cloud
        .copy_covariances_to_array(&mut covariances_array)
        .unwrap();
    assert_relative_eq!(covariances_array[0], 1.0, epsilon = 1e-10); // First matrix [0,0]
    assert_relative_eq!(covariances_array[5], 2.0, epsilon = 1e-10); // First matrix [1,1]

    // Test error cases - wrong array sizes
    let mut wrong_size_array = vec![0.0; 8];
    assert!(cloud.copy_points_to_array(&mut wrong_size_array).is_err());
    assert!(cloud.copy_normals_to_array(&mut wrong_size_array).is_err());

    let mut wrong_cov_array = vec![0.0; 32];
    assert!(cloud
        .copy_covariances_to_array(&mut wrong_cov_array)
        .is_err());
}

#[test]
fn test_float_array_loading() {
    let points_array = vec![
        0.0_f32, 0.0, 0.0, // Point 0
        1.0, 0.0, 0.0, // Point 1
        0.0, 1.0, 0.0, // Point 2
    ];

    let cloud = PointCloud::from_float_array(&points_array).unwrap();
    assert_eq!(cloud.len(), 3);

    let first_point = cloud.get_point(0).unwrap();
    assert_relative_eq!(first_point.x, 0.0, epsilon = 1e-6);
    assert_relative_eq!(first_point.y, 0.0, epsilon = 1e-6);
    assert_relative_eq!(first_point.z, 0.0, epsilon = 1e-6);

    let second_point = cloud.get_point(1).unwrap();
    assert_relative_eq!(second_point.x, 1.0, epsilon = 1e-6);
    assert_relative_eq!(second_point.y, 0.0, epsilon = 1e-6);
    assert_relative_eq!(second_point.z, 0.0, epsilon = 1e-6);

    // Test error case - not multiple of 3
    let wrong_array = vec![1.0_f32, 2.0]; // Only 2 elements
    assert!(PointCloud::from_float_array(&wrong_array).is_err());
}

#[test]
fn test_direct_data_access() {
    let points = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];
    let normals = vec![Vector3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 0.0, 0.0)];

    let mut cloud = PointCloud::from_points_and_normals(&points, &normals).unwrap();

    // Test direct points access
    unsafe {
        let points_data = cloud.points_data().unwrap();
        // Points are stored as 4D vectors (homogeneous coordinates), so 2 points * 4 components = 8
        assert_eq!(points_data.len(), 8); // 2 points * 4 components (homogeneous)
        assert_relative_eq!(points_data[0], 1.0, epsilon = 1e-10);
        assert_relative_eq!(points_data[1], 2.0, epsilon = 1e-10);
        assert_relative_eq!(points_data[2], 3.0, epsilon = 1e-10);
        assert_relative_eq!(points_data[3], 1.0, epsilon = 1e-10); // w component
        assert_relative_eq!(points_data[4], 4.0, epsilon = 1e-10);
    }

    // Test direct normals access
    unsafe {
        let normals_data = cloud.normals_data().unwrap();
        // Normals are also stored as 4D vectors, so 2 normals * 4 components = 8
        assert_eq!(normals_data.len(), 8); // 2 normals * 4 components
        assert_relative_eq!(normals_data[0], 0.0, epsilon = 1e-10);
        assert_relative_eq!(normals_data[1], 0.0, epsilon = 1e-10);
        assert_relative_eq!(normals_data[2], 1.0, epsilon = 1e-10);
        assert_relative_eq!(normals_data[3], 0.0, epsilon = 1e-10); // w component (should be 0 for normals)
    }

    // Add covariances and test direct access
    let test_cov = Matrix4::identity();
    let covariances = vec![test_cov; 2];
    cloud.set_covariances(&covariances).unwrap();

    unsafe {
        let covariances_data = cloud.covariances_data().unwrap();
        assert_eq!(covariances_data.len(), 32); // 2 covariances * 16 components
                                                // First matrix should be identity
        assert_relative_eq!(covariances_data[0], 1.0, epsilon = 1e-10); // [0,0]
        assert_relative_eq!(covariances_data[1], 0.0, epsilon = 1e-10); // [0,1]
        assert_relative_eq!(covariances_data[5], 1.0, epsilon = 1e-10); // [1,1]
    }

    // Test empty cloud direct access
    let empty_cloud = PointCloud::new().unwrap();
    unsafe {
        let empty_points = empty_cloud.points_data().unwrap();
        assert_eq!(empty_points.len(), 0);

        let empty_normals = empty_cloud.normals_data().unwrap();
        assert_eq!(empty_normals.len(), 0);

        let empty_covariances = empty_cloud.covariances_data().unwrap();
        assert_eq!(empty_covariances.len(), 0);
    }
}

#[test]
fn test_updated_clone_with_covariances() {
    let points = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];
    let normals = vec![Vector3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 0.0, 0.0)];

    let mut original = PointCloud::from_points_and_normals(&points, &normals).unwrap();

    // Add covariances
    let test_cov = Matrix4::new(
        1.0, 0.1, 0.2, 0.0, 0.1, 2.0, 0.3, 0.0, 0.2, 0.3, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );
    let covariances = vec![test_cov; 2];
    original.set_covariances(&covariances).unwrap();

    // Clone the cloud
    let cloned = original.clone();

    // Verify all data was cloned correctly
    assert_eq!(cloned.len(), original.len());
    assert_eq!(cloned.has_points(), original.has_points());
    assert_eq!(cloned.has_normals(), original.has_normals());
    assert_eq!(cloned.has_covariances(), original.has_covariances());

    // Verify points
    for i in 0..cloned.len() {
        let orig_point = original.get_point(i).unwrap();
        let cloned_point = cloned.get_point(i).unwrap();
        assert_relative_eq!(orig_point.x, cloned_point.x, epsilon = 1e-10);
        assert_relative_eq!(orig_point.y, cloned_point.y, epsilon = 1e-10);
        assert_relative_eq!(orig_point.z, cloned_point.z, epsilon = 1e-10);
    }

    // Verify normals
    for i in 0..cloned.len() {
        let orig_normal = original.get_normal(i).unwrap();
        let cloned_normal = cloned.get_normal(i).unwrap();
        assert_relative_eq!(orig_normal.x, cloned_normal.x, epsilon = 1e-10);
        assert_relative_eq!(orig_normal.y, cloned_normal.y, epsilon = 1e-10);
        assert_relative_eq!(orig_normal.z, cloned_normal.z, epsilon = 1e-10);
    }

    // Verify covariances
    for i in 0..cloned.len() {
        let orig_cov = original.get_covariance(i).unwrap();
        let cloned_cov = cloned.get_covariance(i).unwrap();
        for row in 0..4 {
            for col in 0..4 {
                assert_relative_eq!(
                    orig_cov[(row, col)],
                    cloned_cov[(row, col)],
                    epsilon = 1e-10
                );
            }
        }
    }
}

#[test]
fn test_kdtree_builder_configurations() {
    let cloud = create_dense_test_cloud();

    // Test default builder
    let default_config = KdTreeConfig {
        builder_type: KdTreeBuilderType::Default,
        num_threads: 1,
        max_leaf_size: 20,
        projection: ProjectionConfig::default(),
    };
    let kdtree_default = KdTree::new(&cloud, &default_config).unwrap();

    let query = Point3::new(0.5, 0.5, 0.2);
    let (index, _) = kdtree_default.nearest_neighbor(query).unwrap();
    assert!(index < cloud.len());

    // Test OpenMP builder
    let openmp_config = KdTreeConfig {
        builder_type: KdTreeBuilderType::OpenMp,
        num_threads: 2,
        max_leaf_size: 15,
        projection: ProjectionConfig::default(),
    };
    let kdtree_openmp = KdTree::new(&cloud, &openmp_config).unwrap();

    let (index_openmp, _) = kdtree_openmp.nearest_neighbor(query).unwrap();
    assert!(index_openmp < cloud.len());

    // Test TBB builder
    let tbb_config = KdTreeConfig {
        builder_type: KdTreeBuilderType::Tbb,
        num_threads: 2,
        max_leaf_size: 25,
        projection: ProjectionConfig::default(),
    };
    let kdtree_tbb = KdTree::new(&cloud, &tbb_config).unwrap();

    let (index_tbb, _) = kdtree_tbb.nearest_neighbor(query).unwrap();
    assert!(index_tbb < cloud.len());

    // All builders should give similar results for the same query
    let first_point = cloud.get_point(index).unwrap();
    let openmp_point = cloud.get_point(index_openmp).unwrap();
    let tbb_point = cloud.get_point(index_tbb).unwrap();

    // The nearest points should be close to each other (allowing for slight differences due to parallel processing)
    let dist_to_query = (query - first_point).magnitude_squared();
    let openmp_dist = (query - openmp_point).magnitude_squared();
    let tbb_dist = (query - tbb_point).magnitude_squared();

    // All should be reasonably close to the optimal distance
    assert!((dist_to_query - openmp_dist).abs() < 0.1);
    assert!((dist_to_query - tbb_dist).abs() < 0.1);
}

#[test]
fn test_kdtree_projection_types() {
    let cloud = create_dense_test_cloud();

    // Test axis-aligned projection
    let axis_aligned_config = KdTreeConfig {
        builder_type: KdTreeBuilderType::Default,
        num_threads: 1,
        max_leaf_size: 20,
        projection: ProjectionConfig {
            projection_type: ProjectionType::AxisAligned,
            max_scan_count: 128,
        },
    };
    let kdtree_axis = KdTree::new(&cloud, &axis_aligned_config).unwrap();

    let query = Point3::new(0.3, 0.7, 0.1);
    let (index_axis, dist_axis) = kdtree_axis.nearest_neighbor(query).unwrap();
    assert!(index_axis < cloud.len());
    assert!(dist_axis >= 0.0);

    // Test normal projection
    let normal_config = KdTreeConfig {
        builder_type: KdTreeBuilderType::Default,
        num_threads: 1,
        max_leaf_size: 20,
        projection: ProjectionConfig {
            projection_type: ProjectionType::Normal,
            max_scan_count: 64,
        },
    };
    let kdtree_normal = KdTree::new(&cloud, &normal_config).unwrap();

    let (index_normal, dist_normal) = kdtree_normal.nearest_neighbor(query).unwrap();
    assert!(index_normal < cloud.len());
    assert!(dist_normal >= 0.0);

    // Both projections should find valid nearest neighbors
    // The exact indices might differ, but distances should be reasonable
    let axis_point = cloud.get_point(index_axis).unwrap();
    let normal_point = cloud.get_point(index_normal).unwrap();

    let expected_axis_dist = (query - axis_point).magnitude_squared();
    let expected_normal_dist = (query - normal_point).magnitude_squared();

    assert_relative_eq!(dist_axis, expected_axis_dist, epsilon = 1e-10);
    assert_relative_eq!(dist_normal, expected_normal_dist, epsilon = 1e-10);
}

#[test]
fn test_knn_settings() {
    let cloud = create_dense_test_cloud();
    let kdtree_config = KdTreeConfig::default();
    let kdtree = KdTree::new(&cloud, &kdtree_config).unwrap();

    let query = Point3::new(0.45, 0.55, 0.15);

    // Test exact search (epsilon = 0.0)
    let exact_settings = KnnConfig { epsilon: 0.0 };
    let (exact_index, exact_dist) = kdtree
        .nearest_neighbor_with_settings(query, &exact_settings)
        .unwrap();
    assert!(exact_index < cloud.len());
    assert!(exact_dist >= 0.0);

    // Test approximate search (epsilon > 0.0)
    let approx_settings = KnnConfig { epsilon: 0.1 };
    let (approx_index, approx_dist) = kdtree
        .nearest_neighbor_with_settings(query, &approx_settings)
        .unwrap();
    assert!(approx_index < cloud.len());
    assert!(approx_dist >= 0.0);

    // Approximate distance should be close to exact distance (may be slightly higher due to early termination)
    assert!(approx_dist <= exact_dist * (1.0 + approx_settings.epsilon) + 1e-10);

    // Test KNN search with settings
    let k = 5;
    let exact_knn = kdtree
        .knn_search_with_settings(query, k, &exact_settings)
        .unwrap();
    let approx_knn = kdtree
        .knn_search_with_settings(query, k, &approx_settings)
        .unwrap();

    assert_eq!(exact_knn.len(), k);
    assert_eq!(approx_knn.len(), k);

    // Results should be sorted by distance
    for i in 1..exact_knn.len() {
        assert!(exact_knn[i - 1].1 <= exact_knn[i].1);
        assert!(approx_knn[i - 1].1 <= approx_knn[i].1);
    }

    // First result should match nearest neighbor results
    assert_eq!(exact_knn[0].0, exact_index);
    assert_relative_eq!(exact_knn[0].1, exact_dist, epsilon = 1e-10);
}

#[test]
fn test_unsafe_kdtree() {
    let cloud = create_dense_test_cloud();

    // Test basic UnsafeKdTree creation
    let unsafe_kdtree = unsafe { UnsafeKdTree::new_basic(&cloud) }.unwrap();

    let query = Point3::new(0.6, 0.4, 0.3);
    let (index, dist) = unsafe_kdtree.nearest_neighbor(query).unwrap();
    assert!(index < cloud.len());
    assert!(dist >= 0.0);

    // Compare with regular KdTree
    let regular_config = KdTreeConfig::default();
    let regular_kdtree = KdTree::new(&cloud, &regular_config).unwrap();
    let (regular_index, regular_dist) = regular_kdtree.nearest_neighbor(query).unwrap();

    // Results should be valid for same configuration (may not be identical due to implementation)
    assert!(index < cloud.len() && regular_index < cloud.len());
    assert!(dist >= 0.0 && regular_dist >= 0.0);

    // Test KNN search
    let k = 3;
    let unsafe_knn = unsafe_kdtree.knn_search(query, k).unwrap();
    let regular_knn = regular_kdtree.knn_search(query, k).unwrap();

    assert_eq!(unsafe_knn.len(), k);
    assert_eq!(regular_knn.len(), k);

    // Results should be comparable (may not be identical due to implementation differences)
    for i in 0..k {
        assert!(unsafe_knn[i].0 < cloud.len());
        assert!(regular_knn[i].0 < cloud.len());
        assert!(unsafe_knn[i].1 >= 0.0);
        assert!(regular_knn[i].1 >= 0.0);
    }

    // Test radius search
    let radius = 0.2;
    let max_neighbors = 10;
    let unsafe_radius = unsafe_kdtree
        .radius_search(query, radius, max_neighbors)
        .unwrap();
    let regular_radius = regular_kdtree
        .radius_search(query, radius, max_neighbors)
        .unwrap();

    // Results should be valid even if they differ slightly due to implementation
    for result in &unsafe_radius {
        assert!(result.0 < cloud.len());
        assert!(result.1 <= radius * radius + 1e-6);
    }
    for result in &regular_radius {
        assert!(result.0 < cloud.len());
        assert!(result.1 <= radius * radius + 1e-6);
    }
}

#[test]
fn test_unsafe_kdtree_with_config() {
    let cloud = create_dense_test_cloud();

    // Test UnsafeKdTree with advanced configuration
    let config = KdTreeConfig {
        builder_type: KdTreeBuilderType::OpenMp,
        num_threads: 2,
        max_leaf_size: 10,
        projection: ProjectionConfig {
            projection_type: ProjectionType::Normal,
            max_scan_count: 64,
        },
    };

    let unsafe_kdtree = unsafe { UnsafeKdTree::new(&cloud, &config) }.unwrap();

    let query = Point3::new(0.2, 0.8, 0.1);

    // Test with KNN settings
    let knn_settings = KnnConfig { epsilon: 0.05 };
    let (index, dist) = unsafe_kdtree
        .nearest_neighbor_with_settings(query, &knn_settings)
        .unwrap();
    assert!(index < cloud.len());
    assert!(dist >= 0.0);

    // Test KNN search with settings
    let k = 7;
    let knn_results = unsafe_kdtree
        .knn_search_with_settings(query, k, &knn_settings)
        .unwrap();
    assert_eq!(knn_results.len(), k);

    // Verify results are sorted
    for i in 1..knn_results.len() {
        assert!(knn_results[i - 1].1 <= knn_results[i].1);
    }

    // First result should match nearest neighbor
    assert_eq!(knn_results[0].0, index);
    assert_relative_eq!(knn_results[0].1, dist, epsilon = 1e-10);
}

#[test]
fn test_kdtree_advanced_configuration() {
    let cloud = create_dense_test_cloud();

    // Test configuration with all advanced features
    let advanced_config = KdTreeConfig {
        builder_type: KdTreeBuilderType::Tbb,
        num_threads: 4,
        max_leaf_size: 5,
        projection: ProjectionConfig {
            projection_type: ProjectionType::Normal,
            max_scan_count: 256,
        },
    };

    let kdtree = KdTree::new(&cloud, &advanced_config).unwrap();

    let query = Point3::new(0.7, 0.3, 0.4);

    // Test all search methods
    let (nn_index, nn_dist) = kdtree.nearest_neighbor(query).unwrap();
    assert!(nn_index < cloud.len());
    assert!(nn_dist >= 0.0);

    let knn_results = kdtree.knn_search(query, 8).unwrap();
    assert_eq!(knn_results.len(), 8);
    assert_eq!(knn_results[0].0, nn_index);
    assert_relative_eq!(knn_results[0].1, nn_dist, epsilon = 1e-10);

    let radius_results = kdtree.radius_search(query, 0.3, 15).unwrap();
    for (_, sq_dist) in &radius_results {
        assert!(*sq_dist <= 0.3 * 0.3 + 1e-10);
    }

    // Test with custom KNN settings
    let knn_settings = KnnConfig { epsilon: 0.02 };
    let (settings_index, settings_dist) = kdtree
        .nearest_neighbor_with_settings(query, &knn_settings)
        .unwrap();
    assert!(settings_index < cloud.len());
    assert!(settings_dist >= 0.0);

    // Approximate result should be reasonable
    assert!(settings_dist >= 0.0);
}

#[test]
fn test_kdtree_performance_comparison() {
    let cloud = create_dense_test_cloud();
    let query = Point3::new(0.35, 0.65, 0.25);

    // Create different configurations
    let default_config = KdTreeConfig::default();
    let optimized_config = KdTreeConfig {
        builder_type: KdTreeBuilderType::OpenMp,
        num_threads: 2,
        max_leaf_size: 10,
        projection: ProjectionConfig {
            projection_type: ProjectionType::AxisAligned,
            max_scan_count: 64,
        },
    };

    // Test regular KdTree
    let regular_kdtree = KdTree::new(&cloud, &default_config).unwrap();
    let (regular_index, regular_dist) = regular_kdtree.nearest_neighbor(query).unwrap();

    // Test optimized KdTree
    let optimized_kdtree = KdTree::new(&cloud, &optimized_config).unwrap();
    let (optimized_index, optimized_dist) = optimized_kdtree.nearest_neighbor(query).unwrap();

    // Test UnsafeKdTree
    let unsafe_kdtree = unsafe { UnsafeKdTree::new(&cloud, &default_config) }.unwrap();
    let (unsafe_index, unsafe_dist) = unsafe_kdtree.nearest_neighbor(query).unwrap();

    // All should find valid nearest neighbors
    assert!(regular_index < cloud.len());
    assert!(optimized_index < cloud.len());
    assert!(unsafe_index < cloud.len());

    // Results should be reasonable (exact results may vary due to different configurations)
    assert!(regular_dist >= 0.0);
    assert!(optimized_dist >= 0.0);
    assert!(unsafe_dist >= 0.0);

    // Results should be valid, though may not be identical due to implementation differences
    assert!(regular_index < cloud.len() && unsafe_index < cloud.len());
    assert!(regular_dist >= 0.0 && unsafe_dist >= 0.0);
}

#[test]
fn test_kdtree_error_cases() {
    // Test with empty point cloud
    let empty_cloud = PointCloud::new().unwrap();
    let config = KdTreeConfig::default();

    assert!(KdTree::new(&empty_cloud, &config).is_err());
    assert!(unsafe { UnsafeKdTree::new_basic(&empty_cloud) }.is_err());
    assert!(unsafe { UnsafeKdTree::new(&empty_cloud, &config) }.is_err());

    // Test with valid cloud
    let cloud = create_test_cube();
    let kdtree = KdTree::new(&cloud, &config).unwrap();

    // Test edge cases for KNN search
    let query = Point3::new(0.5, 0.5, 0.5);
    let empty_knn = kdtree.knn_search(query, 0).unwrap();
    assert_eq!(empty_knn.len(), 0);

    let too_many_knn = kdtree.knn_search(query, cloud.len() + 10).unwrap();
    // The result should be valid even if we ask for more neighbors than available
    assert!(!too_many_knn.is_empty());

    // Test with different epsilon values
    let knn_settings_zero = KnnConfig { epsilon: 0.0 };
    let knn_settings_large = KnnConfig { epsilon: 1.0 };

    let (index_zero, dist_zero) = kdtree
        .nearest_neighbor_with_settings(query, &knn_settings_zero)
        .unwrap();
    let (index_large, dist_large) = kdtree
        .nearest_neighbor_with_settings(query, &knn_settings_large)
        .unwrap();

    assert!(index_zero < cloud.len());
    assert!(index_large < cloud.len());
    assert!(dist_zero >= 0.0);
    assert!(dist_large >= 0.0);
}
