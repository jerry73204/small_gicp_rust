use approx::assert_relative_eq;
use nalgebra::{Matrix4, Point3, Vector3};
use small_gicp::{
    config::{
        CovarianceEstimationConfig, KdTreeConfig, LocalFeatureEstimationConfig,
        LocalFeatureSetterType, LocalFeaturesBackend, NormalEstimationBackend,
        NormalEstimationConfig,
    },
    kdtree::KdTree,
    point_cloud::PointCloud,
    preprocessing::{estimate_local_features_auto, Preprocessing},
};

#[test]
fn test_covariance_matrix_operations() {
    let mut cloud = PointCloud::new().unwrap();

    // Add some test points
    let points_data = vec![
        1.0, 2.0, 3.0, // Point 0
        4.0, 5.0, 6.0, // Point 1
        7.0, 8.0, 9.0, // Point 2
    ];
    cloud.set_points_bulk(&points_data).unwrap();

    // Test individual covariance set/get
    let test_cov = Matrix4::new(
        1.0, 0.1, 0.2, 0.0, 0.1, 2.0, 0.3, 0.0, 0.2, 0.3, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    // Set covariance for point 0
    cloud.set_covariance(0, test_cov).unwrap();
    assert!(cloud.has_covariances());

    // Get and verify
    let retrieved = cloud.covariance_at(0).unwrap();
    for i in 0..4 {
        for j in 0..4 {
            assert_relative_eq!(test_cov[(i, j)], retrieved[(i, j)], epsilon = 1e-10);
        }
    }

    // Test bulk covariance operations
    let different_cov = Matrix4::new(
        4.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    let covariances = vec![test_cov, different_cov, test_cov];
    cloud.set_covariances(&covariances).unwrap();

    // Verify all covariances
    let all_covs = cloud.covariances().unwrap();
    assert_eq!(all_covs.len(), 3);

    // Check first and second covariances are different
    let cov0 = all_covs.get(0).unwrap();
    let cov1 = all_covs.get(1).unwrap();
    assert!((cov0[(0, 0)] - cov1[(0, 0)]).abs() > 1e-10);

    // Test error cases
    assert!(cloud.covariance_at(100).is_err());
    assert!(cloud.set_covariance(100, test_cov).is_err());

    // Wrong number of covariances
    let wrong_covs = vec![test_cov; 5];
    assert!(cloud.set_covariances(&wrong_covs).is_err());
}

#[test]
fn test_covariance_bulk_data_operations() {
    let mut cloud = PointCloud::new().unwrap();

    // Create 3 points
    let points_data = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
    cloud.set_points_bulk(&points_data).unwrap();

    // Create covariance data in row-major format
    let identity = vec![
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];

    let scaled = vec![
        2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];

    let anisotropic = vec![
        3.0, 0.5, 0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];

    // Combine all covariance data
    let cov_data: Vec<f64> = [identity, scaled, anisotropic].concat();

    // Set covariances from raw data
    cloud.set_covariances_bulk(&cov_data).unwrap();
    assert!(cloud.has_covariances());

    // Verify by retrieving individual covariances
    let cov0 = cloud.covariance_at(0).unwrap();
    assert_relative_eq!(cov0[(0, 0)], 1.0, epsilon = 1e-10);
    assert_relative_eq!(cov0[(1, 1)], 1.0, epsilon = 1e-10);

    let cov1 = cloud.covariance_at(1).unwrap();
    assert_relative_eq!(cov1[(0, 0)], 2.0, epsilon = 1e-10);
    assert_relative_eq!(cov1[(1, 1)], 2.0, epsilon = 1e-10);

    let cov2 = cloud.covariance_at(2).unwrap();
    assert_relative_eq!(cov2[(0, 0)], 3.0, epsilon = 1e-10);
    assert_relative_eq!(cov2[(0, 1)], 0.5, epsilon = 1e-10);
    assert_relative_eq!(cov2[(1, 0)], 0.5, epsilon = 1e-10);

    // Test copy to array
    let mut output = vec![0.0; 48]; // 3 matrices * 16 elements
    cloud.copy_covariances_to_array(&mut output).unwrap();

    // Verify copied data matches input
    for i in 0..48 {
        assert_relative_eq!(output[i], cov_data[i], epsilon = 1e-10);
    }

    // Test error cases
    let wrong_size = vec![1.0; 47]; // Not a multiple of 16
    assert!(cloud.set_covariances_bulk(&wrong_size).is_err());

    let wrong_count = vec![1.0; 32]; // 2 matrices for 3 points
    assert!(cloud.set_covariances_bulk(&wrong_count).is_err());

    let mut wrong_output = vec![0.0; 32]; // Wrong size for output
    assert!(cloud.copy_covariances_to_array(&mut wrong_output).is_err());
}

#[test]
fn test_direct_points_access() {
    let points = vec![
        Point3::new(1.0, 2.0, 3.0),
        Point3::new(4.0, 5.0, 6.0),
        Point3::new(7.0, 8.0, 9.0),
    ];
    let cloud = PointCloud::from_points(&points).unwrap();

    unsafe {
        let points_data = cloud.points_data();

        // Points are stored as 4D homogeneous coordinates
        // Format: x, y, z, w for each point
        assert_eq!(points_data.len(), 12); // 3 points * 4 components

        // Verify first point
        assert_relative_eq!(points_data[0], 1.0, epsilon = 1e-10); // x
        assert_relative_eq!(points_data[1], 2.0, epsilon = 1e-10); // y
        assert_relative_eq!(points_data[2], 3.0, epsilon = 1e-10); // z
        assert_relative_eq!(points_data[3], 1.0, epsilon = 1e-10); // w

        // Verify second point
        assert_relative_eq!(points_data[4], 4.0, epsilon = 1e-10); // x
        assert_relative_eq!(points_data[5], 5.0, epsilon = 1e-10); // y
        assert_relative_eq!(points_data[6], 6.0, epsilon = 1e-10); // z
        assert_relative_eq!(points_data[7], 1.0, epsilon = 1e-10); // w

        // Verify third point
        assert_relative_eq!(points_data[8], 7.0, epsilon = 1e-10); // x
        assert_relative_eq!(points_data[9], 8.0, epsilon = 1e-10); // y
        assert_relative_eq!(points_data[10], 9.0, epsilon = 1e-10); // z
        assert_relative_eq!(points_data[11], 1.0, epsilon = 1e-10); // w
    }
}

#[test]
fn test_direct_normals_access() {
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let normals = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];

    let cloud = PointCloud::from_points_and_normals(&points, &normals).unwrap();

    unsafe {
        let normals_data = cloud.normals_data();

        // Normals are stored as 4D vectors (with w=0)
        assert_eq!(normals_data.len(), 12); // 3 normals * 4 components

        // Verify first normal (0, 0, 1)
        assert_relative_eq!(normals_data[0], 0.0, epsilon = 1e-10); // x
        assert_relative_eq!(normals_data[1], 0.0, epsilon = 1e-10); // y
        assert_relative_eq!(normals_data[2], 1.0, epsilon = 1e-10); // z
        assert_relative_eq!(normals_data[3], 0.0, epsilon = 1e-10); // w

        // Verify second normal (1, 0, 0)
        assert_relative_eq!(normals_data[4], 1.0, epsilon = 1e-10); // x
        assert_relative_eq!(normals_data[5], 0.0, epsilon = 1e-10); // y
        assert_relative_eq!(normals_data[6], 0.0, epsilon = 1e-10); // z
        assert_relative_eq!(normals_data[7], 0.0, epsilon = 1e-10); // w

        // Verify third normal (0, 1, 0)
        assert_relative_eq!(normals_data[8], 0.0, epsilon = 1e-10); // x
        assert_relative_eq!(normals_data[9], 1.0, epsilon = 1e-10); // y
        assert_relative_eq!(normals_data[10], 0.0, epsilon = 1e-10); // z
        assert_relative_eq!(normals_data[11], 0.0, epsilon = 1e-10); // w
    }
}

#[test]
fn test_direct_covariances_access() {
    let mut cloud = PointCloud::new().unwrap();

    // Create 2 points
    let points_data = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
    cloud.set_points_bulk(&points_data).unwrap();

    // Set different covariances
    let cov1 = Matrix4::new(
        1.0, 0.1, 0.0, 0.0, 0.1, 2.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    let cov2 = Matrix4::new(
        4.0, 0.0, 0.2, 0.0, 0.0, 5.0, 0.0, 0.0, 0.2, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    cloud.set_covariances(&vec![cov1, cov2]).unwrap();

    unsafe {
        let cov_data = cloud.covariances_data();

        // 2 matrices * 16 elements each
        assert_eq!(cov_data.len(), 32);

        // Verify first covariance matrix (row-major order)
        assert_relative_eq!(cov_data[0], 1.0, epsilon = 1e-10); // [0,0]
        assert_relative_eq!(cov_data[1], 0.1, epsilon = 1e-10); // [0,1]
        assert_relative_eq!(cov_data[5], 2.0, epsilon = 1e-10); // [1,1]
        assert_relative_eq!(cov_data[10], 3.0, epsilon = 1e-10); // [2,2]
        assert_relative_eq!(cov_data[15], 1.0, epsilon = 1e-10); // [3,3]

        // Verify second covariance matrix
        assert_relative_eq!(cov_data[16], 4.0, epsilon = 1e-10); // [0,0]
        assert_relative_eq!(cov_data[18], 0.2, epsilon = 1e-10); // [0,2]
        assert_relative_eq!(cov_data[21], 5.0, epsilon = 1e-10); // [1,1]
        assert_relative_eq!(cov_data[26], 6.0, epsilon = 1e-10); // [2,2]
        assert_relative_eq!(cov_data[31], 1.0, epsilon = 1e-10); // [3,3]
    }
}

#[test]
fn test_copy_operations_comprehensive() {
    let mut cloud = PointCloud::new().unwrap();

    // Create test data
    let points_data = vec![1.1, 2.2, 3.3, 4.4, 5.5, 6.6];
    let normals_data = vec![
        0.577, 0.577, 0.577, // Normalized (1,1,1)
        0.707, 0.707, 0.0, // Normalized (1,1,0)
    ];

    cloud.set_points_bulk(&points_data).unwrap();
    cloud.set_normals_bulk(&normals_data).unwrap();

    // Set covariances
    let cov = Matrix4::new(
        0.1, 0.01, 0.02, 0.0, 0.01, 0.2, 0.03, 0.0, 0.02, 0.03, 0.3, 0.0, 0.0, 0.0, 0.0, 1.0,
    );
    cloud.set_covariances(&vec![cov; 2]).unwrap();

    // Test copying points
    let mut points_out = vec![0.0; 6];
    cloud.copy_points_to_array(&mut points_out).unwrap();

    for i in 0..6 {
        assert_relative_eq!(points_out[i], points_data[i], epsilon = 1e-10);
    }

    // Test copying normals
    let mut normals_out = vec![0.0; 6];
    cloud.copy_normals_to_array(&mut normals_out).unwrap();

    for i in 0..6 {
        assert_relative_eq!(normals_out[i], normals_data[i], epsilon = 1e-6);
    }

    // Test copying covariances
    let mut cov_out = vec![0.0; 32]; // 2 matrices * 16 elements
    cloud.copy_covariances_to_array(&mut cov_out).unwrap();

    // Verify first few elements of first matrix
    assert_relative_eq!(cov_out[0], 0.1, epsilon = 1e-10); // [0,0]
    assert_relative_eq!(cov_out[1], 0.01, epsilon = 1e-10); // [0,1]
    assert_relative_eq!(cov_out[5], 0.2, epsilon = 1e-10); // [1,1]

    // Verify first few elements of second matrix (should be same as first)
    assert_relative_eq!(cov_out[16], 0.1, epsilon = 1e-10); // [0,0]
    assert_relative_eq!(cov_out[17], 0.01, epsilon = 1e-10); // [0,1]
    assert_relative_eq!(cov_out[21], 0.2, epsilon = 1e-10); // [1,1]
}

#[test]
fn test_empty_cloud_direct_access() {
    let cloud = PointCloud::new().unwrap();

    unsafe {
        // Empty cloud should return empty slices
        let points = cloud.points_data();
        assert_eq!(points.len(), 0);

        let normals = cloud.normals_data();
        assert_eq!(normals.len(), 0);

        let covariances = cloud.covariances_data();
        assert_eq!(covariances.len(), 0);
    }

    // Copy operations should fail with empty cloud
    let mut array = vec![0.0; 1];
    assert!(cloud.copy_points_to_array(&mut array).is_err());
    assert!(cloud.copy_normals_to_array(&mut array).is_err());
    assert!(cloud.copy_covariances_to_array(&mut array).is_err());
}

#[test]
fn test_covariance_estimation_integration() {
    // Create a cloud with some structure
    let mut points = Vec::new();

    // Create a planar structure
    for i in 0..5 {
        for j in 0..5 {
            points.push(Point3::new(
                i as f64 * 0.1,
                j as f64 * 0.1,
                0.0 + (i as f64 * 0.001), // Slight variation in Z
            ));
        }
    }

    let mut cloud = PointCloud::from_points(&points).unwrap();

    // Estimate covariances using auto estimation
    let config = LocalFeatureEstimationConfig {
        setter_type: LocalFeatureSetterType::Covariance,
        backend: LocalFeaturesBackend::Default,
        num_neighbors: 10,
        num_threads: 1,
    };

    estimate_local_features_auto(&mut cloud, &config).unwrap();

    assert!(cloud.has_covariances());

    // Check that covariances are reasonable
    let cov = cloud.covariance_at(12).unwrap(); // Center point

    // For a planar structure, Z variance should be smaller
    assert!(cov[(0, 0)] > 0.0); // X variance
    assert!(cov[(1, 1)] > 0.0); // Y variance
    assert!(cov[(2, 2)] > 0.0); // Z variance

    // Check symmetry
    assert_relative_eq!(cov[(0, 1)], cov[(1, 0)], epsilon = 1e-10);
    assert_relative_eq!(cov[(0, 2)], cov[(2, 0)], epsilon = 1e-10);
    assert_relative_eq!(cov[(1, 2)], cov[(2, 1)], epsilon = 1e-10);

    // Test combined normal and covariance estimation
    let mut cloud2 = PointCloud::from_points(&points).unwrap();

    let normal_covariance_config = LocalFeatureEstimationConfig {
        setter_type: LocalFeatureSetterType::NormalCovariance,
        backend: LocalFeaturesBackend::Default,
        num_neighbors: 10,
        num_threads: 1,
    };

    estimate_local_features_auto(&mut cloud2, &normal_covariance_config).unwrap();

    assert!(cloud2.has_normals());
    assert!(cloud2.has_covariances());

    // For a planar structure, normals should point mostly in Z direction
    let normal = cloud2.normal_at(12).unwrap();
    assert!(normal.z.abs() > 0.9); // Should be close to (0, 0, 1)
}

#[test]
fn test_validation_functions_comprehensive() {
    // Test all combinations of data presence

    // Empty cloud
    let empty = PointCloud::new().unwrap();
    assert!(!empty.has_points());
    assert!(!empty.has_normals());
    assert!(!empty.has_covariances());
    assert!(empty.empty());

    // Points only
    let points = vec![Point3::new(1.0, 2.0, 3.0)];
    let points_only = PointCloud::from_points(&points).unwrap();
    assert!(points_only.has_points());
    assert!(points_only.has_normals()); // May have space allocated
    assert!(!points_only.empty());

    // Points and normals
    let normals = vec![Vector3::new(0.0, 0.0, 1.0)];
    let with_normals = PointCloud::from_points_and_normals(&points, &normals).unwrap();
    assert!(with_normals.has_points());
    assert!(with_normals.has_normals());
    assert!(!with_normals.empty());

    // All data types
    let mut full_cloud = PointCloud::from_points_and_normals(&points, &normals).unwrap();
    let cov = Matrix4::identity();
    full_cloud.set_covariances(&vec![cov]).unwrap();

    assert!(full_cloud.has_points());
    assert!(full_cloud.has_normals());
    assert!(full_cloud.has_covariances());
    assert!(!full_cloud.empty());
}

#[test]
fn test_data_persistence_after_operations() {
    let mut cloud = PointCloud::new().unwrap();

    // Add points
    let points_data = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
    cloud.set_points_bulk(&points_data).unwrap();

    // Add normals
    let normals_data = vec![0.0, 0.0, 1.0, 1.0, 0.0, 0.0];
    cloud.set_normals_bulk(&normals_data).unwrap();

    // Add covariances
    let cov = Matrix4::identity();
    cloud.set_covariances(&vec![cov; 2]).unwrap();

    // Verify all data is still accessible
    assert!(cloud.has_points());
    assert!(cloud.has_normals());
    assert!(cloud.has_covariances());

    // Verify data integrity
    let point0 = cloud.point_at(0).unwrap();
    assert_relative_eq!(point0.0, 1.0, epsilon = 1e-10);

    let normal0 = cloud.normal_at(0).unwrap();
    assert_relative_eq!(normal0.z, 1.0, epsilon = 1e-10);

    let cov0 = cloud.covariance_at(0).unwrap();
    assert_relative_eq!(cov0[(0, 0)], 1.0, epsilon = 1e-10);

    // Test that direct access still works
    unsafe {
        let points_direct = cloud.points_data();
        assert!(points_direct.len() > 0);

        let normals_direct = cloud.normals_data();
        assert!(normals_direct.len() > 0);

        let cov_direct = cloud.covariances_data();
        assert!(cov_direct.len() > 0);
    }
}

#[test]
fn test_performance_oriented_access() {
    // Create a larger cloud for performance testing
    let num_points = 1000;
    let mut points_data = Vec::with_capacity(num_points * 3);

    for i in 0..num_points {
        let angle = (i as f64) * 2.0 * std::f64::consts::PI / (num_points as f64);
        points_data.push(angle.cos());
        points_data.push(angle.sin());
        points_data.push(i as f64 * 0.01);
    }

    let mut cloud = PointCloud::new().unwrap();
    cloud.set_points_bulk(&points_data).unwrap();

    // Set all covariances at once using bulk operation
    let identity_flat: Vec<f64> = (0..16)
        .map(|i| if i % 5 == 0 { 1.0 } else { 0.0 })
        .collect();
    let all_cov_data: Vec<f64> = identity_flat.repeat(num_points);

    cloud.set_covariances_bulk(&all_cov_data).unwrap();

    // Direct access is most efficient for reading
    unsafe {
        let points_direct = cloud.points_data();
        assert_eq!(points_direct.len(), num_points * 4); // 4D homogeneous

        let cov_direct = cloud.covariances_data();
        assert_eq!(cov_direct.len(), num_points * 16);

        // Verify a few samples
        assert!(points_direct[0].abs() - 1.0 < 0.01); // First point x ~ cos(0) = 1
        assert!(cov_direct[0] - 1.0 < 1e-10); // First covariance [0,0] = 1
    }

    // Bulk copy is efficient for extracting data
    let mut points_out = vec![0.0; num_points * 3];
    cloud.copy_points_to_array(&mut points_out).unwrap();

    let mut cov_out = vec![0.0; num_points * 16];
    cloud.copy_covariances_to_array(&mut cov_out).unwrap();

    // Verify extraction worked
    assert!(points_out[0].abs() - 1.0 < 0.01);
    assert!(cov_out[0] - 1.0 < 1e-10);
}
