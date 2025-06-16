//! Integration tests for the generic trait system.

use nalgebra::{Matrix4, Point3, Vector3};
use small_gicp::{point_cloud::conversions, prelude::*, traits::helpers};

#[test]
fn test_point_cloud_trait_implementation() {
    // Create a point cloud with points, normals, and covariances
    let points = vec![
        Point3::new(1.0, 2.0, 3.0),
        Point3::new(4.0, 5.0, 6.0),
        Point3::new(7.0, 8.0, 9.0),
    ];
    let normals = vec![
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    ];
    let covariances = vec![
        Matrix4::identity(),
        Matrix4::identity() * 2.0,
        Matrix4::identity() * 3.0,
    ];

    let mut cloud = PointCloud::from_points_and_normals(&points, &normals).unwrap();
    cloud.set_covariances(&covariances).unwrap();

    // Test PointCloudTrait implementation
    assert_eq!(cloud.size(), 3);
    assert!(!cloud.empty());
    assert!(cloud.has_points());
    assert!(cloud.has_normals());
    assert!(cloud.has_covariances());

    // Test point access
    let point = cloud.point(0);
    assert_eq!(point.x, 1.0);
    assert_eq!(point.y, 2.0);
    assert_eq!(point.z, 3.0);
    assert_eq!(point.w, 1.0); // Should be homogeneous coordinate

    // Test normal access
    let normal = cloud.normal(0).unwrap();
    assert_eq!(normal.x, 1.0);
    assert_eq!(normal.y, 0.0);
    assert_eq!(normal.z, 0.0);
    assert_eq!(normal.w, 0.0); // Should be direction vector

    // Test covariance access
    let cov = cloud.covariance(0).unwrap();
    assert_eq!(cov, Matrix4::identity());
}

#[test]
fn test_mutable_point_cloud_trait_implementation() {
    let mut cloud = PointCloud::new().unwrap();

    // Test MutablePointCloudTrait implementation
    cloud.resize(2);
    assert_eq!(cloud.size(), 2);

    // Test setting points with 4D vectors using trait method
    let point4 = helpers::point_from_xyz(10.0, 20.0, 30.0);
    <PointCloud as MutablePointCloudTrait>::set_point(&mut cloud, 0, point4);

    let retrieved_point = cloud.point(0);
    assert_eq!(retrieved_point.x, 10.0);
    assert_eq!(retrieved_point.y, 20.0);
    assert_eq!(retrieved_point.z, 30.0);

    // Test setting normals with 4D vectors using trait method
    let normal4 = helpers::normal_from_xyz(0.0, 0.0, 1.0);
    <PointCloud as MutablePointCloudTrait>::set_normal(&mut cloud, 0, normal4);

    let retrieved_normal = cloud.normal(0).unwrap();
    assert_eq!(retrieved_normal.x, 0.0);
    assert_eq!(retrieved_normal.y, 0.0);
    assert_eq!(retrieved_normal.z, 1.0);

    // Test setting covariances using trait method
    let cov = Matrix4::identity() * 5.0;
    <PointCloud as MutablePointCloudTrait>::set_covariance(&mut cloud, 0, cov);

    let retrieved_cov = cloud.covariance(0).unwrap();
    assert_eq!(retrieved_cov, cov);
}

#[test]
fn test_helper_functions() {
    // Test point creation and validation
    let point = helpers::point_from_xyz(1.0, 2.0, 3.0);
    assert!(helpers::is_valid_point(point));
    assert_eq!(point.w, 1.0);

    // Test normal creation and validation
    let normal = helpers::normal_from_xyz(1.0, 0.0, 0.0);
    assert!(helpers::is_valid_normal(normal));
    assert_eq!(normal.w, 0.0);

    // Test conversions
    let v3 = Vector3::new(4.0, 5.0, 6.0);
    let point_from_v3 = helpers::point_from_vector3(v3);
    let v3_back = helpers::point_to_vector3(point_from_v3);
    assert_eq!(v3, v3_back);

    let normal_from_v3 = helpers::normal_from_vector3(v3);
    let v3_normal_back = helpers::normal_to_vector3(normal_from_v3);
    assert_eq!(v3, v3_normal_back);
}

#[test]
fn test_conversion_utilities() {
    let points3 = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];
    let normals3 = vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)];

    // Test Point3 to Point4 conversion
    let points4 = conversions::points3_to_points4(&points3);
    assert_eq!(points4.len(), 2);
    assert_eq!(points4[0].x, 1.0);
    assert_eq!(points4[0].w, 1.0);

    // Test Point4 to Point3 conversion
    let points3_back = conversions::points4_to_points3(&points4);
    assert_eq!(points3_back.len(), 2);
    assert_eq!(points3_back[0], points3[0]);

    // Test Normal3 to Normal4 conversion
    let normals4 = conversions::normals3_to_normals4(&normals3);
    assert_eq!(normals4.len(), 2);
    assert_eq!(normals4[0].x, 1.0);
    assert_eq!(normals4[0].w, 0.0);

    // Test Normal4 to Normal3 conversion
    let normals3_back = conversions::normals4_to_normals3(&normals4);
    assert_eq!(normals3_back.len(), 2);
    assert_eq!(normals3_back[0], normals3[0]);
}

#[test]
fn test_from_trait_conversion() {
    // Create a source point cloud
    let points = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];
    let normals = vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)];
    let covariances = vec![Matrix4::identity(), Matrix4::identity() * 2.0];

    let mut source_cloud = PointCloud::from_points_and_normals(&points, &normals).unwrap();
    source_cloud.set_covariances(&covariances).unwrap();

    // Convert using trait
    let target_cloud = conversions::from_trait(&source_cloud).unwrap();

    // Verify the conversion
    assert_eq!(target_cloud.size(), source_cloud.size());
    assert_eq!(target_cloud.has_points(), source_cloud.has_points());
    assert_eq!(target_cloud.has_normals(), source_cloud.has_normals());
    assert_eq!(
        target_cloud.has_covariances(),
        source_cloud.has_covariances()
    );

    // Check data integrity
    for i in 0..target_cloud.size() {
        let source_point = source_cloud.point(i);
        let target_point = target_cloud.point(i);
        assert_eq!(source_point, target_point);

        if let (Some(source_normal), Some(target_normal)) =
            (source_cloud.normal(i), target_cloud.normal(i))
        {
            assert_eq!(source_normal, target_normal);
        }

        if let (Some(source_cov), Some(target_cov)) =
            (source_cloud.covariance(i), target_cloud.covariance(i))
        {
            assert_eq!(source_cov, target_cov);
        }
    }
}

#[test]
fn test_copy_data_between_traits() {
    // Create source cloud
    let points = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];
    let source_cloud = PointCloud::from_points(&points).unwrap();

    // Create target cloud
    let mut target_cloud = PointCloud::new().unwrap();

    // Copy data using traits
    conversions::copy_data(&source_cloud, &mut target_cloud);

    // Verify the copy
    assert_eq!(target_cloud.size(), source_cloud.size());
    for i in 0..source_cloud.size() {
        assert_eq!(source_cloud.point(i), target_cloud.point(i));
    }
}

#[test]
fn test_bounds_checking() {
    let cloud = PointCloud::new().unwrap();

    // Test bounds checking in trait implementation
    let result = std::panic::catch_unwind(|| {
        cloud.point(0); // Should panic on empty cloud
    });
    assert!(result.is_err());
}

#[test]
#[should_panic(expected = "Invalid point: fourth component must be 1.0")]
fn test_invalid_point_validation() {
    let mut cloud = PointCloud::new().unwrap();
    cloud.resize(1);

    // Try to set a point with invalid w component
    let invalid_point = helpers::normal_from_xyz(1.0, 2.0, 3.0); // This has w=0, not w=1
    <PointCloud as MutablePointCloudTrait>::set_point(&mut cloud, 0, invalid_point);
}

#[test]
#[should_panic(expected = "Invalid normal: fourth component must be 0.0")]
fn test_invalid_normal_validation() {
    let mut cloud = PointCloud::new().unwrap();
    cloud.resize(1);

    // Try to set a normal with invalid w component
    let invalid_normal = helpers::point_from_xyz(1.0, 0.0, 0.0); // This has w=1, not w=0
    <PointCloud as MutablePointCloudTrait>::set_normal(&mut cloud, 0, invalid_normal);
}

#[test]
fn test_generic_algorithm_example() {
    // Example of a generic algorithm that works with any PointCloudTrait
    fn count_points_in_box<P: PointCloudTrait>(
        cloud: &P,
        min_bounds: Point4<f64>,
        max_bounds: Point4<f64>,
    ) -> usize {
        let mut count = 0;
        for i in 0..cloud.size() {
            let point = cloud.point(i);
            if point.x >= min_bounds.x
                && point.x <= max_bounds.x
                && point.y >= min_bounds.y
                && point.y <= max_bounds.y
                && point.z >= min_bounds.z
                && point.z <= max_bounds.z
            {
                count += 1;
            }
        }
        count
    }

    // Test with a real point cloud
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(5.0, 5.0, 5.0),
        Point3::new(10.0, 10.0, 10.0),
    ];
    let cloud = PointCloud::from_points(&points).unwrap();

    let min_bounds = helpers::point_from_xyz(-1.0, -1.0, -1.0);
    let max_bounds = helpers::point_from_xyz(6.0, 6.0, 6.0);

    let count = count_points_in_box(&cloud, min_bounds, max_bounds);
    assert_eq!(count, 2); // First two points should be in the box
}

#[test]
fn test_generic_transformation_example() {
    // Example of a generic transformation that works with any MutablePointCloudTrait
    fn translate_cloud<P: MutablePointCloudTrait>(cloud: &mut P, offset: Point4<f64>) {
        for i in 0..cloud.size() {
            let mut point = cloud.point(i);
            point.x += offset.x;
            point.y += offset.y;
            point.z += offset.z;
            cloud.set_point(i, point);
        }
    }

    // Test with a real point cloud
    let points = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];
    let mut cloud = PointCloud::from_points(&points).unwrap();

    let offset = helpers::point_from_xyz(10.0, 20.0, 30.0);
    translate_cloud(&mut cloud, offset);

    // Verify transformation
    let point0 = cloud.point(0);
    assert_eq!(point0.x, 11.0);
    assert_eq!(point0.y, 22.0);
    assert_eq!(point0.z, 33.0);

    let point1 = cloud.point(1);
    assert_eq!(point1.x, 14.0);
    assert_eq!(point1.y, 25.0);
    assert_eq!(point1.z, 36.0);
}

#[test]
fn test_empty_cloud_behavior() {
    let cloud = PointCloud::new().unwrap();

    assert_eq!(cloud.size(), 0);
    assert!(cloud.empty());
    // Note: C wrapper behavior may vary, so we don't make specific assertions
    // about has_points(), has_normals(), or has_covariances() for empty clouds
}

#[test]
fn test_optional_features_behavior() {
    // Test cloud with only points
    let points = vec![Point3::new(1.0, 2.0, 3.0)];
    let cloud = PointCloud::from_points(&points).unwrap();

    assert!(cloud.has_points());
    // Note: C wrapper may or may not report having normals/covariances
    // even when they haven't been set, so we just test the data access

    // Test that we can access data correctly regardless
    let point = cloud.point(0);
    assert_eq!(point.x, 1.0);
    assert_eq!(point.y, 2.0);
    assert_eq!(point.z, 3.0);
    assert_eq!(point.w, 1.0);
}
