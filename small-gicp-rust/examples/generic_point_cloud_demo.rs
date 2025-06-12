//! Demonstration of the generic point cloud trait system.
//!
//! This example shows how to use the trait-based approach to work with
//! point clouds in a generic way, similar to C++ templates.

use nalgebra::{Matrix4, Point3, Vector3, Vector4};
use small_gicp_rust::{
    point_cloud::conversions,
    prelude::*,
    traits::{helpers, MutablePointCloudTrait, Point4, PointCloudTrait},
};
use std::fmt::Debug;

fn main() -> Result<()> {
    println!("=== Generic Point Cloud Trait System Demo ===\n");

    // 1. Basic trait usage with C wrapper types
    demonstrate_basic_trait_usage()?;

    // 2. Generic algorithms
    demonstrate_generic_algorithms()?;

    // 3. Generic transformations
    demonstrate_generic_transformations()?;

    // 4. Conversion utilities
    demonstrate_conversion_utilities()?;

    // 5. Custom trait implementation example
    demonstrate_custom_implementation();

    println!("Demo completed successfully!");
    Ok(())
}

fn demonstrate_basic_trait_usage() -> Result<()> {
    println!("1. Basic Trait Usage");
    println!("-------------------");

    // Create a point cloud using the C wrapper
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

    let mut cloud = PointCloud::from_points_and_normals(&points, &normals)?;

    // Add covariances
    let covariances = vec![
        Matrix4::identity(),
        Matrix4::identity() * 2.0,
        Matrix4::identity() * 3.0,
    ];
    cloud.set_covariances(&covariances)?;

    // Use trait methods for access
    println!("Point cloud size: {}", cloud.size());
    println!("Has points: {}", cloud.has_points());
    println!("Has normals: {}", cloud.has_normals());
    println!("Has covariances: {}", cloud.has_covariances());

    // Access data using 4D vectors
    for i in 0..cloud.size() {
        let point = cloud.point(i);
        let normal = cloud.normal(i).unwrap();
        let cov = cloud.covariance(i).unwrap();

        println!(
            "Point {}: ({:.1}, {:.1}, {:.1}, {:.1})",
            i, point.x, point.y, point.z, point.w
        );
        println!(
            "Normal {}: ({:.1}, {:.1}, {:.1}, {:.1})",
            i, normal.x, normal.y, normal.z, normal.w
        );
        println!(
            "Covariance {} diagonal: [{:.1}, {:.1}, {:.1}, {:.1}]",
            i,
            cov[(0, 0)],
            cov[(1, 1)],
            cov[(2, 2)],
            cov[(3, 3)]
        );
    }

    println!();
    Ok(())
}

fn demonstrate_generic_algorithms() -> Result<()> {
    println!("2. Generic Algorithms");
    println!("--------------------");

    // Create test data
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(5.0, 5.0, 5.0),
        Point3::new(10.0, 10.0, 10.0),
        Point3::new(-5.0, -5.0, -5.0),
    ];
    let cloud = PointCloud::from_points(&points)?;

    // Generic algorithm: count points in a bounding box
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

    let min_bounds = helpers::point_from_xyz(-1.0, -1.0, -1.0);
    let max_bounds = helpers::point_from_xyz(6.0, 6.0, 6.0);

    let count = count_points_in_box(&cloud, min_bounds, max_bounds);
    println!(
        "Points in box [{:.1}, {:.1}, {:.1}] to [{:.1}, {:.1}, {:.1}]: {}",
        min_bounds.x, min_bounds.y, min_bounds.z, max_bounds.x, max_bounds.y, max_bounds.z, count
    );

    // Generic algorithm: compute centroid
    fn compute_centroid<P: PointCloudTrait>(cloud: &P) -> Point4<f64> {
        if cloud.empty() {
            return helpers::point_from_xyz(0.0, 0.0, 0.0);
        }

        let mut sum = Vector4::zeros();
        for i in 0..cloud.size() {
            sum += cloud.point(i);
        }
        sum / cloud.size() as f64
    }

    let centroid = compute_centroid(&cloud);
    println!(
        "Centroid: ({:.1}, {:.1}, {:.1})",
        centroid.x, centroid.y, centroid.z
    );

    // Generic algorithm: find closest point to a query
    fn find_closest_point<P: PointCloudTrait>(
        cloud: &P,
        query: Point4<f64>,
    ) -> Option<(usize, f64)> {
        if cloud.empty() {
            return None;
        }

        let mut best_index = 0;
        let mut best_distance = f64::INFINITY;

        for i in 0..cloud.size() {
            let point = cloud.point(i);
            let diff = point - query;
            let distance = (diff.x * diff.x + diff.y * diff.y + diff.z * diff.z).sqrt();

            if distance < best_distance {
                best_distance = distance;
                best_index = i;
            }
        }

        Some((best_index, best_distance))
    }

    let query = helpers::point_from_xyz(3.0, 3.0, 3.0);
    if let Some((index, distance)) = find_closest_point(&cloud, query) {
        println!(
            "Closest point to ({:.1}, {:.1}, {:.1}) is index {} at distance {:.2}",
            query.x, query.y, query.z, index, distance
        );
    }

    println!();
    Ok(())
}

fn demonstrate_generic_transformations() -> Result<()> {
    println!("3. Generic Transformations");
    println!("-------------------------");

    // Create test data
    let points = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];
    let mut cloud = PointCloud::from_points(&points)?;

    println!("Original points:");
    for i in 0..cloud.size() {
        let point = cloud.point(i);
        println!(
            "  Point {}: ({:.1}, {:.1}, {:.1})",
            i, point.x, point.y, point.z
        );
    }

    // Generic transformation: translate all points
    fn translate_cloud<P: MutablePointCloudTrait>(cloud: &mut P, offset: Point4<f64>) {
        for i in 0..cloud.size() {
            let mut point = cloud.point(i);
            point.x += offset.x;
            point.y += offset.y;
            point.z += offset.z;
            cloud.set_point(i, point);
        }
    }

    let offset = helpers::point_from_xyz(10.0, 20.0, 30.0);
    translate_cloud(&mut cloud, offset);

    println!(
        "After translation by ({:.1}, {:.1}, {:.1}):",
        offset.x, offset.y, offset.z
    );
    for i in 0..cloud.size() {
        let point = cloud.point(i);
        println!(
            "  Point {}: ({:.1}, {:.1}, {:.1})",
            i, point.x, point.y, point.z
        );
    }

    // Generic transformation: scale all points
    fn scale_cloud<P: MutablePointCloudTrait>(cloud: &mut P, scale: f64) {
        for i in 0..cloud.size() {
            let mut point = cloud.point(i);
            point.x *= scale;
            point.y *= scale;
            point.z *= scale;
            cloud.set_point(i, point);
        }
    }

    scale_cloud(&mut cloud, 0.1);

    println!("After scaling by 0.1:");
    for i in 0..cloud.size() {
        let point = cloud.point(i);
        println!(
            "  Point {}: ({:.1}, {:.1}, {:.1})",
            i, point.x, point.y, point.z
        );
    }

    println!();
    Ok(())
}

fn demonstrate_conversion_utilities() -> Result<()> {
    println!("4. Conversion Utilities");
    println!("----------------------");

    // Create source data with different types
    let points3 = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];
    let normals3 = vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)];

    println!("Original 3D data:");
    for i in 0..points3.len() {
        println!("  Point {}: {:?}", i, points3[i]);
        println!("  Normal {}: {:?}", i, normals3[i]);
    }

    // Convert to 4D using helpers
    let points4 = conversions::points3_to_points4(&points3);
    let normals4 = conversions::normals3_to_normals4(&normals3);

    println!("Converted to 4D:");
    for i in 0..points4.len() {
        println!(
            "  Point4 {}: ({:.1}, {:.1}, {:.1}, {:.1})",
            i, points4[i].x, points4[i].y, points4[i].z, points4[i].w
        );
        println!(
            "  Normal4 {}: ({:.1}, {:.1}, {:.1}, {:.1})",
            i, normals4[i].x, normals4[i].y, normals4[i].z, normals4[i].w
        );
    }

    // Convert back to 3D
    let points3_back = conversions::points4_to_points3(&points4);
    let normals3_back = conversions::normals4_to_normals3(&normals4);

    println!("Converted back to 3D:");
    for i in 0..points3_back.len() {
        println!("  Point {}: {:?}", i, points3_back[i]);
        println!("  Normal {}: {:?}", i, normals3_back[i]);
    }

    // Verify round-trip conversion
    assert_eq!(points3, points3_back);
    assert_eq!(normals3, normals3_back);
    println!("Round-trip conversion verified!");

    // Demonstrate creating a PointCloud from trait
    let source_cloud = PointCloud::from_points_and_normals(&points3, &normals3)?;
    let target_cloud = conversions::from_trait(&source_cloud)?;

    println!("Created new PointCloud from trait:");
    println!(
        "  Source size: {}, Target size: {}",
        source_cloud.size(),
        target_cloud.size()
    );
    println!(
        "  Data integrity: {}",
        source_cloud.point(0) == target_cloud.point(0)
            && source_cloud.normal(0) == target_cloud.normal(0)
    );

    println!();
    Ok(())
}

fn demonstrate_custom_implementation() {
    println!("5. Custom Trait Implementation Example");
    println!("-------------------------------------");

    // Example of a custom point cloud implementation
    #[derive(Debug)]
    struct SimplePointCloud {
        points: Vec<Point4<f64>>,
        normals: Option<Vec<Vector4<f64>>>,
    }

    impl SimplePointCloud {
        fn new() -> Self {
            Self {
                points: Vec::new(),
                normals: None,
            }
        }

        fn with_normals(mut self) -> Self {
            self.normals = Some(Vec::new());
            self
        }
    }

    impl PointCloudTrait for SimplePointCloud {
        fn size(&self) -> usize {
            self.points.len()
        }

        fn has_points(&self) -> bool {
            true
        }

        fn has_normals(&self) -> bool {
            self.normals.is_some()
        }

        fn point(&self, index: usize) -> Point4<f64> {
            self.points[index]
        }

        fn normal(&self, index: usize) -> Option<Vector4<f64>> {
            self.normals.as_ref().map(|normals| normals[index])
        }
    }

    impl MutablePointCloudTrait for SimplePointCloud {
        fn resize(&mut self, size: usize) {
            self.points.resize(size, Vector4::zeros());
            if let Some(ref mut normals) = self.normals {
                normals.resize(size, Vector4::zeros());
            }
        }

        fn set_point(&mut self, index: usize, point: Point4<f64>) {
            self.points[index] = point;
        }

        fn set_normal(&mut self, index: usize, normal: Vector4<f64>) {
            if let Some(ref mut normals) = self.normals {
                normals[index] = normal;
            }
        }
    }

    // Test the custom implementation
    let mut custom_cloud = SimplePointCloud::new().with_normals();
    custom_cloud.resize(2);

    custom_cloud.set_point(0, helpers::point_from_xyz(1.0, 2.0, 3.0));
    custom_cloud.set_point(1, helpers::point_from_xyz(4.0, 5.0, 6.0));
    custom_cloud.set_normal(0, helpers::normal_from_xyz(1.0, 0.0, 0.0));
    custom_cloud.set_normal(1, helpers::normal_from_xyz(0.0, 1.0, 0.0));

    println!("Custom point cloud:");
    println!("  Size: {}", custom_cloud.size());
    println!("  Has normals: {}", custom_cloud.has_normals());

    for i in 0..custom_cloud.size() {
        let point = custom_cloud.point(i);
        let normal = custom_cloud.normal(i).unwrap();
        println!(
            "  Point {}: ({:.1}, {:.1}, {:.1})",
            i, point.x, point.y, point.z
        );
        println!(
            "  Normal {}: ({:.1}, {:.1}, {:.1})",
            i, normal.x, normal.y, normal.z
        );
    }

    // Use generic algorithm with custom implementation
    fn compute_bounding_box<P: PointCloudTrait>(cloud: &P) -> (Point4<f64>, Point4<f64>) {
        if cloud.empty() {
            let zero = helpers::point_from_xyz(0.0, 0.0, 0.0);
            return (zero, zero);
        }

        let first_point = cloud.point(0);
        let mut min_bounds = first_point;
        let mut max_bounds = first_point;

        for i in 1..cloud.size() {
            let point = cloud.point(i);
            min_bounds.x = min_bounds.x.min(point.x);
            min_bounds.y = min_bounds.y.min(point.y);
            min_bounds.z = min_bounds.z.min(point.z);
            max_bounds.x = max_bounds.x.max(point.x);
            max_bounds.y = max_bounds.y.max(point.y);
            max_bounds.z = max_bounds.z.max(point.z);
        }

        (min_bounds, max_bounds)
    }

    let (min_bounds, max_bounds) = compute_bounding_box(&custom_cloud);
    println!(
        "  Bounding box: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})",
        min_bounds.x, min_bounds.y, min_bounds.z, max_bounds.x, max_bounds.y, max_bounds.z
    );

    println!();
}
