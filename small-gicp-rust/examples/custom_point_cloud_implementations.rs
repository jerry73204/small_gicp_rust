//! Comprehensive examples of custom point cloud implementations.
//!
//! This example demonstrates various strategies for implementing custom point cloud types
//! that work with the generic trait system, including different storage layouts,
//! performance optimizations, and specialized use cases.

use nalgebra::{Matrix4, Vector3, Vector4};
use small_gicp_rust::{
    point_cloud::conversions,
    prelude::*,
    traits::{helpers, MutablePointCloudTrait, Point4, PointCloudTrait},
};
use std::fmt::Debug;

fn main() -> Result<()> {
    println!("=== Custom Point Cloud Implementation Examples ===\n");

    // 1. Array of Structures (AoS) - Simple and cache-friendly for random access
    demonstrate_aos_implementation()?;

    // 2. Structure of Arrays (SoA) - Better for vectorized operations
    demonstrate_soa_implementation()?;

    // 3. Minimal Point Cloud - For memory-constrained environments
    demonstrate_minimal_implementation()?;

    // 4. Specialized 2D Point Cloud - For planar data
    demonstrate_2d_implementation()?;

    // 5. Builder Pattern - For gradual construction
    demonstrate_builder_pattern()?;

    // 6. Interoperability between implementations
    demonstrate_interoperability()?;

    println!("All custom implementations demonstrated successfully!");
    Ok(())
}

/// Example 1: Array of Structures (AoS) Layout
///
/// This layout stores complete point data together, which is cache-friendly
/// for algorithms that access all attributes of individual points.
fn demonstrate_aos_implementation() -> Result<()> {
    println!("1. Array of Structures (AoS) Implementation");
    println!("==========================================");

    #[derive(Debug, Clone, Copy)]
    struct PointData {
        position: Point4<f64>,
        normal: Vector4<f64>,
        covariance: Matrix4<f64>,
    }

    impl Default for PointData {
        fn default() -> Self {
            Self {
                position: helpers::point_from_xyz(0.0, 0.0, 0.0),
                normal: helpers::normal_from_xyz(0.0, 0.0, 1.0),
                covariance: Matrix4::identity(),
            }
        }
    }

    #[derive(Debug)]
    pub struct AoSPointCloud {
        points: Vec<PointData>,
        has_normals: bool,
        has_covariances: bool,
    }

    impl AoSPointCloud {
        pub fn new() -> Self {
            Self {
                points: Vec::new(),
                has_normals: false,
                has_covariances: false,
            }
        }

        pub fn with_capacity(capacity: usize) -> Self {
            Self {
                points: Vec::with_capacity(capacity),
                has_normals: false,
                has_covariances: false,
            }
        }

        pub fn enable_normals(&mut self) {
            self.has_normals = true;
        }

        pub fn enable_covariances(&mut self) {
            self.has_covariances = true;
        }
    }

    impl PointCloudTrait for AoSPointCloud {
        fn size(&self) -> usize {
            self.points.len()
        }

        fn has_points(&self) -> bool {
            true
        }

        fn has_normals(&self) -> bool {
            self.has_normals
        }

        fn has_covariances(&self) -> bool {
            self.has_covariances
        }

        fn point(&self, index: usize) -> Point4<f64> {
            self.points[index].position
        }

        fn normal(&self, index: usize) -> Option<Vector4<f64>> {
            if self.has_normals {
                Some(self.points[index].normal)
            } else {
                None
            }
        }

        fn covariance(&self, index: usize) -> Option<Matrix4<f64>> {
            if self.has_covariances {
                Some(self.points[index].covariance)
            } else {
                None
            }
        }
    }

    impl MutablePointCloudTrait for AoSPointCloud {
        fn resize(&mut self, size: usize) {
            self.points.resize(size, PointData::default());
        }

        fn set_point(&mut self, index: usize, point: Point4<f64>) {
            self.points[index].position = point;
        }

        fn set_normal(&mut self, index: usize, normal: Vector4<f64>) {
            if self.has_normals {
                self.points[index].normal = normal;
            }
        }

        fn set_covariance(&mut self, index: usize, covariance: Matrix4<f64>) {
            if self.has_covariances {
                self.points[index].covariance = covariance;
            }
        }
    }

    // Test the AoS implementation
    let mut aos_cloud = AoSPointCloud::with_capacity(1000);
    aos_cloud.enable_normals();
    aos_cloud.enable_covariances();
    aos_cloud.resize(3);

    aos_cloud.set_point(0, helpers::point_from_xyz(1.0, 2.0, 3.0));
    aos_cloud.set_point(1, helpers::point_from_xyz(4.0, 5.0, 6.0));
    aos_cloud.set_point(2, helpers::point_from_xyz(7.0, 8.0, 9.0));

    aos_cloud.set_normal(0, helpers::normal_from_xyz(1.0, 0.0, 0.0));
    aos_cloud.set_normal(1, helpers::normal_from_xyz(0.0, 1.0, 0.0));
    aos_cloud.set_normal(2, helpers::normal_from_xyz(0.0, 0.0, 1.0));

    println!("AoS Point Cloud:");
    println!("  Size: {}", aos_cloud.size());
    println!("  Memory layout: Interleaved (position, normal, covariance per point)");
    println!("  Cache behavior: Good for random access, moderate for vectorization");

    for i in 0..aos_cloud.size() {
        let point = aos_cloud.point(i);
        println!(
            "  Point {}: ({:.1}, {:.1}, {:.1})",
            i, point.x, point.y, point.z
        );
    }

    // Use with generic algorithm
    let centroid = compute_centroid(&aos_cloud);
    println!(
        "  Centroid: ({:.1}, {:.1}, {:.1})",
        centroid.x, centroid.y, centroid.z
    );

    println!();
    Ok(())
}

/// Example 2: Structure of Arrays (SoA) Layout
///
/// This layout stores each attribute separately, which is optimal for
/// vectorized operations and algorithms that process single attributes.
fn demonstrate_soa_implementation() -> Result<()> {
    println!("2. Structure of Arrays (SoA) Implementation");
    println!("==========================================");

    #[derive(Debug)]
    pub struct SoAPointCloud {
        positions: Vec<Point4<f64>>,
        normals: Vec<Vector4<f64>>,
        covariances: Vec<Matrix4<f64>>,
        features_enabled: FeatureFlags,
    }

    #[derive(Debug, Clone, Copy)]
    struct FeatureFlags {
        normals: bool,
        covariances: bool,
    }

    impl SoAPointCloud {
        pub fn new() -> Self {
            Self {
                positions: Vec::new(),
                normals: Vec::new(),
                covariances: Vec::new(),
                features_enabled: FeatureFlags {
                    normals: false,
                    covariances: false,
                },
            }
        }

        pub fn with_features(normals: bool, covariances: bool) -> Self {
            Self {
                positions: Vec::new(),
                normals: Vec::new(),
                covariances: Vec::new(),
                features_enabled: FeatureFlags {
                    normals,
                    covariances,
                },
            }
        }

        /// Get direct access to position data for vectorized operations
        pub fn positions(&self) -> &[Point4<f64>] {
            &self.positions
        }

        /// Get mutable access to position data for vectorized operations
        pub fn positions_mut(&mut self) -> &mut [Point4<f64>] {
            &mut self.positions
        }

        /// Apply a transformation to all points using vectorized operations
        pub fn transform_vectorized(&mut self, transform: &Matrix4<f64>) {
            for position in &mut self.positions {
                *position = transform * *position;
            }
        }
    }

    impl PointCloudTrait for SoAPointCloud {
        fn size(&self) -> usize {
            self.positions.len()
        }

        fn has_points(&self) -> bool {
            true
        }

        fn has_normals(&self) -> bool {
            self.features_enabled.normals
        }

        fn has_covariances(&self) -> bool {
            self.features_enabled.covariances
        }

        fn point(&self, index: usize) -> Point4<f64> {
            self.positions[index]
        }

        fn normal(&self, index: usize) -> Option<Vector4<f64>> {
            if self.features_enabled.normals && index < self.normals.len() {
                Some(self.normals[index])
            } else {
                None
            }
        }

        fn covariance(&self, index: usize) -> Option<Matrix4<f64>> {
            if self.features_enabled.covariances && index < self.covariances.len() {
                Some(self.covariances[index])
            } else {
                None
            }
        }
    }

    impl MutablePointCloudTrait for SoAPointCloud {
        fn resize(&mut self, size: usize) {
            self.positions
                .resize(size, helpers::point_from_xyz(0.0, 0.0, 0.0));

            if self.features_enabled.normals {
                self.normals
                    .resize(size, helpers::normal_from_xyz(0.0, 0.0, 1.0));
            }

            if self.features_enabled.covariances {
                self.covariances.resize(size, Matrix4::identity());
            }
        }

        fn set_point(&mut self, index: usize, point: Point4<f64>) {
            self.positions[index] = point;
        }

        fn set_normal(&mut self, index: usize, normal: Vector4<f64>) {
            if self.features_enabled.normals {
                self.normals[index] = normal;
            }
        }

        fn set_covariance(&mut self, index: usize, covariance: Matrix4<f64>) {
            if self.features_enabled.covariances {
                self.covariances[index] = covariance;
            }
        }
    }

    // Test the SoA implementation
    let mut soa_cloud = SoAPointCloud::with_features(true, false);
    soa_cloud.resize(4);

    // Set points using direct array access (more efficient for bulk operations)
    let positions = soa_cloud.positions_mut();
    positions[0] = helpers::point_from_xyz(1.0, 0.0, 0.0);
    positions[1] = helpers::point_from_xyz(0.0, 1.0, 0.0);
    positions[2] = helpers::point_from_xyz(0.0, 0.0, 1.0);
    positions[3] = helpers::point_from_xyz(1.0, 1.0, 1.0);

    // Apply vectorized transformation
    let transform = Matrix4::new_scaling(2.0);
    soa_cloud.transform_vectorized(&transform);

    println!("SoA Point Cloud:");
    println!("  Size: {}", soa_cloud.size());
    println!("  Memory layout: Separate arrays per attribute");
    println!("  Cache behavior: Excellent for vectorization, moderate for random access");
    println!("  After 2x scaling transformation:");

    for i in 0..soa_cloud.size() {
        let point = soa_cloud.point(i);
        println!(
            "  Point {}: ({:.1}, {:.1}, {:.1})",
            i, point.x, point.y, point.z
        );
    }

    println!();
    Ok(())
}

/// Example 3: Minimal Point Cloud for Memory-Constrained Environments
///
/// This implementation uses the most compact representation possible,
/// suitable for embedded systems or large-scale processing.
fn demonstrate_minimal_implementation() -> Result<()> {
    println!("3. Minimal Memory Point Cloud Implementation");
    println!("===========================================");

    #[derive(Debug)]
    pub struct MinimalPointCloud {
        // Store as flat f32 array for minimal memory usage
        data: Vec<f32>,
        count: usize,
    }

    impl MinimalPointCloud {
        pub fn new() -> Self {
            Self {
                data: Vec::new(),
                count: 0,
            }
        }

        pub fn from_points(points: &[Vector3<f32>]) -> Self {
            let mut data = Vec::with_capacity(points.len() * 3);
            for point in points {
                data.push(point.x);
                data.push(point.y);
                data.push(point.z);
            }
            Self {
                data,
                count: points.len(),
            }
        }

        /// Memory usage in bytes
        pub fn memory_usage(&self) -> usize {
            self.data.len() * std::mem::size_of::<f32>() + std::mem::size_of::<Self>()
        }
    }

    impl PointCloudTrait for MinimalPointCloud {
        fn size(&self) -> usize {
            self.count
        }

        fn has_points(&self) -> bool {
            true
        }

        fn point(&self, index: usize) -> Point4<f64> {
            let base = index * 3;
            helpers::point_from_xyz(
                self.data[base] as f64,
                self.data[base + 1] as f64,
                self.data[base + 2] as f64,
            )
        }

        // No normals or covariances in minimal implementation
    }

    impl MutablePointCloudTrait for MinimalPointCloud {
        fn resize(&mut self, size: usize) {
            self.data.resize(size * 3, 0.0);
            self.count = size;
        }

        fn set_point(&mut self, index: usize, point: Point4<f64>) {
            let base = index * 3;
            self.data[base] = point.x as f32;
            self.data[base + 1] = point.y as f32;
            self.data[base + 2] = point.z as f32;
        }

        // No-op for normals and covariances
    }

    // Test minimal implementation
    let test_points = vec![
        Vector3::new(1.0f32, 2.0, 3.0),
        Vector3::new(4.0, 5.0, 6.0),
        Vector3::new(7.0, 8.0, 9.0),
    ];

    let minimal_cloud = MinimalPointCloud::from_points(&test_points);

    println!("Minimal Point Cloud:");
    println!("  Size: {}", minimal_cloud.size());
    println!("  Memory usage: {} bytes", minimal_cloud.memory_usage());
    println!("  Features: Positions only (f32 precision)");
    println!("  Use case: Large-scale processing, embedded systems");

    for i in 0..minimal_cloud.size() {
        let point = minimal_cloud.point(i);
        println!(
            "  Point {}: ({:.1}, {:.1}, {:.1})",
            i, point.x, point.y, point.z
        );
    }

    println!();
    Ok(())
}

/// Example 4: Specialized 2D Point Cloud
///
/// This implementation is optimized for planar data processing,
/// demonstrating how to create domain-specific point cloud types.
fn demonstrate_2d_implementation() -> Result<()> {
    println!("4. Specialized 2D Point Cloud Implementation");
    println!("===========================================");

    #[derive(Debug)]
    pub struct PointCloud2D {
        points: Vec<[f64; 2]>,
        z_plane: f64,
    }

    impl PointCloud2D {
        pub fn new(z_plane: f64) -> Self {
            Self {
                points: Vec::new(),
                z_plane,
            }
        }

        pub fn add_point(&mut self, x: f64, y: f64) {
            self.points.push([x, y]);
        }

        pub fn set_z_plane(&mut self, z: f64) {
            self.z_plane = z;
        }

        /// Compute 2D convex hull (simplified example)
        pub fn convex_hull_area(&self) -> f64 {
            // Simplified: assume rectangular bounds
            if self.points.is_empty() {
                return 0.0;
            }

            let min_x = self
                .points
                .iter()
                .map(|p| p[0])
                .fold(f64::INFINITY, f64::min);
            let max_x = self
                .points
                .iter()
                .map(|p| p[0])
                .fold(f64::NEG_INFINITY, f64::max);
            let min_y = self
                .points
                .iter()
                .map(|p| p[1])
                .fold(f64::INFINITY, f64::min);
            let max_y = self
                .points
                .iter()
                .map(|p| p[1])
                .fold(f64::NEG_INFINITY, f64::max);

            (max_x - min_x) * (max_y - min_y)
        }
    }

    impl PointCloudTrait for PointCloud2D {
        fn size(&self) -> usize {
            self.points.len()
        }

        fn has_points(&self) -> bool {
            true
        }

        fn point(&self, index: usize) -> Point4<f64> {
            let [x, y] = self.points[index];
            helpers::point_from_xyz(x, y, self.z_plane)
        }

        // 2D clouds typically don't have normals or covariances
    }

    impl MutablePointCloudTrait for PointCloud2D {
        fn resize(&mut self, size: usize) {
            self.points.resize(size, [0.0, 0.0]);
        }

        fn set_point(&mut self, index: usize, point: Point4<f64>) {
            self.points[index] = [point.x, point.y];
            // Note: Z coordinate is ignored (stays at z_plane)
        }
    }

    // Test 2D implementation
    let mut cloud_2d = PointCloud2D::new(5.0);
    cloud_2d.add_point(0.0, 0.0);
    cloud_2d.add_point(1.0, 0.0);
    cloud_2d.add_point(1.0, 1.0);
    cloud_2d.add_point(0.0, 1.0);

    println!("2D Point Cloud:");
    println!("  Size: {}", cloud_2d.size());
    println!("  Z-plane: {:.1}", cloud_2d.z_plane);
    println!("  Convex hull area: {:.1}", cloud_2d.convex_hull_area());
    println!("  Use case: Planar data, maps, floor plans");

    for i in 0..cloud_2d.size() {
        let point = cloud_2d.point(i);
        println!(
            "  Point {}: ({:.1}, {:.1}, {:.1})",
            i, point.x, point.y, point.z
        );
    }

    println!();
    Ok(())
}

/// Example 5: Builder Pattern for Point Cloud Construction
///
/// This demonstrates a fluent API for gradually constructing point clouds.
fn demonstrate_builder_pattern() -> Result<()> {
    println!("5. Builder Pattern Implementation");
    println!("================================");

    #[derive(Debug)]
    pub struct PointCloudBuilder {
        positions: Vec<Point4<f64>>,
        normals: Vec<Vector4<f64>>,
        covariances: Vec<Matrix4<f64>>,
        enable_normals: bool,
        enable_covariances: bool,
    }

    impl PointCloudBuilder {
        pub fn new() -> Self {
            Self {
                positions: Vec::new(),
                normals: Vec::new(),
                covariances: Vec::new(),
                enable_normals: false,
                enable_covariances: false,
            }
        }

        pub fn with_capacity(capacity: usize) -> Self {
            Self {
                positions: Vec::with_capacity(capacity),
                normals: Vec::with_capacity(capacity),
                covariances: Vec::with_capacity(capacity),
                enable_normals: false,
                enable_covariances: false,
            }
        }

        pub fn enable_normals(mut self) -> Self {
            self.enable_normals = true;
            self
        }

        pub fn enable_covariances(mut self) -> Self {
            self.enable_covariances = true;
            self
        }

        pub fn add_point(mut self, x: f64, y: f64, z: f64) -> Self {
            self.positions.push(helpers::point_from_xyz(x, y, z));

            if self.enable_normals {
                self.normals.push(helpers::normal_from_xyz(0.0, 0.0, 1.0));
            }

            if self.enable_covariances {
                self.covariances.push(Matrix4::identity());
            }

            self
        }

        pub fn add_point_with_normal(
            mut self,
            x: f64,
            y: f64,
            z: f64,
            nx: f64,
            ny: f64,
            nz: f64,
        ) -> Self {
            self.positions.push(helpers::point_from_xyz(x, y, z));

            if self.enable_normals {
                self.normals.push(helpers::normal_from_xyz(nx, ny, nz));
            } else {
                self.normals.push(helpers::normal_from_xyz(nx, ny, nz));
                self.enable_normals = true;
            }

            if self.enable_covariances {
                self.covariances.push(Matrix4::identity());
            }

            self
        }

        pub fn build(self) -> BuiltPointCloud {
            BuiltPointCloud {
                positions: self.positions,
                normals: if self.enable_normals {
                    Some(self.normals)
                } else {
                    None
                },
                covariances: if self.enable_covariances {
                    Some(self.covariances)
                } else {
                    None
                },
            }
        }
    }

    #[derive(Debug)]
    pub struct BuiltPointCloud {
        positions: Vec<Point4<f64>>,
        normals: Option<Vec<Vector4<f64>>>,
        covariances: Option<Vec<Matrix4<f64>>>,
    }

    impl PointCloudTrait for BuiltPointCloud {
        fn size(&self) -> usize {
            self.positions.len()
        }

        fn has_points(&self) -> bool {
            true
        }

        fn has_normals(&self) -> bool {
            self.normals.is_some()
        }

        fn has_covariances(&self) -> bool {
            self.covariances.is_some()
        }

        fn point(&self, index: usize) -> Point4<f64> {
            self.positions[index]
        }

        fn normal(&self, index: usize) -> Option<Vector4<f64>> {
            self.normals.as_ref().map(|normals| normals[index])
        }

        fn covariance(&self, index: usize) -> Option<Matrix4<f64>> {
            self.covariances
                .as_ref()
                .map(|covariances| covariances[index])
        }
    }

    // Note: BuiltPointCloud is immutable by design, so no MutablePointCloudTrait

    // Test builder pattern
    let built_cloud = PointCloudBuilder::new()
        .enable_normals()
        .add_point(1.0, 2.0, 3.0)
        .add_point_with_normal(4.0, 5.0, 6.0, 1.0, 0.0, 0.0)
        .add_point_with_normal(7.0, 8.0, 9.0, 0.0, 1.0, 0.0)
        .build();

    println!("Builder Pattern Point Cloud:");
    println!("  Size: {}", built_cloud.size());
    println!("  Has normals: {}", built_cloud.has_normals());
    println!("  Construction: Fluent API with method chaining");
    println!("  Use case: Gradual construction, configuration flexibility");

    for i in 0..built_cloud.size() {
        let point = built_cloud.point(i);
        let normal = built_cloud
            .normal(i)
            .unwrap_or_else(|| helpers::normal_from_xyz(0.0, 0.0, 1.0));
        println!(
            "  Point {}: ({:.1}, {:.1}, {:.1}), Normal: ({:.1}, {:.1}, {:.1})",
            i, point.x, point.y, point.z, normal.x, normal.y, normal.z
        );
    }

    println!();
    Ok(())
}

/// Example 6: Interoperability Between Different Implementations
///
/// This demonstrates how different custom implementations can work together
/// through the common trait interface.
fn demonstrate_interoperability() -> Result<()> {
    println!("6. Interoperability Between Implementations");
    println!("==========================================");

    // Create different implementations
    let mut aos_cloud = {
        use self::demonstrate_aos_implementation as _;
        // Re-define AoS for this scope
        #[derive(Debug, Clone)]
        struct AoSPointCloud {
            points: Vec<Point4<f64>>,
        }

        impl PointCloudTrait for AoSPointCloud {
            fn size(&self) -> usize {
                self.points.len()
            }
            fn has_points(&self) -> bool {
                true
            }
            fn point(&self, index: usize) -> Point4<f64> {
                self.points[index]
            }
        }

        impl MutablePointCloudTrait for AoSPointCloud {
            fn resize(&mut self, size: usize) {
                self.points
                    .resize(size, helpers::point_from_xyz(0.0, 0.0, 0.0));
            }
            fn set_point(&mut self, index: usize, point: Point4<f64>) {
                self.points[index] = point;
            }
        }

        let mut cloud = AoSPointCloud { points: Vec::new() };
        cloud.resize(2);
        cloud.set_point(0, helpers::point_from_xyz(1.0, 0.0, 0.0));
        cloud.set_point(1, helpers::point_from_xyz(0.0, 1.0, 0.0));
        cloud
    };

    let soa_cloud = {
        #[derive(Debug)]
        struct SoAPointCloud {
            positions: Vec<Point4<f64>>,
        }

        impl PointCloudTrait for SoAPointCloud {
            fn size(&self) -> usize {
                self.positions.len()
            }
            fn has_points(&self) -> bool {
                true
            }
            fn point(&self, index: usize) -> Point4<f64> {
                self.positions[index]
            }
        }

        SoAPointCloud {
            positions: vec![
                helpers::point_from_xyz(2.0, 0.0, 0.0),
                helpers::point_from_xyz(0.0, 2.0, 0.0),
            ],
        }
    };

    // Generic function that works with any point cloud implementation
    fn merge_clouds<T1: PointCloudTrait, T2: PointCloudTrait, T3: MutablePointCloudTrait>(
        cloud1: &T1,
        cloud2: &T2,
        output: &mut T3,
    ) {
        let total_size = cloud1.size() + cloud2.size();
        output.resize(total_size);

        // Copy from first cloud
        for i in 0..cloud1.size() {
            output.set_point(i, cloud1.point(i));
        }

        // Copy from second cloud
        for i in 0..cloud2.size() {
            output.set_point(cloud1.size() + i, cloud2.point(i));
        }
    }

    // Merge different implementations into another implementation
    let aos_cloud_copy = aos_cloud.clone();
    merge_clouds(&aos_cloud_copy, &soa_cloud, &mut aos_cloud);

    println!("Merged Point Cloud:");
    println!("  Total size: {}", aos_cloud.size());
    println!("  Source 1: AoS implementation (2 points)");
    println!("  Source 2: SoA implementation (2 points)");
    println!("  Target: AoS implementation");

    for i in 0..aos_cloud.size() {
        let point = aos_cloud.point(i);
        let source = if i < 2 { "AoS" } else { "SoA" };
        println!(
            "  Point {} (from {}): ({:.1}, {:.1}, {:.1})",
            i, source, point.x, point.y, point.z
        );
    }

    // Demonstrate conversion to C wrapper format
    let c_wrapper_cloud = conversions::from_trait(&aos_cloud)?;
    println!(
        "  Converted to C wrapper: {} points",
        c_wrapper_cloud.size()
    );

    println!();
    Ok(())
}

/// Generic helper function used in examples
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
