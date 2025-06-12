//! Performance comparison between C wrapper and custom point cloud implementations.
//!
//! This example provides benchmarks and analysis to help you choose the right
//! point cloud implementation for your specific use case.

use nalgebra::{Matrix4, Vector3, Vector4};
use small_gicp_rust::{
    prelude::*,
    traits::{helpers, MutablePointCloudTrait, Point4, PointCloudTrait},
};
use std::time::Instant;

fn main() -> Result<()> {
    println!("=== Point Cloud Implementation Performance Comparison ===\n");

    const SIZES: &[usize] = &[1_000, 10_000, 100_000, 1_000_000];

    for &size in SIZES {
        println!("Benchmarking with {} points:", size);
        println!("----------------------------------------");

        // 1. Memory usage comparison
        compare_memory_usage(size)?;

        // 2. Creation performance
        compare_creation_performance(size)?;

        // 3. Access patterns performance
        compare_access_patterns(size)?;

        // 4. Transformation performance
        compare_transformation_performance(size)?;

        // 5. Generic algorithm performance
        compare_generic_algorithms(size)?;

        println!();
    }

    // Overall recommendations
    print_recommendations();

    Ok(())
}

/// Compare memory usage between different implementations
fn compare_memory_usage(size: usize) -> Result<()> {
    println!("Memory Usage Comparison:");

    // C wrapper implementation
    let mut c_cloud = PointCloud::new()?;
    c_cloud.resize(size)?;
    let c_memory = estimate_c_wrapper_memory(&c_cloud);

    // AoS implementation
    let aos_cloud = create_aos_cloud(size);
    let aos_memory = aos_cloud.memory_usage();

    // SoA implementation
    let soa_cloud = create_soa_cloud(size);
    let soa_memory = soa_cloud.memory_usage();

    // Minimal implementation
    let minimal_cloud = create_minimal_cloud(size);
    let minimal_memory = minimal_cloud.memory_usage();

    println!(
        "  C Wrapper:     {:8} bytes ({:6.2} bytes/point)",
        c_memory,
        c_memory as f64 / size as f64
    );
    println!(
        "  AoS Custom:    {:8} bytes ({:6.2} bytes/point)",
        aos_memory,
        aos_memory as f64 / size as f64
    );
    println!(
        "  SoA Custom:    {:8} bytes ({:6.2} bytes/point)",
        soa_memory,
        soa_memory as f64 / size as f64
    );
    println!(
        "  Minimal:       {:8} bytes ({:6.2} bytes/point)",
        minimal_memory,
        minimal_memory as f64 / size as f64
    );

    let baseline = c_memory as f64;
    println!("  Memory efficiency vs C wrapper:");
    println!("    AoS: {:.1}x", aos_memory as f64 / baseline);
    println!("    SoA: {:.1}x", soa_memory as f64 / baseline);
    println!("    Minimal: {:.1}x", minimal_memory as f64 / baseline);

    Ok(())
}

/// Compare point cloud creation performance
fn compare_creation_performance(size: usize) -> Result<()> {
    println!("Creation Performance:");

    let test_points: Vec<Vector3<f64>> = (0..size)
        .map(|i| Vector3::new(i as f64, (i * 2) as f64, (i * 3) as f64))
        .collect();

    // C wrapper creation
    let start = Instant::now();
    let mut c_cloud = PointCloud::new()?;
    c_cloud.resize(size)?;
    for (i, point) in test_points.iter().enumerate() {
        c_cloud.set_point(i, nalgebra::Point3::new(point.x, point.y, point.z))?;
    }
    let c_duration = start.elapsed();

    // AoS creation
    let start = Instant::now();
    let mut aos_cloud = create_aos_cloud(0);
    aos_cloud.resize(size);
    for (i, point) in test_points.iter().enumerate() {
        aos_cloud.set_point(i, helpers::point_from_vector3(*point));
    }
    let aos_duration = start.elapsed();

    // SoA creation
    let start = Instant::now();
    let mut soa_cloud = create_soa_cloud(0);
    soa_cloud.resize(size);
    for (i, point) in test_points.iter().enumerate() {
        soa_cloud.set_point(i, helpers::point_from_vector3(*point));
    }
    let soa_duration = start.elapsed();

    // SoA bulk creation (optimized)
    let start = Instant::now();
    let soa_bulk_cloud = create_soa_cloud_bulk(&test_points);
    let soa_bulk_duration = start.elapsed();

    println!(
        "  C Wrapper:     {:8.2}ms ({:6.2}ns/point)",
        c_duration.as_nanos() as f64 / 1_000_000.0,
        c_duration.as_nanos() as f64 / size as f64
    );
    println!(
        "  AoS:           {:8.2}ms ({:6.2}ns/point)",
        aos_duration.as_nanos() as f64 / 1_000_000.0,
        aos_duration.as_nanos() as f64 / size as f64
    );
    println!(
        "  SoA:           {:8.2}ms ({:6.2}ns/point)",
        soa_duration.as_nanos() as f64 / 1_000_000.0,
        soa_duration.as_nanos() as f64 / size as f64
    );
    println!(
        "  SoA (bulk):    {:8.2}ms ({:6.2}ns/point)",
        soa_bulk_duration.as_nanos() as f64 / 1_000_000.0,
        soa_bulk_duration.as_nanos() as f64 / size as f64
    );

    let baseline = c_duration.as_nanos() as f64;
    println!("  Speed vs C wrapper:");
    println!("    AoS: {:.2}x", baseline / aos_duration.as_nanos() as f64);
    println!("    SoA: {:.2}x", baseline / soa_duration.as_nanos() as f64);
    println!(
        "    SoA (bulk): {:.2}x",
        baseline / soa_bulk_duration.as_nanos() as f64
    );

    Ok(())
}

/// Compare different access patterns
fn compare_access_patterns(size: usize) -> Result<()> {
    println!("Access Pattern Performance:");

    // Create test clouds
    let c_cloud = create_c_cloud_with_data(size)?;
    let aos_cloud = create_aos_cloud_with_data(size);
    let soa_cloud = create_soa_cloud_with_data(size);

    // Sequential access test
    println!("  Sequential access (sum all coordinates):");

    let start = Instant::now();
    let mut sum = 0.0f64;
    for i in 0..c_cloud.size() {
        let point = c_cloud.point(i);
        sum += point.x + point.y + point.z;
    }
    let c_seq_duration = start.elapsed();

    let start = Instant::now();
    let mut sum = 0.0f64;
    for i in 0..aos_cloud.size() {
        let point = aos_cloud.point(i);
        sum += point.x + point.y + point.z;
    }
    let aos_seq_duration = start.elapsed();

    let start = Instant::now();
    let mut sum = 0.0f64;
    for i in 0..soa_cloud.size() {
        let point = soa_cloud.point(i);
        sum += point.x + point.y + point.z;
    }
    let soa_seq_duration = start.elapsed();

    // SoA optimized access
    let start = Instant::now();
    let mut sum = 0.0f64;
    for position in soa_cloud.positions() {
        sum += position.x + position.y + position.z;
    }
    let soa_opt_duration = start.elapsed();

    println!(
        "    C Wrapper:   {:8.2}ms",
        c_seq_duration.as_nanos() as f64 / 1_000_000.0
    );
    println!(
        "    AoS:         {:8.2}ms",
        aos_seq_duration.as_nanos() as f64 / 1_000_000.0
    );
    println!(
        "    SoA:         {:8.2}ms",
        soa_seq_duration.as_nanos() as f64 / 1_000_000.0
    );
    println!(
        "    SoA (opt):   {:8.2}ms",
        soa_opt_duration.as_nanos() as f64 / 1_000_000.0
    );

    // Random access test
    println!("  Random access (every 7th point):");

    let indices: Vec<usize> = (0..size).step_by(7).collect();

    let start = Instant::now();
    let mut sum = 0.0f64;
    for &i in &indices {
        let point = c_cloud.point(i);
        sum += point.x + point.y + point.z;
    }
    let c_rand_duration = start.elapsed();

    let start = Instant::now();
    let mut sum = 0.0f64;
    for &i in &indices {
        let point = aos_cloud.point(i);
        sum += point.x + point.y + point.z;
    }
    let aos_rand_duration = start.elapsed();

    let start = Instant::now();
    let mut sum = 0.0f64;
    for &i in &indices {
        let point = soa_cloud.point(i);
        sum += point.x + point.y + point.z;
    }
    let soa_rand_duration = start.elapsed();

    println!(
        "    C Wrapper:   {:8.2}ms",
        c_rand_duration.as_nanos() as f64 / 1_000_000.0
    );
    println!(
        "    AoS:         {:8.2}ms",
        aos_rand_duration.as_nanos() as f64 / 1_000_000.0
    );
    println!(
        "    SoA:         {:8.2}ms",
        soa_rand_duration.as_nanos() as f64 / 1_000_000.0
    );

    Ok(())
}

/// Compare transformation performance
fn compare_transformation_performance(size: usize) -> Result<()> {
    println!("Transformation Performance (translate all points):");

    let mut c_cloud = create_c_cloud_with_data(size)?;
    let mut aos_cloud = create_aos_cloud_with_data(size);
    let mut soa_cloud = create_soa_cloud_with_data(size);

    let offset = Vector3::new(1.0, 2.0, 3.0);

    // C wrapper transformation
    let start = Instant::now();
    for i in 0..c_cloud.size() {
        let mut point = c_cloud.get_point(i).unwrap();
        point.x += offset.x;
        point.y += offset.y;
        point.z += offset.z;
        c_cloud.set_point(i, point).unwrap();
    }
    let c_duration = start.elapsed();

    // AoS transformation (using trait interface)
    let start = Instant::now();
    for i in 0..aos_cloud.size() {
        let mut point = aos_cloud.point(i);
        point.x += offset.x;
        point.y += offset.y;
        point.z += offset.z;
        aos_cloud.set_point(i, point);
    }
    let aos_duration = start.elapsed();

    // SoA transformation (using trait interface)
    let start = Instant::now();
    for i in 0..soa_cloud.size() {
        let mut point = soa_cloud.point(i);
        point.x += offset.x;
        point.y += offset.y;
        point.z += offset.z;
        soa_cloud.set_point(i, point);
    }
    let soa_duration = start.elapsed();

    // SoA optimized transformation
    let start = Instant::now();
    soa_cloud.translate_all(offset);
    let soa_opt_duration = start.elapsed();

    println!(
        "  C Wrapper:     {:8.2}ms",
        c_duration.as_nanos() as f64 / 1_000_000.0
    );
    println!(
        "  AoS:           {:8.2}ms",
        aos_duration.as_nanos() as f64 / 1_000_000.0
    );
    println!(
        "  SoA:           {:8.2}ms",
        soa_duration.as_nanos() as f64 / 1_000_000.0
    );
    println!(
        "  SoA (opt):     {:8.2}ms",
        soa_opt_duration.as_nanos() as f64 / 1_000_000.0
    );

    let baseline = c_duration.as_nanos() as f64;
    println!("  Speed vs C wrapper:");
    println!("    AoS: {:.2}x", baseline / aos_duration.as_nanos() as f64);
    println!("    SoA: {:.2}x", baseline / soa_duration.as_nanos() as f64);
    println!(
        "    SoA (opt): {:.2}x",
        baseline / soa_opt_duration.as_nanos() as f64
    );

    Ok(())
}

/// Compare performance with generic algorithms
fn compare_generic_algorithms(size: usize) -> Result<()> {
    println!("Generic Algorithm Performance (centroid calculation):");

    let c_cloud = create_c_cloud_with_data(size)?;
    let aos_cloud = create_aos_cloud_with_data(size);
    let soa_cloud = create_soa_cloud_with_data(size);

    fn compute_centroid<P: PointCloudTrait>(cloud: &P) -> Point4<f64> {
        let mut sum = Vector4::zeros();
        for i in 0..cloud.size() {
            sum += cloud.point(i);
        }
        sum / cloud.size() as f64
    }

    // Test with each implementation
    let start = Instant::now();
    let _centroid = compute_centroid(&c_cloud);
    let c_duration = start.elapsed();

    let start = Instant::now();
    let _centroid = compute_centroid(&aos_cloud);
    let aos_duration = start.elapsed();

    let start = Instant::now();
    let _centroid = compute_centroid(&soa_cloud);
    let soa_duration = start.elapsed();

    println!(
        "  C Wrapper:     {:8.2}ms",
        c_duration.as_nanos() as f64 / 1_000_000.0
    );
    println!(
        "  AoS:           {:8.2}ms",
        aos_duration.as_nanos() as f64 / 1_000_000.0
    );
    println!(
        "  SoA:           {:8.2}ms",
        soa_duration.as_nanos() as f64 / 1_000_000.0
    );

    let baseline = c_duration.as_nanos() as f64;
    println!("  Speed vs C wrapper:");
    println!("    AoS: {:.2}x", baseline / aos_duration.as_nanos() as f64);
    println!("    SoA: {:.2}x", baseline / soa_duration.as_nanos() as f64);

    Ok(())
}

fn print_recommendations() {
    println!("=== RECOMMENDATIONS ===");
    println!();

    println!("📊 MEMORY EFFICIENCY:");
    println!("  🥇 Minimal Implementation: Best for memory-constrained environments");
    println!("  🥈 SoA Custom: Good balance of memory and flexibility");
    println!("  🥉 AoS Custom: Moderate memory usage, good cache locality");
    println!("  📝 C Wrapper: Higher memory overhead but proven implementation");
    println!();

    println!("⚡ PERFORMANCE:");
    println!("  📦 Array of Structures (AoS):");
    println!("    ✅ Best for: Random access, mixed attribute access");
    println!("    ✅ Good cache locality for point-wise operations");
    println!("    ❌ Poor for vectorized operations on single attributes");
    println!();

    println!("  🔧 Structure of Arrays (SoA):");
    println!("    ✅ Best for: Bulk operations, transformations, filtering");
    println!("    ✅ Excellent for vectorization and SIMD");
    println!("    ❌ Poor cache locality for mixed attribute access");
    println!();

    println!("  🛡️ C Wrapper:");
    println!("    ✅ Best for: Production reliability, feature completeness");
    println!("    ✅ Proven algorithms and optimizations");
    println!("    ❌ Higher memory overhead and FFI costs");
    println!();

    println!("🎯 USE CASE RECOMMENDATIONS:");
    println!();

    println!("  🌟 **C Wrapper PointCloud** (Recommended Default):");
    println!("    • General-purpose point cloud processing");
    println!("    • When you need all features (normals, covariances)");
    println!("    • Production applications requiring reliability");
    println!("    • Integration with existing small_gicp algorithms");
    println!();

    println!("  ⚡ **SoA Custom Implementation**:");
    println!("    • High-performance bulk operations");
    println!("    • Real-time processing pipelines");
    println!("    • Filtering and transformation-heavy workflows");
    println!("    • When you can optimize access patterns");
    println!();

    println!("  🎮 **AoS Custom Implementation**:");
    println!("    • Random access patterns");
    println!("    • Mixed attribute processing per point");
    println!("    • Simple, straightforward data access");
    println!("    • When cache locality matters most");
    println!();

    println!("  💾 **Minimal Implementation**:");
    println!("    • Embedded systems or strict memory limits");
    println!("    • Large-scale processing (millions of points)");
    println!("    • Position-only processing");
    println!("    • Network transmission or storage");
    println!();

    println!("  🔬 **Specialized Implementations**:");
    println!("    • Domain-specific needs (LiDAR, RGB-D, stereo)");
    println!("    • Custom attributes or metadata");
    println!("    • Integration with existing data structures");
    println!("    • Performance-critical applications");
    println!();

    println!("📋 DECISION MATRIX:");
    println!("  If memory is primary concern      → Minimal Implementation");
    println!("  If reliability is primary concern → C Wrapper");
    println!("  If speed is primary concern       → SoA Custom + optimizations");
    println!("  If simplicity is primary concern  → AoS Custom");
    println!("  If features are primary concern   → C Wrapper");
    println!("  If customization is needed        → Custom Implementation");
}

// Helper functions to create different implementations

#[derive(Debug)]
struct AoSPointCloud {
    positions: Vec<Point4<f64>>,
}

impl AoSPointCloud {
    fn new() -> Self {
        Self {
            positions: Vec::new(),
        }
    }

    fn memory_usage(&self) -> usize {
        std::mem::size_of::<Self>() + self.positions.capacity() * std::mem::size_of::<Point4<f64>>()
    }
}

impl PointCloudTrait for AoSPointCloud {
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

impl MutablePointCloudTrait for AoSPointCloud {
    fn resize(&mut self, size: usize) {
        self.positions
            .resize(size, helpers::point_from_xyz(0.0, 0.0, 0.0));
    }
    fn set_point(&mut self, index: usize, point: Point4<f64>) {
        self.positions[index] = point;
    }
}

#[derive(Debug)]
struct SoAPointCloud {
    positions: Vec<Point4<f64>>,
}

impl SoAPointCloud {
    fn new() -> Self {
        Self {
            positions: Vec::new(),
        }
    }

    fn memory_usage(&self) -> usize {
        std::mem::size_of::<Self>() + self.positions.capacity() * std::mem::size_of::<Point4<f64>>()
    }

    fn positions(&self) -> &[Point4<f64>] {
        &self.positions
    }

    fn translate_all(&mut self, offset: Vector3<f64>) {
        for pos in &mut self.positions {
            pos.x += offset.x;
            pos.y += offset.y;
            pos.z += offset.z;
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
    fn point(&self, index: usize) -> Point4<f64> {
        self.positions[index]
    }
}

impl MutablePointCloudTrait for SoAPointCloud {
    fn resize(&mut self, size: usize) {
        self.positions
            .resize(size, helpers::point_from_xyz(0.0, 0.0, 0.0));
    }
    fn set_point(&mut self, index: usize, point: Point4<f64>) {
        self.positions[index] = point;
    }
}

#[derive(Debug)]
struct MinimalPointCloud {
    data: Vec<f32>,
    count: usize,
}

impl MinimalPointCloud {
    fn new() -> Self {
        Self {
            data: Vec::new(),
            count: 0,
        }
    }

    fn memory_usage(&self) -> usize {
        std::mem::size_of::<Self>() + self.data.capacity() * std::mem::size_of::<f32>()
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
}

fn create_aos_cloud(size: usize) -> AoSPointCloud {
    let mut cloud = AoSPointCloud::new();
    cloud.resize(size);
    cloud
}

fn create_soa_cloud(size: usize) -> SoAPointCloud {
    let mut cloud = SoAPointCloud::new();
    cloud.resize(size);
    cloud
}

fn create_minimal_cloud(size: usize) -> MinimalPointCloud {
    let mut cloud = MinimalPointCloud::new();
    cloud.resize(size);
    cloud
}

fn create_soa_cloud_bulk(points: &[Vector3<f64>]) -> SoAPointCloud {
    SoAPointCloud {
        positions: points
            .iter()
            .map(|p| helpers::point_from_vector3(*p))
            .collect(),
    }
}

fn create_c_cloud_with_data(size: usize) -> Result<PointCloud> {
    let mut cloud = PointCloud::new()?;
    cloud.resize(size)?;
    for i in 0..size {
        let point = nalgebra::Point3::new(i as f64, (i * 2) as f64, (i * 3) as f64);
        cloud.set_point(i, point)?;
    }
    Ok(cloud)
}

fn create_aos_cloud_with_data(size: usize) -> AoSPointCloud {
    let mut cloud = create_aos_cloud(size);
    for i in 0..size {
        let point = helpers::point_from_xyz(i as f64, (i * 2) as f64, (i * 3) as f64);
        cloud.set_point(i, point);
    }
    cloud
}

fn create_soa_cloud_with_data(size: usize) -> SoAPointCloud {
    let mut cloud = create_soa_cloud(size);
    for i in 0..size {
        let point = helpers::point_from_xyz(i as f64, (i * 2) as f64, (i * 3) as f64);
        cloud.set_point(i, point);
    }
    cloud
}

fn estimate_c_wrapper_memory(cloud: &PointCloud) -> usize {
    // Rough estimate: C++ PointCloud overhead + point data
    let base_size = 1024; // Estimated C++ object overhead
    let point_size = 4 * std::mem::size_of::<f64>(); // 4D point
    base_size + cloud.size() * point_size
}
