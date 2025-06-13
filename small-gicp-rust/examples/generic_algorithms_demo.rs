//! Demonstration of generic algorithms with KdTree and preprocessing.

use nalgebra::Point3;
use small_gicp_rust::{
    config::{DownsamplingConfig, KdTreeConfig},
    kdtree::{algorithms, KdTree},
    point_cloud::PointCloud,
    prelude::*,
    preprocessing::{Downsampling, PreprocessingStrategy},
};

fn main() -> Result<()> {
    println!("=== Generic Algorithms Demo ===\n");

    // Create test data
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.5, 0.5, 1.0),
    ];
    let cloud = PointCloud::from_points(&points)?;

    println!("Original cloud size: {}", cloud.size());

    // Test unified KdTree
    println!("\n--- Unified KdTree Test ---");
    let kdtree = KdTree::new(&cloud, &KdTreeConfig::default())?;
    println!("KdTree backend: {}", kdtree.backend_info());

    let query = Point3::new(0.1, 0.1, 0.0);
    let (index, distance) = kdtree.nearest_neighbor(query)?;
    println!(
        "Nearest neighbor to ({:.1}, {:.1}, {:.1}): index {}, distance {:.3}",
        query.x, query.y, query.z, index, distance
    );

    // Test generic algorithms
    println!("\n--- Generic Algorithms Test ---");
    let (closest_idx, closest_point, sq_dist) = algorithms::find_closest_point_generic(
        &cloud,
        &kdtree,
        small_gicp_rust::traits::helpers::point_from_xyz(0.1, 0.1, 0.0),
    )?;
    println!(
        "Closest point: index {}, point ({:.1}, {:.1}, {:.1}), sq_distance {:.3}",
        closest_idx, closest_point.x, closest_point.y, closest_point.z, sq_dist
    );

    // Test K-NN
    let knn_results = algorithms::find_knn_points_generic(
        &cloud,
        &kdtree,
        small_gicp_rust::traits::helpers::point_from_xyz(0.5, 0.5, 0.0),
        3,
    )?;
    println!("3-NN results:");
    for (i, (idx, point, dist)) in knn_results.iter().enumerate() {
        println!(
            "  {}: index {}, point ({:.1}, {:.1}, {:.1}), sq_distance {:.3}",
            i, idx, point.x, point.y, point.z, dist
        );
    }

    // Test generic downsampling
    println!("\n--- Generic Downsampling Test ---");
    let config = DownsamplingConfig::default();

    let downsampled =
        Downsampling::voxel_grid(&cloud, 0.5, &config, PreprocessingStrategy::CWrapper)?;
    println!(
        "Downsampled cloud size (voxel_grid): {}",
        downsampled.size()
    );

    let random_sampled =
        Downsampling::random_sampling(&cloud, 3, &config, PreprocessingStrategy::PureRust)?;
    println!("Random sampled cloud size: {}", random_sampled.size());

    println!("\nDemo completed successfully!");
    Ok(())
}
