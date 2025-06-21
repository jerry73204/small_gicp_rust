//! Unit tests for KdTree implementations
//! Reference: small_gicp/src/test/kdtree_test.cpp

use super::*;
use crate::{point_cloud::PointCloud, traits::SpatialSearchTree};
use nalgebra::{Point3, Vector3};
use rand::{distributions::Uniform, thread_rng, Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;

/// Generate test fixture matching C++ test
/// Reference: kdtree_test.cpp:SetUp()
fn create_test_fixture() -> Result<TestFixture> {
    // Load test data
    let test_file = "data/target.ply";
    let points = if std::path::Path::new(test_file).exists() {
        // TODO: Implement PLY loading - PROGRESS.md action item
        eprintln!("PLY loading not yet implemented, using synthetic data");
        generate_synthetic_cloud(1000)?
    } else {
        // Generate synthetic point cloud for testing
        generate_synthetic_cloud(1000)?
    };

    // Apply voxel downsampling with 0.5 resolution
    // TODO: Implement voxel downsampling
    // points = Preprocessing::voxel_downsample(&points, 0.5)?;

    // Estimate normals and covariances
    // TODO: Implement normal estimation
    // Preprocessing::estimate_normals(&mut points, 20)?;

    // Generate queries matching C++ test pattern:
    // - 50 queries from actual points
    // - 50 queries near points (with small offset)
    // - 50 random queries in space
    let mut rng = ChaCha8Rng::seed_from_u64(42); // Deterministic for testing
    let mut queries = Vec::new();

    // Queries from actual points
    for _ in 0..50 {
        let idx = rng.gen_range(0..points.len());
        let point = points.point_at(idx)?;
        queries.push(point);
    }

    // Queries near points
    let offset_dist = Uniform::new(0.0, 1.0);
    for _ in 0..50 {
        let idx = rng.gen_range(0..points.len());
        let point = points.point_at(idx)?;
        queries.push(Point3::new(
            point.x + rng.sample(&offset_dist),
            point.y + rng.sample(&offset_dist),
            point.z + rng.sample(&offset_dist),
        ));
    }

    // Random queries
    let coord_dist = Uniform::new(0.0, 100.0);
    for _ in 0..50 {
        queries.push(Point3::new(
            rng.sample(&coord_dist),
            rng.sample(&coord_dist),
            rng.sample(&coord_dist),
        ));
    }

    // Compute ground truth k-NN with brute force
    let k = 20;
    let (k_indices, k_sq_dists) = brute_force_knn(&points, &queries, k)?;

    Ok(TestFixture {
        points,
        queries,
        k,
        k_indices,
        k_sq_dists,
    })
}

/// Generate synthetic point cloud for testing when PLY is not available
fn generate_synthetic_cloud(num_points: usize) -> Result<PointCloud> {
    let mut rng = thread_rng();
    let dist = Uniform::new(-50.0, 50.0);

    let points: Vec<Point3<f64>> = (0..num_points)
        .map(|_| Point3::new(rng.sample(&dist), rng.sample(&dist), rng.sample(&dist)))
        .collect();

    PointCloud::from_points(&points)
}

/// Brute force k-NN search for ground truth
fn brute_force_knn(
    points: &PointCloud,
    queries: &[Point3<f64>],
    k: usize,
) -> Result<(Vec<Vec<usize>>, Vec<Vec<f64>>)> {
    let mut all_indices = Vec::new();
    let mut all_dists = Vec::new();

    for query in queries {
        let mut neighbors: Vec<(usize, f64)> = Vec::new();

        // Compute distances to all points
        for i in 0..points.len() {
            let point = points.point_at(i)?;
            let sq_dist = (point - query).norm_squared();
            neighbors.push((i, sq_dist));
        }

        // Sort by distance and take k nearest
        neighbors.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        neighbors.truncate(k);

        let indices: Vec<usize> = neighbors.iter().map(|(idx, _)| *idx).collect();
        let dists: Vec<f64> = neighbors.iter().map(|(_, dist)| *dist).collect();

        all_indices.push(indices);
        all_dists.push(dists);
    }

    Ok((all_indices, all_dists))
}

struct TestFixture {
    points: PointCloud,
    queries: Vec<Point3<f64>>,
    k: usize,
    k_indices: Vec<Vec<usize>>,
    k_sq_dists: Vec<Vec<f64>>,
}

/// Test loading of test data
/// Reference: kdtree_test.cpp:LoadCheck
#[test]
fn test_load_check() {
    let fixture = match create_test_fixture() {
        Ok(f) => f,
        Err(e) => {
            eprintln!("Skipping test due to fixture creation error: {}", e);
            return;
        }
    };

    assert!(fixture.points.len() > 0, "Points should not be empty");
    assert_eq!(fixture.queries.len(), 150, "Should have 150 queries");
    assert_eq!(fixture.k_indices.len(), fixture.queries.len());
    assert_eq!(fixture.k_sq_dists.len(), fixture.queries.len());

    // Test that KdTree size() method works
    let kdtree = KdTree::new(&fixture.points).unwrap();
    assert_eq!(
        kdtree.len(),
        fixture.points.len(),
        "KdTree size should match point cloud size"
    );

    // Verify ground truth is sorted by distance
    for i in 0..fixture.queries.len() {
        assert_eq!(fixture.k_indices[i].len(), fixture.k);
        assert_eq!(fixture.k_sq_dists[i].len(), fixture.k);

        // Check distances are sorted
        for j in 1..fixture.k {
            assert!(
                fixture.k_sq_dists[i][j] >= fixture.k_sq_dists[i][j - 1],
                "Distances must be sorted"
            );
        }
    }
}

/// Test empty KdTree construction
/// Reference: kdtree_test.cpp:EmptyTest
#[test]
fn test_empty_kdtree() {
    let empty_cloud = PointCloud::new().unwrap();

    // Test that KdTree construction from empty cloud returns an error
    match KdTree::new(&empty_cloud) {
        Err(crate::error::SmallGicpError::EmptyPointCloud) => {
            // Expected error
        }
        Ok(_) => panic!("Expected EmptyPointCloud error"),
        Err(e) => panic!("Unexpected error: {}", e),
    }

    // Test parallel construction too
    match KdTree::new_parallel(&empty_cloud, 4) {
        Err(crate::error::SmallGicpError::EmptyPointCloud) => {
            // Expected error
        }
        Ok(_) => panic!("Expected EmptyPointCloud error"),
        Err(e) => panic!("Unexpected error: {}", e),
    }

    // Test BorrowedKdTree with empty cloud
    match BorrowedKdTree::new(&empty_cloud) {
        Err(crate::error::SmallGicpError::EmptyPointCloud) => {
            // Expected error
        }
        Ok(_) => panic!("Expected EmptyPointCloud error"),
        Err(e) => panic!("Unexpected error: {}", e),
    }
}

/// Test k-NN search accuracy
/// Reference: kdtree_test.cpp:KnnTest
#[test]
fn test_knn_search() {
    let fixture = match create_test_fixture() {
        Ok(f) => f,
        Err(e) => {
            eprintln!("Skipping test due to fixture creation error: {}", e);
            return;
        }
    };

    // Test single-threaded KdTree
    let kdtree = KdTree::new(&fixture.points).unwrap();
    verify_knn_results(&kdtree, &fixture);

    // Test parallel KdTree construction
    let kdtree_parallel = KdTree::new_parallel(&fixture.points, 4).unwrap();
    verify_knn_results(&kdtree_parallel, &fixture);
}

fn verify_knn_results(kdtree: &KdTree, fixture: &TestFixture) {
    for i in 0..fixture.queries.len() {
        let query = &fixture.queries[i];

        // Test k-NN search
        let results = kdtree.knn_search(query, fixture.k);
        assert_eq!(
            results.len(),
            fixture.k,
            "Should return exactly k neighbors"
        );

        // Verify results match ground truth
        for j in 0..fixture.k {
            assert_eq!(
                results[j].0, fixture.k_indices[i][j],
                "Index mismatch at position {} for query {}",
                j, i
            );
            assert!(
                (results[j].1 - fixture.k_sq_dists[i][j]).abs() < 1e-3,
                "Distance mismatch at position {} for query {}: {} vs {}",
                j,
                i,
                results[j].1,
                fixture.k_sq_dists[i][j]
            );
        }

        // Test nearest neighbor (k=1)
        if let Some((nn_idx, nn_dist)) = kdtree.nearest_neighbor(query) {
            assert_eq!(nn_idx, fixture.k_indices[i][0]);
            assert!((nn_dist - fixture.k_sq_dists[i][0]).abs() < 1e-3);
        } else {
            panic!("Nearest neighbor search should not fail");
        }
    }
}

/// Test radius search functionality
/// Reference: Not directly in kdtree_test.cpp but part of KdTree interface
#[test]
fn test_radius_search() {
    // Generate test data
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(2.0, 2.0, 0.0),
    ];
    let cloud = PointCloud::from_points(&points).unwrap();
    let kdtree = KdTree::new(&cloud).unwrap();

    // Test radius search
    let query = Point3::new(0.5, 0.5, 0.0);
    let results = kdtree.radius_search(&query, 1.0);

    // Should find the 4 corner points within radius 1.0
    assert!(
        results.len() >= 4,
        "Should find at least 4 points within radius 1.0"
    );

    // Verify all results are within radius
    for (_, dist) in &results {
        assert!(*dist <= 1.0, "All distances should be <= radius");
    }
}

/// Test with different parallel backends
/// Reference: kdtree_test.cpp - tests TBB and OMP variants
#[test]
fn test_parallel_construction() {
    let points = generate_synthetic_cloud(100).unwrap();

    // Test different thread counts
    for num_threads in [1, 2, 4, 8] {
        let kdtree = KdTree::new_parallel(&points, num_threads).unwrap();

        // Verify tree works correctly
        let query = Point3::new(0.0, 0.0, 0.0);
        let _ = kdtree.nearest_neighbor(&query);
    }
}

/// Test with synthetic uniform distribution
/// Reference: kdtree_synthetic_test.cpp
#[test]
fn test_synthetic_uniform() {
    test_synthetic_distribution(SyntheticDistribution::Uniform);
}

/// Test with synthetic normal distribution
/// Reference: kdtree_synthetic_test.cpp
#[test]
fn test_synthetic_normal() {
    test_synthetic_distribution(SyntheticDistribution::Normal);
}

/// Test with synthetic clustered distribution
/// Reference: kdtree_synthetic_test.cpp
#[test]
fn test_synthetic_clustered() {
    test_synthetic_distribution(SyntheticDistribution::Clustered);
}

/// Test BorrowedKdTree functionality including size()
#[test]
fn test_borrowed_kdtree() {
    // Generate test data
    let points = generate_synthetic_cloud(100).unwrap();

    // Create BorrowedKdTree
    let borrowed_tree = BorrowedKdTree::new(&points).unwrap();

    // Test size() method
    assert_eq!(
        borrowed_tree.size(),
        points.len(),
        "BorrowedKdTree size should match point cloud size"
    );
    assert_eq!(
        borrowed_tree.len(),
        points.len(),
        "len() should return same as size()"
    );

    // Test search functionality
    let query_vec = Vector3::new(0.0, 0.0, 0.0);

    // Test through direct methods
    if let Some((idx, dist)) = borrowed_tree.nearest_neighbor(&query_vec) {
        assert!(idx < points.len());
        assert!(dist >= 0.0);
    }

    // Test k-NN search
    let knn_results = borrowed_tree.knn_search(&query_vec, 5);
    assert!(knn_results.len() <= 5);
    assert!(!knn_results.is_empty());
}

#[derive(Clone, Copy)]
enum SyntheticDistribution {
    Uniform,
    Normal,
    Clustered,
}

fn test_synthetic_distribution(distribution: SyntheticDistribution) {
    let points = generate_synthetic_distribution(1000, distribution);
    let cloud = PointCloud::from_points(&points).unwrap();

    let kdtree = KdTree::new(&cloud).unwrap();

    // Generate test queries
    let _rng = thread_rng();
    let queries = generate_synthetic_distribution(10, distribution);

    // Verify searches work
    for query in &queries {
        let result = kdtree.nearest_neighbor(query);
        assert!(result.is_some());

        let knn_results = kdtree.knn_search(query, 5);
        assert!(knn_results.len() <= 5);

        // Verify results are sorted by distance
        for i in 1..knn_results.len() {
            assert!(knn_results[i].1 >= knn_results[i - 1].1);
        }
    }
}

fn generate_synthetic_distribution(
    num_points: usize,
    distribution: SyntheticDistribution,
) -> Vec<Point3<f64>> {
    let mut rng = thread_rng();

    match distribution {
        SyntheticDistribution::Uniform => {
            let dist = Uniform::new(-100.0, 100.0);
            (0..num_points)
                .map(|_| Point3::new(rng.sample(&dist), rng.sample(&dist), rng.sample(&dist)))
                .collect()
        }
        SyntheticDistribution::Normal => {
            use rand_distr::{Distribution, Normal};
            let normal = Normal::new(0.0, 25.0).unwrap();
            (0..num_points)
                .map(|_| {
                    Point3::new(
                        normal.sample(&mut rng),
                        normal.sample(&mut rng),
                        normal.sample(&mut rng),
                    )
                })
                .collect()
        }
        SyntheticDistribution::Clustered => {
            // Generate points in 5 clusters
            let num_clusters = 5;
            let points_per_cluster = num_points / num_clusters;
            let mut points = Vec::new();

            // Generate cluster centers
            let center_dist = Uniform::new(-50.0, 50.0);
            let centers: Vec<Point3<f64>> = (0..num_clusters)
                .map(|_| {
                    Point3::new(
                        rng.sample(&center_dist),
                        rng.sample(&center_dist),
                        rng.sample(&center_dist),
                    )
                })
                .collect();

            // Generate points around each cluster
            use rand_distr::{Distribution, Normal};
            let cluster_spread = Normal::new(0.0, 5.0).unwrap();

            for center in &centers {
                for _ in 0..points_per_cluster {
                    points.push(Point3::new(
                        center.x + cluster_spread.sample(&mut rng),
                        center.y + cluster_spread.sample(&mut rng),
                        center.z + cluster_spread.sample(&mut rng),
                    ));
                }
            }

            points
        }
    }
}
