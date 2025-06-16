use nalgebra::{Point3, Vector3};
use small_gicp::{traits::SpatialSearchTree, BorrowedKdTree, KdTree, PointCloud, SmallGicpError};

#[test]
fn test_borrowed_kdtree_zero_copy() {
    // Create a simple point cloud
    let mut cloud = PointCloud::new().unwrap();
    cloud.add_point(0.0, 0.0, 0.0);
    cloud.add_point(1.0, 0.0, 0.0);
    cloud.add_point(0.0, 1.0, 0.0);
    cloud.add_point(1.0, 1.0, 0.0);

    // Create a BorrowedKdTree (zero-copy)
    let borrowed_tree = BorrowedKdTree::new(&cloud).unwrap();

    // Test nearest neighbor search
    let query = Vector3::new(0.5, 0.5, 0.0);
    let result = borrowed_tree.nearest_neighbor(&query);
    assert!(result.is_some());

    // Verify the result makes sense (could be any of the 4 points)
    let (index, distance) = result.unwrap();
    assert!(index < 4);
    assert!(distance > 0.0);
}

#[test]
fn test_borrowed_kdtree_lifetime() {
    // Create a scope to test lifetime management
    let result = {
        let mut cloud = PointCloud::new().unwrap();
        cloud.add_point(0.0, 0.0, 0.0);
        cloud.add_point(1.0, 0.0, 0.0);

        let borrowed_tree = BorrowedKdTree::new(&cloud).unwrap();

        // Use the tree within the scope
        let query = Vector3::new(0.5, 0.0, 0.0);
        borrowed_tree.nearest_neighbor(&query)
    };

    // The cloud is dropped here, so borrowed_tree cannot exist outside this scope
    assert!(result.is_some());
}

#[test]
fn test_borrowed_kdtree_parallel() {
    let mut cloud = PointCloud::new().unwrap();
    for i in 0..100 {
        let x = (i % 10) as f64;
        let y = (i / 10) as f64;
        cloud.add_point(x, y, 0.0);
    }

    // Test parallel construction
    let borrowed_tree = BorrowedKdTree::new_parallel(&cloud, 4).unwrap();

    // Test k-NN search
    let query = Vector3::new(5.0, 5.0, 0.0);
    let results = borrowed_tree.knn_search(&query, 5);
    assert_eq!(results.len(), 5);

    // Verify results are sorted by distance
    for i in 1..results.len() {
        assert!(results[i].1 >= results[i - 1].1);
    }
}

#[test]
fn test_borrowed_kdtree_radius_search() {
    let mut cloud = PointCloud::new().unwrap();
    cloud.add_point(0.0, 0.0, 0.0);
    cloud.add_point(1.0, 0.0, 0.0);
    cloud.add_point(2.0, 0.0, 0.0);
    cloud.add_point(3.0, 0.0, 0.0);

    let borrowed_tree = BorrowedKdTree::new(&cloud).unwrap();

    // Search within radius 1.5 from origin
    let query = Vector3::new(0.0, 0.0, 0.0);
    let results = borrowed_tree.radius_search(&query, 1.5);

    // Should find points at distance 0 and 1
    assert_eq!(results.len(), 2);
    assert!(results.iter().any(|(_, d)| *d == 0.0));
    assert!(results.iter().any(|(_, d)| *d == 1.0));
}

#[test]
fn test_borrowed_kdtree_empty_cloud() {
    let cloud = PointCloud::new().unwrap();

    // Should fail to create tree from empty cloud
    let result = BorrowedKdTree::new(&cloud);
    assert!(matches!(result, Err(SmallGicpError::EmptyPointCloud)));
}

#[test]
fn test_borrowed_vs_owned_kdtree() {
    // Create identical point clouds
    let mut cloud = PointCloud::new().unwrap();
    for i in 0..50 {
        let angle = 2.0 * std::f64::consts::PI * i as f64 / 50.0;
        cloud.add_point(angle.cos(), angle.sin(), 0.0);
    }

    // Create both tree types
    let owned_tree = KdTree::new(&cloud).unwrap();
    let borrowed_tree = BorrowedKdTree::new(&cloud).unwrap();

    // Test that they produce identical results
    let query = Vector3::new(0.7, 0.7, 0.0);
    let query_point = Point3::new(0.7, 0.7, 0.0);

    let owned_result = owned_tree.nearest_neighbor(&query_point);
    let borrowed_result = borrowed_tree.nearest_neighbor(&query);

    assert_eq!(owned_result, borrowed_result);

    // Test k-NN
    let owned_knn = owned_tree.knn_search(&query_point, 3);
    let borrowed_knn = borrowed_tree.knn_search(&query, 3);

    assert_eq!(owned_knn, borrowed_knn);
}

#[test]
fn test_borrowed_kdtree_trait_usage() {
    let mut cloud = PointCloud::new().unwrap();
    cloud.add_point(0.0, 0.0, 0.0);
    cloud.add_point(1.0, 1.0, 1.0);

    let borrowed_tree = BorrowedKdTree::new(&cloud).unwrap();

    // Test through trait interface
    assert_eq!(borrowed_tree.empty(), borrowed_tree.size() == 0);

    // Use as trait object
    let tree_ref: &dyn SpatialSearchTree = &borrowed_tree;
    let query = Vector3::new(0.5, 0.5, 0.5);
    let result = tree_ref.nearest_neighbor(&query);
    assert!(result.is_some());
}
