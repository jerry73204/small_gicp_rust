//! Integration tests for test I/O utilities

mod common;

use common::{
    test_io::{load_test_ply, save_test_ply},
    TestData,
};
use small_gicp::{PointCloud, PointCloudTrait};

#[test]
fn test_load_test_data() {
    // Test loading the actual test PLY files
    if let Ok(target) = load_test_ply("data/target.ply") {
        println!(
            "Successfully loaded target.ply with {} points",
            target.len()
        );
        assert!(!target.is_empty(), "Expected non-empty point cloud");
    } else {
        println!("target.ply not found - skipping test");
    }

    if let Ok(source) = load_test_ply("data/source.ply") {
        println!(
            "Successfully loaded source.ply with {} points",
            source.len()
        );
        assert!(!source.is_empty(), "Expected non-empty point cloud");
    } else {
        println!("source.ply not found - skipping test");
    }
}

#[test]
fn test_roundtrip() {
    // Create a test point cloud
    let mut cloud = PointCloud::new().unwrap();
    for i in 0..10 {
        let x = i as f64 * 0.1;
        cloud.add_point(x, x * 2.0, x * 3.0);
    }

    // Save and load back
    let temp_path = "test_io_roundtrip.ply";
    save_test_ply(temp_path, &cloud).expect("Failed to save PLY");

    let loaded = load_test_ply(temp_path).expect("Failed to load PLY");

    // Verify
    assert_eq!(cloud.len(), loaded.len());
    for i in 0..cloud.len() {
        let p1 = cloud.point(i);
        let p2 = loaded.point(i);
        assert!((p1.x - p2.x).abs() < 1e-6);
        assert!((p1.y - p2.y).abs() < 1e-6);
        assert!((p1.z - p2.z).abs() < 1e-6);
    }

    // Clean up
    std::fs::remove_file(temp_path).ok();
}

#[test]
fn test_convenience_loaders() {
    // Test TestData convenience methods
    match TestData::source_cloud() {
        Ok(source) => {
            println!("TestData::source_cloud() loaded {} points", source.len());
            assert_eq!(source.len(), 69792, "Source cloud should have 69792 points");
        }
        Err(e) => {
            println!("Failed to load source cloud: {} - skipping test", e);
        }
    }

    match TestData::target_cloud() {
        Ok(target) => {
            println!("TestData::target_cloud() loaded {} points", target.len());
            assert_eq!(target.len(), 69088, "Target cloud should have 69088 points");
        }
        Err(e) => {
            println!("Failed to load target cloud: {} - skipping test", e);
        }
    }
}
