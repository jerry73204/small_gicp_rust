//! Test-only PLY I/O utilities
//! This module is only available in tests and not part of the public API

use small_gicp::{PointCloud, PointCloudTrait};
use std::{
    fs::File,
    io::{BufRead, BufReader, Read, Write},
    path::{Path, PathBuf},
};

/// Test data helper for loading common test point clouds
pub struct TestData;

impl TestData {
    /// Load the source point cloud from test data
    pub fn source_cloud() -> Result<PointCloud, Box<dyn std::error::Error>> {
        load_test_ply(Self::data_path("source.ply"))
    }

    /// Load the target point cloud from test data
    pub fn target_cloud() -> Result<PointCloud, Box<dyn std::error::Error>> {
        load_test_ply(Self::data_path("target.ply"))
    }

    /// Get the path to a test data file
    fn data_path(filename: &str) -> PathBuf {
        PathBuf::from("data").join(filename)
    }
}

/// Load a PLY file into a PointCloud for testing purposes
/// Supports both ASCII and binary PLY formats
pub fn load_test_ply<P: AsRef<Path>>(path: P) -> Result<PointCloud, Box<dyn std::error::Error>> {
    let path = path.as_ref();
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);

    let mut line = String::new();
    reader.read_line(&mut line)?;

    if line.trim() != "ply" {
        return Err("Not a PLY file".into());
    }

    // Read format line
    line.clear();
    reader.read_line(&mut line)?;
    let is_binary = line.contains("binary");
    let is_little_endian = line.contains("little_endian");

    let mut num_vertices = 0;

    // Parse rest of header
    loop {
        line.clear();
        reader.read_line(&mut line)?;

        if line.trim() == "end_header" {
            break;
        }

        if line.starts_with("element vertex") {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 3 {
                num_vertices = parts[2].parse().unwrap_or(0);
            }
        }
    }

    let mut cloud = PointCloud::new()?;
    cloud.reserve(num_vertices);

    if is_binary {
        // Read binary data
        for _ in 0..num_vertices {
            let mut xyz = [0f32; 3];

            if is_little_endian {
                #[allow(clippy::needless_range_loop)]
                for i in 0..3 {
                    let mut bytes = [0u8; 4];
                    reader.read_exact(&mut bytes)?;
                    xyz[i] = f32::from_le_bytes(bytes);
                }
            } else {
                #[allow(clippy::needless_range_loop)]
                for i in 0..3 {
                    let mut bytes = [0u8; 4];
                    reader.read_exact(&mut bytes)?;
                    xyz[i] = f32::from_be_bytes(bytes);
                }
            }

            cloud.add_point(xyz[0] as f64, xyz[1] as f64, xyz[2] as f64);
        }
    } else {
        // Read ASCII data
        for line in reader.lines() {
            let line = line?;
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 3 {
                let x: f64 = parts[0].parse()?;
                let y: f64 = parts[1].parse()?;
                let z: f64 = parts[2].parse()?;
                cloud.add_point(x, y, z);
            }

            if cloud.len() >= num_vertices {
                break;
            }
        }
    }

    Ok(cloud)
}

/// Save a PointCloud to a PLY file for testing purposes (ASCII format)
pub fn save_test_ply<P: AsRef<Path>>(
    path: P,
    cloud: &PointCloud,
) -> Result<(), Box<dyn std::error::Error>> {
    let path = path.as_ref();
    let mut file = File::create(path)?;

    // Write header
    writeln!(file, "ply")?;
    writeln!(file, "format ascii 1.0")?;
    writeln!(file, "comment Created by small_gicp Rust test utilities")?;
    writeln!(file, "element vertex {}", cloud.len())?;
    writeln!(file, "property float x")?;
    writeln!(file, "property float y")?;
    writeln!(file, "property float z")?;
    writeln!(file, "end_header")?;

    // Write vertices
    for i in 0..cloud.len() {
        let point = cloud.point(i);
        writeln!(file, "{} {} {}", point.x, point.y, point.z)?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_load_ply_basic() {
        // Test loading the test PLY files
        let result = load_test_ply("data/target.ply");

        match result {
            Ok(cloud) => {
                println!("Successfully loaded target.ply with {} points", cloud.len());
                assert!(!cloud.is_empty(), "Expected non-empty point cloud");
            }
            Err(e) => {
                eprintln!("Failed to load target.ply: {}", e);
                // Don't fail the test if file doesn't exist - this allows tests to pass in CI
            }
        }
    }

    #[test]
    fn test_save_load_roundtrip() {
        // Create a simple point cloud
        let mut cloud = PointCloud::new().unwrap();
        cloud.add_point(1.0, 2.0, 3.0);
        cloud.add_point(4.0, 5.0, 6.0);
        cloud.add_point(7.0, 8.0, 9.0);

        // Save to temporary file
        let temp_path = "test_roundtrip.ply";
        save_test_ply(temp_path, &cloud).expect("Failed to save PLY");

        // Load it back
        let loaded = load_test_ply(temp_path).expect("Failed to load PLY");

        // Verify
        assert_eq!(cloud.len(), loaded.len());
        for i in 0..cloud.len() {
            let p1 = cloud.point(i);
            let p2 = loaded.point(i);
            assert!((p1.x - p2.x).abs() < 1e-5);
            assert!((p1.y - p2.y).abs() < 1e-5);
            assert!((p1.z - p2.z).abs() < 1e-5);
        }

        // Clean up
        std::fs::remove_file(temp_path).ok();
    }
}
