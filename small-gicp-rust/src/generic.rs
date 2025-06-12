//! Generic algorithms that work with any implementation of PointCloudTrait.
//!
//! This module provides high-level generic algorithms that can work with
//! both C wrapper types and custom implementations, providing the benefits
//! of generic programming while maintaining performance through the C wrapper backend.

pub mod kdtree;
pub mod preprocessing;

pub use kdtree::{GenericKdTree, KdTreeStrategy};
