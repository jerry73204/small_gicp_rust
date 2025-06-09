#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <small_gicp_c.h>

int test_covariance_operations() {
    printf("Testing covariance operations...\n");
    
    small_gicp_point_cloud_t* cloud = NULL;
    small_gicp_error_t error;
    
    // Create point cloud
    error = small_gicp_point_cloud_create(&cloud);
    if (error != SMALL_GICP_SUCCESS) {
        printf("Failed to create point cloud\n");
        return 1;
    }
    
    // Resize to hold 1 point
    error = small_gicp_point_cloud_resize(cloud, 1);
    if (error != SMALL_GICP_SUCCESS) {
        printf("Failed to resize point cloud\n");
        return 1;
    }
    
    // Set a point
    error = small_gicp_point_cloud_set_point(cloud, 0, 1.0, 2.0, 3.0);
    if (error != SMALL_GICP_SUCCESS) {
        printf("Failed to set point\n");
        return 1;
    }
    
    // Test covariance matrix (4x4 matrix in row-major order)
    double cov_matrix[16] = {
        1.0, 0.1, 0.2, 0.0,
        0.1, 2.0, 0.3, 0.0,
        0.2, 0.3, 3.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
    
    // Set covariance
    error = small_gicp_point_cloud_set_covariance(cloud, 0, cov_matrix);
    if (error != SMALL_GICP_SUCCESS) {
        printf("Failed to set covariance\n");
        return 1;
    }
    
    // Get covariance back
    double retrieved_cov[16];
    error = small_gicp_point_cloud_get_covariance(cloud, 0, retrieved_cov);
    if (error != SMALL_GICP_SUCCESS) {
        printf("Failed to get covariance\n");
        return 1;
    }
    
    // Verify values match
    for (int i = 0; i < 16; i++) {
        if (fabs(cov_matrix[i] - retrieved_cov[i]) > 1e-9) {
            printf("Covariance mismatch at index %d: expected %f, got %f\n", 
                   i, cov_matrix[i], retrieved_cov[i]);
            return 1;
        }
    }
    
    // Test has_covariances function
    bool has_covariances;
    error = small_gicp_point_cloud_has_covariances(cloud, &has_covariances);
    if (error != SMALL_GICP_SUCCESS || !has_covariances) {
        printf("Point cloud should have covariances\n");
        return 1;
    }
    
    // Cleanup
    small_gicp_point_cloud_destroy(cloud);
    
    printf("Covariance operations test passed!\n");
    return 0;
}

int test_parallel_kdtree() {
    printf("Testing parallel KdTree creation...\n");
    
    small_gicp_point_cloud_t* cloud = NULL;
    small_gicp_kdtree_t* kdtree = NULL;
    small_gicp_error_t error;
    
    // Create point cloud with some test points
    error = small_gicp_point_cloud_create(&cloud);
    if (error != SMALL_GICP_SUCCESS) {
        printf("Failed to create point cloud\n");
        return 1;
    }
    
    // Add some test points
    error = small_gicp_point_cloud_resize(cloud, 100);
    if (error != SMALL_GICP_SUCCESS) {
        printf("Failed to resize point cloud\n");
        return 1;
    }
    
    for (int i = 0; i < 100; i++) {
        error = small_gicp_point_cloud_set_point(cloud, i, 
                                                 i * 0.1, i * 0.2, i * 0.3);
        if (error != SMALL_GICP_SUCCESS) {
            printf("Failed to set point %d\n", i);
            return 1;
        }
    }
    
    // Test default builder
    printf("Testing default builder...\n");
    error = small_gicp_kdtree_create_with_builder(cloud, 
                                                  SMALL_GICP_KDTREE_BUILDER_DEFAULT, 
                                                  1, &kdtree);
    if (error != SMALL_GICP_SUCCESS) {
        printf("Failed to create KdTree with default builder\n");
        return 1;
    }
    small_gicp_kdtree_destroy(kdtree);
    kdtree = NULL;
    
    // Test OpenMP builder
    printf("Testing OpenMP builder...\n");
    error = small_gicp_kdtree_create_with_builder(cloud, 
                                                  SMALL_GICP_KDTREE_BUILDER_OPENMP, 
                                                  4, &kdtree);
    if (error != SMALL_GICP_SUCCESS) {
        printf("Failed to create KdTree with OpenMP builder\n");
        return 1;
    }
    small_gicp_kdtree_destroy(kdtree);
    kdtree = NULL;
    
    // Test TBB builder
    printf("Testing TBB builder...\n");
    error = small_gicp_kdtree_create_with_builder(cloud, 
                                                  SMALL_GICP_KDTREE_BUILDER_TBB, 
                                                  1, &kdtree);
    if (error != SMALL_GICP_SUCCESS) {
        printf("Failed to create KdTree with TBB builder\n");
        return 1;
    }
    small_gicp_kdtree_destroy(kdtree);
    kdtree = NULL;
    
    // Test configuration-based creation
    printf("Testing configuration-based creation...\n");
    small_gicp_kdtree_config_t* config = NULL;
    error = small_gicp_kdtree_config_create(SMALL_GICP_KDTREE_BUILDER_OPENMP, 
                                           4, 10, &config);
    if (error != SMALL_GICP_SUCCESS) {
        printf("Failed to create KdTree config\n");
        return 1;
    }
    
    error = small_gicp_kdtree_create_with_config(cloud, config, &kdtree);
    if (error != SMALL_GICP_SUCCESS) {
        printf("Failed to create KdTree with config\n");
        return 1;
    }
    
    // Test a simple search
    size_t index;
    double sq_dist;
    error = small_gicp_kdtree_nearest_neighbor_search(kdtree, 1.0, 2.0, 3.0, 
                                                     &index, &sq_dist);
    if (error != SMALL_GICP_SUCCESS) {
        printf("Failed to perform nearest neighbor search\n");
        return 1;
    }
    
    printf("Nearest neighbor search found index %zu with distance^2 %f\n", 
           index, sq_dist);
    
    // Cleanup
    small_gicp_kdtree_destroy(kdtree);
    small_gicp_kdtree_config_destroy(config);
    small_gicp_point_cloud_destroy(cloud);
    
    printf("Parallel KdTree test passed!\n");
    return 0;
}

int main() {
    printf("Running tests for new covariance and parallel KdTree functionality\n\n");
    
    if (test_covariance_operations() != 0) {
        printf("Covariance test failed!\n");
        return 1;
    }
    
    if (test_parallel_kdtree() != 0) {
        printf("Parallel KdTree test failed!\n");
        return 1;
    }
    
    printf("\nAll tests passed successfully!\n");
    return 0;
}