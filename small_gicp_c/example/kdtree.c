#include <math.h>
#include <small_gicp_c.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Generate random point cloud
small_gicp_point_cloud_t *generate_random_cloud(size_t num_points) {
  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_point_cloud_create(&cloud);
  small_gicp_point_cloud_resize(cloud, num_points);

  srand(42); // Fixed seed for reproducibility

  for (size_t i = 0; i < num_points; i++) {
    double x = (double)rand() / RAND_MAX * 100.0 - 50.0;
    double y = (double)rand() / RAND_MAX * 100.0 - 50.0;
    double z = (double)rand() / RAND_MAX * 20.0 - 10.0;
    small_gicp_point_cloud_set_point(cloud, i, x, y, z);
  }

  return cloud;
}

int main(int argc, char **argv) {
  small_gicp_error_t error;

  // Generate or load point cloud
  small_gicp_point_cloud_t *cloud = NULL;

  if (argc > 1) {
    printf("Loading point cloud from %s...\n", argv[1]);
    error = small_gicp_load_ply(argv[1], &cloud);
    if (error != SMALL_GICP_SUCCESS) {
      fprintf(stderr, "Failed to load point cloud: %s\n",
              small_gicp_error_string(error));
      return 1;
    }
  } else {
    printf("Generating random point cloud...\n");
    cloud = generate_random_cloud(10000);
  }

  size_t cloud_size;
  small_gicp_point_cloud_size(cloud, &cloud_size);
  printf("Point cloud size: %zu\n", cloud_size);

  // Build KdTree
  printf("\nBuilding KdTree...\n");
  small_gicp_kdtree_t *kdtree = NULL;

  clock_t start = clock();
  error = small_gicp_kdtree_create(cloud, 4, &kdtree);
  clock_t end = clock();

  if (error != SMALL_GICP_SUCCESS) {
    fprintf(stderr, "Failed to create KdTree: %s\n",
            small_gicp_error_string(error));
    small_gicp_point_cloud_destroy(cloud);
    return 1;
  }

  double build_time = (double)(end - start) / CLOCKS_PER_SEC;
  printf("KdTree built in %.3f seconds\n", build_time);

  // Test nearest neighbor search
  printf("\nTesting nearest neighbor search...\n");
  double query_points[][3] = {{0.0, 0.0, 0.0},
                              {10.0, 10.0, 5.0},
                              {-20.0, 30.0, -5.0},
                              {45.0, -45.0, 8.0}};

  for (int i = 0; i < 4; i++) {
    size_t nearest_idx;
    double sq_dist;

    error = small_gicp_kdtree_nearest_neighbor_search(
        kdtree, query_points[i][0], query_points[i][1], query_points[i][2],
        &nearest_idx, &sq_dist);

    if (error == SMALL_GICP_SUCCESS) {
      double nx, ny, nz;
      small_gicp_point_cloud_get_point(cloud, nearest_idx, &nx, &ny, &nz);

      printf("Query: (%.1f, %.1f, %.1f) -> Nearest: idx=%zu (%.2f, %.2f, "
             "%.2f), dist=%.3f\n",
             query_points[i][0], query_points[i][1], query_points[i][2],
             nearest_idx, nx, ny, nz, sqrt(sq_dist));
    } else {
      fprintf(stderr, "Nearest neighbor search failed: %s\n",
              small_gicp_error_string(error));
    }
  }

  // Test k-nearest neighbors search
  printf("\nTesting k-nearest neighbors search (k=5)...\n");
  const int k = 5;
  size_t *indices = malloc(k * sizeof(size_t));
  double *sq_dists = malloc(k * sizeof(double));

  for (int i = 0; i < 2; i++) {
    error = small_gicp_kdtree_knn_search(kdtree, query_points[i][0],
                                         query_points[i][1], query_points[i][2],
                                         k, indices, sq_dists);

    if (error == SMALL_GICP_SUCCESS) {
      printf("\nQuery: (%.1f, %.1f, %.1f)\n", query_points[i][0],
             query_points[i][1], query_points[i][2]);

      for (int j = 0; j < k; j++) {
        double nx, ny, nz;
        small_gicp_point_cloud_get_point(cloud, indices[j], &nx, &ny, &nz);
        printf("  %d: idx=%zu (%.2f, %.2f, %.2f), dist=%.3f\n", j + 1,
               indices[j], nx, ny, nz, sqrt(sq_dists[j]));
      }
    } else {
      fprintf(stderr, "KNN search failed: %s\n",
              small_gicp_error_string(error));
    }
  }

  // Performance test
  printf("\nPerformance test: 1000 random queries...\n");
  start = clock();

  for (int i = 0; i < 1000; i++) {
    double qx = (double)rand() / RAND_MAX * 100.0 - 50.0;
    double qy = (double)rand() / RAND_MAX * 100.0 - 50.0;
    double qz = (double)rand() / RAND_MAX * 20.0 - 10.0;

    size_t idx;
    double dist;
    small_gicp_kdtree_nearest_neighbor_search(kdtree, qx, qy, qz, &idx, &dist);
  }

  end = clock();
  double query_time = (double)(end - start) / CLOCKS_PER_SEC;
  printf("1000 queries completed in %.3f seconds (%.3f ms per query)\n",
         query_time, query_time * 1000.0 / 1000.0);

  // Clean up
  free(indices);
  free(sq_dists);
  small_gicp_point_cloud_destroy(cloud);
  small_gicp_kdtree_destroy(kdtree);

  return 0;
}
