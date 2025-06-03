#include <small_gicp_c/io/io.h>
#include "../common.h"

#include <fstream>

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

small_gicp_error_t small_gicp_load_ply(const char *filename,
                                       small_gicp_point_cloud_t **cloud) {
  CHECK_NULL(filename);
  CHECK_NULL(cloud);

  TRY_CATCH_BEGIN
  auto points = small_gicp::read_ply(filename);
  if (points.empty()) {
    return SMALL_GICP_ERROR_FILE_NOT_FOUND;
  }

  *cloud = new small_gicp_point_cloud;
  (*cloud)->cloud = std::make_shared<small_gicp::PointCloud>();

  // Convert from vector of Vector4f to PointCloud
  (*cloud)->cloud->resize(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    (*cloud)->cloud->point(i) = points[i].cast<double>();
  }
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_save_ply(const char *filename,
                                       const small_gicp_point_cloud_t *cloud) {
  CHECK_NULL(filename);
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);

  TRY_CATCH_BEGIN
  // Simple PLY writer
  std::ofstream ofs(filename);
  if (!ofs) {
    return SMALL_GICP_ERROR_IO_ERROR;
  }

  const auto &pc = *cloud->cloud;
  ofs << "ply\n";
  ofs << "format ascii 1.0\n";
  ofs << "element vertex " << pc.size() << "\n";
  ofs << "property float x\n";
  ofs << "property float y\n";
  ofs << "property float z\n";
  if (!pc.normals.empty()) {
    ofs << "property float nx\n";
    ofs << "property float ny\n";
    ofs << "property float nz\n";
  }
  ofs << "end_header\n";

  for (size_t i = 0; i < pc.size(); ++i) {
    const auto &p = pc.point(i);
    ofs << p.x() << " " << p.y() << " " << p.z();
    if (!pc.normals.empty()) {
      const auto &n = pc.normal(i);
      ofs << " " << n.x() << " " << n.y() << " " << n.z();
    }
    ofs << "\n";
  }
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_load_points_from_array(const float *points, size_t num_points,
                                  small_gicp_point_cloud_t **cloud) {
  CHECK_NULL(points);
  CHECK_NULL(cloud);

  TRY_CATCH_BEGIN
  *cloud = new small_gicp_point_cloud;
  (*cloud)->cloud = std::make_shared<small_gicp::PointCloud>();
  (*cloud)->cloud->resize(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    (*cloud)->cloud->point(i) = Eigen::Vector4d(
        points[i * 3 + 0], points[i * 3 + 1], points[i * 3 + 2], 1.0);
  }
  TRY_CATCH_END
}