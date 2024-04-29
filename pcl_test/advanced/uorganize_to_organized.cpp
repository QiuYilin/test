#include <pcl/common/transforms.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

// 投影点云到平面
pcl::PointCloud<pcl::PointXYZINormal>::Ptr ProjectToPlane(
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, Eigen::Vector3f origin,
    Eigen::Vector3f axis_x, Eigen::Vector3f axis_y) {
  PointCloud<PointXYZINormal>::Ptr aux_cloud(new PointCloud<PointXYZINormal>);
  copyPointCloud(*cloud, *aux_cloud);

  auto normal = axis_x.cross(axis_y);
  Eigen::Hyperplane<float, 3> plane(normal, origin);

  for (auto itPoint = aux_cloud->begin(); itPoint != aux_cloud->end();
       itPoint++) {
    // project point to plane
    auto proj = plane.projection(itPoint->getVector3fMap());
    itPoint->getVector3fMap() = proj;
    // optional: save the reconstruction information as normals in the projected
    // cloud
    itPoint->getNormalVector3fMap() = itPoint->getVector3fMap() - proj;
  }
  return aux_cloud;
}

// 生成栅格 需要原点 坐标轴x 坐标轴y 现实长度 预定图像大小
pcl::PointCloud<pcl::PointXYZINormal>::Ptr GenerateGrid(Eigen::Vector3f origin,
                                                        Eigen::Vector3f axis_x,
                                                        Eigen::Vector3f axis_y,
                                                        float length,
                                                        int image_size) {
  auto step = length / image_size;

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr image_cloud(
      new pcl::PointCloud<pcl::PointXYZINormal>(image_size, image_size));
  for (auto i = 0; i < image_size; i++)
    for (auto j = 0; j < image_size; j++) {
      int x = i - int(image_size / 2);
      int y = j - int(image_size / 2);
      image_cloud->at(i, j).getVector3fMap() =
          origin + (x * step * axis_x) +
          (y * step * axis_y);  // 点中的元素就是当前网格系统中的坐标
    }

  return image_cloud;
}

// 平面点云插值到栅格
void InterpolateToGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr grid,
                       float max_resolution, int max_nn_to_consider) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  for (auto idx = 0; idx < grid->points.size(); idx++) {
    std::vector<int> indices;
    std::vector<float> distances;
    // 线性插值强度和法线信息
    if (tree->radiusSearch(grid->points[idx], max_resolution, indices,
                           distances, max_nn_to_consider) > 0) {
      // Linear Interpolation of:
      //      Intensity
      //      Normals- residual vector to inflate(recondtruct) the surface
      float intensity(0);
      Eigen::Vector3f n(0, 0, 0);
      float weight_factor =
          1.0F / accumulate(distances.begin(), distances.end(), 0.0F);
      for (auto i = 0; i < indices.size(); i++) {
        float w = weight_factor * distances[i];
        intensity += w * cloud->points[indices[i]].intensity;
        auto res = cloud->points[indices[i]].getVector3fMap() -
                   grid->points[idx].getVector3fMap();
        n += w * res;
      }
      grid->points[idx].intensity = intensity;
      grid->points[idx].getNormalVector3fMap() = n;
      grid->points[idx].curvature = 1;
    } else {
      grid->points[idx].intensity = 0;
      grid->points[idx].curvature = 0;
      grid->points[idx].getNormalVector3fMap() = Eigen::Vector3f(0, 0, 0);
    }
  }
}

int main() {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr original_cloud = ...;

  // reference frame for the projection
  // e.g. take XZ plane around 0,0,0 of length 100 and map to 128*128 image
  Eigen::Vector3f origin = Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f axis_x = Eigen::Vector3f(1, 0, 0);
  Eigen::Vector3f axis_y = Eigen::Vector3f(0, 0, 1);
  float length = 100;
  int image_size = 128;

  auto aux_cloud = ProjectToPlane(original_cloud, origin, axis_x, axis_y);
  // aux_cloud now contains the points of original_cloud, with:
  //      xyz coordinates projected to XZ plane
  //      color (intensity) of the original_cloud (remains unchanged)
  //      normals - we lose the normal information, as we use this field to save
  //      the projection information. if you wish to keep the normal data, you
  //      should define a custom PointType.
  // note: for the sake of projection, the origin is only used to define the
  // plane, so any arbitrary point on the plane can be used

  auto grid = GenerateGrid(origin, axis_x, axis_y, length, image_size)
      // organized cloud that can be trivially mapped to an image

      float max_resolution = 2 * length / image_size;
  int max_nn_to_consider = 16;
  InterpolateToGrid(aux_cloud, grid, max_resolution, max_nn_to_consider);
  // Now you have a grid (an organized cloud), which you can easily map to an
  // image. Any changes you make to the images, you can map back to the grid,
  // and use the normals to project back to your original point cloud.
}
