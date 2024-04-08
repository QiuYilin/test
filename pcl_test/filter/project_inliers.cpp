#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main() {
  // 创建一个点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // 填充点云数据（假设您已经有了点云数据）

  pcl::PCDReader reader;
  reader.read("../../source/table_scene_lms400.pcd",
              *cloud);  // Remember to download the file first!
                        // 创建体素滤波器对象
  std::cout << "size before filter: " << cloud->size() << std::endl;
  // 创建一个平面模型
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  coefficients->values.resize(4);
  coefficients->values[0] = 0.0;  // a
  coefficients->values[1] = 0.0;  // b
  coefficients->values[2] = 1.0;  // c (平面的法线方向)
  coefficients->values[3] = 0.0;  // d

  // 创建 ProjectInliers 对象
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);  // 使用平面模型
  proj.setInputCloud(cloud);
  proj.setModelCoefficients(coefficients);

  // 进行投影
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(
      new pcl::PointCloud<pcl::PointXYZ>);
  proj.filter(*cloud_projected);

  // 现在 cloud_projected 中包含了投影到平面上的内点
  std::cout << "size after filter: " << cloud_projected->size() << std::endl;

  // 可视化cloud_filtered和cloud_filtered1
  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(
      cloud, 255, 0, 0);  // Red color for cloud_filtered
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(
      cloud_projected, 0, 255, 0);  // Green color for cloud_filtered1
  int vp_1, vp_2;
  viewer.createViewPort(0.0, 0, 0.5, 1.0, vp_1);
  viewer.createViewPort(0.5, 0, 1.0, 1.0, vp_2);
  viewer.addPointCloud<pcl::PointXYZ>(cloud, color1, "cloud", vp_1);
  viewer.addPointCloud<pcl::PointXYZ>(cloud_projected, color2,
                                      "cloud_projected", vp_2);
  viewer.spin();
  return 0;
}
