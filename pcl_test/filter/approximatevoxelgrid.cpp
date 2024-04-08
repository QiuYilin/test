#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <iostream>

int main() {
  // 定义PointCloud对象并填充数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1(
      new pcl::PointCloud<pcl::PointXYZ>);

  // ... 填充cloud数据 ...
  // Fill in the cloud data pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  pcl::PCDReader reader;
  reader.read("../../source/table_scene_lms400.pcd",
              *cloud);  // Remember to download the file first!
                        // 创建体素滤波器对象
  std::cout << "size before filter: " << cloud->size() << std::endl;
  std::cout << "start" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);  // 设置体素大小
  sor.filter(*cloud_filtered);

  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();

  std::cout << "Execution time: " << duration << " milliseconds" << std::endl;
  std::cout << "size after filter: " << cloud_filtered->size() << std::endl;

  std::cout << "start" << std::endl;
  auto start1 = std::chrono::high_resolution_clock::now();
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor1;
  sor1.setInputCloud(cloud);
  sor1.setLeafSize(0.01f, 0.01f, 0.01f);  // 设置体素大小
  sor1.filter(*cloud_filtered1);

  auto end1 = std::chrono::high_resolution_clock::now();
  auto duration1 =
      std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1)
          .count();

  std::cout << "Execution time: " << duration1 << " milliseconds" << std::endl;

  std::cout << "size after filter: " << cloud_filtered1->size() << std::endl;

  // 可视化cloud_filtered和cloud_filtered1
  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(
      cloud_filtered, 255, 0, 0);  // Red color for cloud_filtered
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(
      cloud_filtered1, 0, 255, 0);  // Green color for cloud_filtered1
  int vp_1, vp_2;
  viewer.createViewPort(0.0, 0, 0.5, 1.0, vp_1);
  viewer.createViewPort(0.5, 0, 1.0, 1.0, vp_2);
  viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, color1, "cloud_filtered",
                                      vp_1);
  viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered1, color2,
                                      "cloud_filtered1", vp_2);
  viewer.spin();

  return (0);
}