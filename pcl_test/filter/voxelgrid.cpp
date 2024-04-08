#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_label.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <chrono>
#include <iostream>

int main() {
  // VoxelGrid<pcl::PCLPointCloud2>的情况
  //   pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
  //   pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

  //   // Fill in the cloud data
  //   pcl::PCDReader reader;
  //   // Replace the path below with the path where you saved your file
  //   reader.read("../../source/table_scene_lms400.pcd",
  //               *cloud);  // Remember to download the file first!

  //   std::cerr << "PointCloud before filtering: " << cloud->width *
  //   cloud->height
  //             << " data points (" << pcl::getFieldsList(*cloud) << ")."
  //             << std::endl;

  //   // Create the filtering object
  //   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  //   sor.setInputCloud(cloud);
  //   sor.setLeafSize(0.01f, 0.01f, 0.01f);
  //   sor.filter(*cloud_filtered);

  //   std::cerr << "PointCloud after filtering: "
  //             << cloud_filtered->width * cloud_filtered->height
  //             << " data points (" << pcl::getFieldsList(*cloud_filtered) <<
  //             ")."
  //             << std::endl;

  //   pcl::PCDWriter writer;
  //   writer.write("table_scene_lms400_downsampled.pcd", *cloud_filtered,
  //                Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),
  //                false);

  // 定义PointCloud对象并填充数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(
      new pcl::PointCloud<pcl::PointXYZ>);

  // ... 填充cloud数据 ...
  // Fill in the cloud data pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  pcl::PCDReader reader;
  reader.read("../../source/table_scene_lms400.pcd",
              *cloud);  // Remember to download the file first!
                        // 创建体素滤波器对象

  // 体素滤波的同时直通滤波
  std::cout << "start" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);  // 设置体素大小
  sor.setFilterFieldName("x");
  sor.setFilterLimits(0, INT_MAX);
  sor.filter(*cloud_filtered);

  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();

  std::cout << "Execution time: " << duration << " milliseconds" << std::endl;

  // 先直通滤波再体素滤波
  std::cout << "start" << std::endl;
  auto start1 = std::chrono::high_resolution_clock::now();
  pcl::PassThrough<pcl::PointXYZ> sor1;
  sor1.setInputCloud(cloud);
  sor1.setFilterFieldName("x");
  sor1.setFilterLimits(0, INT_MAX);
  sor1.filter(*cloud_filtered1);
  auto end1 = std::chrono::high_resolution_clock::now();
  auto duration1 =
      std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1)
          .count();
  std::cout << "Execution time: " << duration1 << " milliseconds" << std::endl;

  std::cout << "start" << std::endl;
  auto start0 = std::chrono::high_resolution_clock::now();
  pcl::VoxelGrid<pcl::PointXYZ> sor2;
  sor2.setInputCloud(cloud_filtered1);
  sor2.setLeafSize(0.01f, 0.01f, 0.01f);  // 设置体素大小
  sor2.filter(*cloud_filtered2);
  auto end0 = std::chrono::high_resolution_clock::now();
  auto duration0 =
      std::chrono::duration_cast<std::chrono::milliseconds>(end0 - start0)
          .count();
  std::cout << "Execution time: " << duration0 << " milliseconds" << std::endl;

  return (0);
}