#include <pcl/filters/median_filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_label.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <chrono>
#include <iostream>

int main() {
  // VoxelGrid<pcl::PCLPointCloud2>的情况
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read("../../source/table_scene_lms400.pcd",
              *cloud);  // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
            << " data points (" << pcl::getFieldsList(*cloud) << ")."
            << std::endl;

  // Create the filtering object
  //   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  //   sor.setInputCloud(cloud);
  //   sor.setLeafSize(0.01f, 0.01f, 0.01f);
  //   sor.filter(*cloud_filtered);

  pcl::MedianFilter<pcl::PointXYZ> median_filter;

  std::cerr << "PointCloud after filtering: "
            << cloud_filtered->width * cloud_filtered->height
            << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")."
            << std::endl;

  pcl::PCDWriter writer;
  writer.write("table_scene_lms400_downsampled.pcd", *cloud_filtered,
               Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

  return (0);
}