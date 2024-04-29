#define PCL_NO_PRECOMPILE

#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

int main() {
  // 读取点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          "D:/program_on_git/own/test/pcl_test/source/croped.pcd", *cloud) ==
      -1) {
    PCL_ERROR("Couldn't read file.\n");
    return -1;
  }

  // 法线估计
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setNumberOfThreads(8);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(100);  // 设置最近邻点的数量
  ne.setViewPoint(100, 100, 1000);
  ne.compute(*normals);

  // 可视化点云和法线
  pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
      cloud, normals, cloud->size() / 100, 20, "normals");
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  viewer.spin();

  return 0;
}