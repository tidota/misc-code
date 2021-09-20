// pcl_prac2.cpp
//
// 5/4/2019
//
// Practice of filtering including downsampling and outlier removal
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html#voxelgrid
// https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html#statistical-outlier-removal
int main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  std::cout << " ================================================" << std::endl;
  std::cout << "                 Voxel Grid                      " << std::endl;
  std::cout << " ================================================" << std::endl;

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!
  // wget https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);

  std::cout << " ================================================" << std::endl;
  std::cout << "          Statistical Outlier Removal            " << std::endl;
  std::cout << " ================================================" << std::endl;

  pcl::fromPCLPointCloud2(*cloud_filtered, *cloud2);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud2 << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
  sor2.setInputCloud (cloud2);
  sor2.setMeanK (50);
  sor2.setStddevMulThresh (1.0);
  sor2.filter (*cloud_filtered2);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered2 << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered2, false);

  return (0);
}
