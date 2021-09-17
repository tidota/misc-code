// plc_prac4.cpp
//
// 5/4/2019
//
// Practice of segmentation

#include <iostream>
#include <iomanip>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// http://pointclouds.org/documentation/tutorials/planar_segmentation.html#planar-segmentation
int main()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 10;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1.0;
  }

  // outliers
  cloud->points[3].z = -2.0;
  cloud->points[5].z = 2.0;
  cloud->points[9].z = 4.0;

  pcl::console::print_highlight("cloud\n");
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    std::cout << std::setw(3) << i << ": ";
    std::cout << cloud->points[i].x << ", "
              << cloud->points[i].y << ", "
              << cloud->points[i].z << std::endl;
  }

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.filter(*cloud_filtered);

  pcl::console::print_highlight("cloud_filtered\n");
  for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
  {
    std::cout << std::setw(3) << i << ": ";
    std::cout << cloud_filtered->points[i].x << ", "
              << cloud_filtered->points[i].y << ", "
              << cloud_filtered->points[i].z << std::endl;
  }

  return (0);
}
