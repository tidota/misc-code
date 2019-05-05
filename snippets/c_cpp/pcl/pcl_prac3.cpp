// plc_prac3.cpp
//
// 5/4/2019
//
// Practice of filtering and registration

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

//#include <boost/make_shared.hpp>
//#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
//#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

int main()
{
  pcl::console::print_highlight ("Creating point clouds...");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud1->width  = 3000;
  cloud1->height = 1;
  cloud1->points.resize (cloud1->width * cloud1->height);

  for (size_t i = 0; i < cloud1->points.size (); ++i)
  {
    cloud1->points[i].x = 1024.0 * rand () / (RAND_MAX + 1.0f);
    cloud1->points[i].y = 1024.0 * rand () / (RAND_MAX + 1.0f);
    cloud1->points[i].z = 1024.0 * rand () / (RAND_MAX + 1.0f);
  }
  std::cout << "First few points of cloud1" << std::endl;
  for (size_t i = 0; i < cloud1->points.size() && i < 5; ++i)
  {
    std::cout << cloud1->points[i].x << ", "
              << cloud1->points[i].y << ", "
              << cloud1->points[i].z << std::endl;
  }

  *cloud2 = *cloud1;

  cloud_temp->width = 500;
  cloud_temp->height = 1;
  cloud_temp->points.resize (cloud_temp->width * cloud_temp->height);

  for (size_t i = 0; i < cloud_temp->points.size (); ++i)
  {
    cloud_temp->points[i].x = 1024.0 + 500.0 * rand () / (RAND_MAX + 1.0f);
    cloud_temp->points[i].y = 300 + 500.0 * rand () / (RAND_MAX + 1.0f);
    cloud_temp->points[i].z = 300 + 500.0 * rand () / (RAND_MAX + 1.0f);
  }

  *cloud2 += *cloud_temp;


  std::cout << "First few points of cloud2" << std::endl;
  for (size_t i = 0; i < cloud2->points.size() && i < 5; ++i)
  {
    std::cout << cloud2->points[i].x << ", "
              << cloud2->points[i].y << ", "
              << cloud2->points[i].z << std::endl;
  }

  pcl::console::print_highlight("Applying passthrough...\n");

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud1);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 500.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud1);

  pass.setInputCloud (cloud2);
  pass.filter (*cloud2);

  pcl::console::print_highlight("Downsampling...\n");

  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud (cloud1);
  vox.setLeafSize (0.05, 0.05, 0.05);
  vox.filter (*cloud1);
  vox.setInputCloud (cloud2);
  vox.setLeafSize (0.05, 0.05, 0.05);
  vox.filter (*cloud2);

  pcl::console::print_highlight("Transformation cloud2...\n");

  // rotation along z axis and translation along x axis
  Eigen::Matrix4f Trans = Eigen::Matrix4f::Identity();
  float theta = 3.1459 / 4.0;
  Trans (0,0) = cos (theta);
  Trans (0,1) = -sin (theta);
  Trans (1,0) = sin (theta);
  Trans (1,1) = cos (theta);
  Trans (0,3) = 500.0;
  std::cout << "Transform: " << std::endl << Trans << std::endl;
  pcl::transformPointCloud(*cloud2, *cloud2, Trans);


  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_h (cloud1, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud2_h (cloud2, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> result_h (result, 0, 0, 255);

  Eigen::Matrix4f Trans_result = Eigen::Matrix4f::Identity();
  *cloud_temp = *cloud1;
  for (int i = 0; i < 30; ++i)
  {
    viewer->removePointCloud("cloud1");
    viewer->removePointCloud("cloud2");
    viewer->removePointCloud("result");

    pcl::console::print_highlight("Iterative Closest Point...\n");

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_temp);
    icp.setInputTarget(cloud2);
    icp.align(*result);

    std::cout << "has converged: " << icp.hasConverged() << std::endl;
    std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;
    Trans_result *= icp.getFinalTransformation();
    std::cout << "Transformation: " << std::endl << Trans_result << std::endl;

    pcl::console::print_highlight("Visualizing data...\n");

    viewer->addPointCloud (cloud1, cloud1_h, "cloud1");
    viewer->addPointCloud (cloud2, cloud2_h, "cloud2");
    viewer->addPointCloud (result, result_h, "result");

    std::cout << "Press q to proceed" << std::endl;
    viewer->spin();

    *cloud_temp = *result;
  }

  return (0);
}