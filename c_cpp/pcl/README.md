# Point Cloud Library

Point Cloud Library (PCL) is a library offering a set of functionalities to manipulate point clouds.

This is just some notes for myself to get familiar with it.

To insall
```
sudo apt install libpcl-dev
```

- [Tutorial](http://pointclouds.org/documentation/tutorials/)

  The starting point.

- [Getting Started / Basic Structures](http://pointclouds.org/documentation/tutorials/basic_structures.php#basic-structures)

  Overview of the data structure.

- [Using PCL in your own project](http://pointclouds.org/documentation/tutorials/using_pcl_pcl_config.php#using-pcl-pcl-config)

  How to prepare a project by CMake.

- [Writing Point Cloud data to PCD files](http://pointclouds.org/documentation/tutorials/writing_pcd.php#writing-pcd)

  Sample code to use PCL.

- [Using a matrix to transform a point cloud](http://pointclouds.org/documentation/tutorials/matrix_transform.php#matrix-transform)


# list of code

- pcl_prac.cpp

   - Passthrough
   - Projection (to a plane)

- pcl_prac2.cpp

   - Statistical Outlier Removal (to remove noisy data)

      need to download the dataset
      ```
      wget https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd
      ```

- pcl_prac3.cpp

   - Iterative Closest Point (ICP)

      Note: it is used for registration

- pcl_prac4.cpp

   - Plane Model Segmentation (to find a plane)

- pcd_write.cpp

   - Saving data into a file
