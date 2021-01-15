// prac.cpp
// 03/09/2020
// most of the parts are from octomap->src/simple_example.cpp

//#include <stdio.h>
#include <iostream>
#include <iomanip>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

void print_query_info(octomap::point3d query, octomap::OcTreeNode* node)
{
  if (node != NULL)
    std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
  else
    std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;
}

int main(int argc, char** argv )
{

    octomap::OcTree *map;

    map = new octomap::OcTree(0.1);
    map->setProbHit(0.7);
    map->setProbMiss(0.4);
    map->setClampingThresMin(0.12);
    map->setClampingThresMax(0.97);

    for (double x = -1.0; x < 1.0; x += 0.05f)
    {
        for (double y = -1.0; y < 1.0; y += 0.05f)
        {
            for (double z = -1.0; z < 1.0; z += 0.05f)
            {
                octomap::point3d endpoint ((float) x, (float) y, (float) z);
                map->updateNode(endpoint, true); // integrate 'occupied' measurement
            }
        }
    }
    for (double x = -1.58; x <= -0.4; x += 0.02f)
    {
        for (double y = -1.58; y <= -0.4; y += 0.02f)
        {
            for (double z = -1.58; z <= -0.4; z += 0.02f)
            {
                octomap::point3d endpoint ((float) x, (float) y, (float) z);
                map->updateNode(endpoint, false);  // integrate 'free' measurement
            }
        }
    }

    std::cout << std::endl;
    std::cout << "performing some queries:" << std::endl;

    octomap::point3d query (0., 0., 0.);
    octomap::OcTreeNode* result = map->search (query);
    print_query_info(query, result);

    query = octomap::point3d(-1.,-1.,-1.);
    result = map->search (query);
    print_query_info(query, result);

    query = octomap::point3d(1.,1.,1.);
    result = map->search (query);
    print_query_info(query, result);

    std::cout << "=== scan ===" << std::endl;
    for (double x = -1.7; x <= 1.7; x += 0.1)
    {
      for (double y = -1.7; y <= 1.7; y += 0.1)
      {
        query = octomap::point3d(x, y, -0.5);
        result = map->search (query);
        //print_query_info(query, result);
        if (result)
        {
          std::cout << std::setw(5) << std::setprecision(2) << std::fixed
                    << result->getOccupancy();
        }
        else
        {
          std::cout << " XXXX";
        }
      }
      std::cout << std::endl;
    }

    std::cout << std::endl;
    map->writeBinary("simple_tree.bt");
    std::cout << "wrote example file simple_tree.bt" << std::endl << std::endl;
    std::cout << "now you can use octovis to visualize: octovis simple_tree.bt" << std::endl;
    std::cout << "Hint: hit 'F'-key in viewer to see the freespace" << std::endl << std::endl;

    delete map;

    return 0;
}
