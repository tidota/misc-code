// prac.cpp
// 03/09/2020
// most of the parts are from octomap->src/simple_example.cpp

#include <stdio.h>

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

    for (int x=-20; x<20; x++)
    {
        for (int y=-20; y<20; y++)
        {
            for (int z=-20; z<20; z++)
            {
                octomap::point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
                map->updateNode(endpoint, true); // integrate 'occupied' measurement
            }
        }
    }
    for (int x=-30; x<30; x++)
    {
        for (int y=-30; y<30; y++)
        {
            for (int z=-30; z<30; z++)
            {
                octomap::point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
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

    std::cout << std::endl;
    map->writeBinary("simple_tree.bt");
    std::cout << "wrote example file simple_tree.bt" << std::endl << std::endl;
    std::cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << std::endl;
    std::cout << "Hint: hit 'F'-key in viewer to see the freespace" << std::endl  << std::endl;  

    delete map;

    return 0;
}
