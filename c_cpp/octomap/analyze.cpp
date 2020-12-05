// analyze.cpp
// 12/04/2020
//
// this program extracts information from a given .bt file.

// for reference,
// https://octomap.github.io/octomap/doc/classoctomap_1_1OcTree.html

#include <stdio.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Pose6D.h>

int main(int argc, char** argv )
{
    if (argc != 2)
    {
        printf("Wrong argument\n");
        return -1;
    }

    octomap::OcTree *map;
    map = new octomap::OcTree(0.1);

    if (!map->readBinary(argv[1]))
    {
        printf("File NOT OPEN: %s\n", argv[1]);
        return -1;
    }

    printf("File: %s\n", argv[1]);
    printf("ProbHit: %f\n", map->getProbHit());
    printf("ProbMiss: %f\n", map->getProbMiss());
    printf("ClampingThresMin: %f\n", map->getClampingThresMin());
    printf("ClampingThresMax: %f\n", map->getClampingThresMax());

    //octomap::point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
    //map->updateNode(endpoint, true); // integrate 'occupied' measurement

    //octomap::point3d query (0., 0., 0.);
    //octomap::OcTreeNode* node = map->search (query);
    //if (node != NULL)
    //    std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
    //else 
    //    std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;    

    //map->writeBinary("simple_tree.bt");

    delete map;

    return 0;
}
