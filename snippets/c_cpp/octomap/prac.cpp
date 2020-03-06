#include <stdio.h>

#include <octomap/octomap.h>


int main(int argc, char** argv )
{

    octomap::OcTree *map;

    map = new octomap::OcTree(0.25);
    map->setProbHit(0.7);
    map->setProbMiss(0.4);
    map->setClampingThresMin(0.12);
    map->setClampingThresMax(0.97);

    delete map;

    return 0;
}
