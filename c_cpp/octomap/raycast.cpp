// raycast.cpp
// 05/13/2021

//#include <stdio.h>
#include <iostream>
#include <iomanip>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

int main(int argc, char** argv )
{

    octomap::OcTree *map;

    map = new octomap::OcTree(0.25);
    map->setProbHit(0.7);
    map->setProbMiss(0.4);
    map->setClampingThresMin(0.12);
    map->setClampingThresMax(0.97);

    std::cout << "creating a simple map ...";
    for (double x = 0; x < 1.0; x += 0.05f)
    {
        octomap::point3d endpoint ((float) x, (float) 0, (float) 0);
        map->updateNode(endpoint, false);
    }
    {
        octomap::point3d endpoint ((float) 1.0, (float) 0, (float) 0);
        map->updateNode(endpoint, true);
    }
    std::cout << "done" << std::endl;

    std::cout << "raycast from the origin" << std::endl;
    {
        octomap::point3d start(0, 0, 0);
        octomap::point3d dir(1, 0, 0);
        octomap::point3d hit;
        if (map->castRay(start, dir, hit, true, 10))
        {
            std::cout << "hit norm: " << hit.norm() << std::endl;
        }
        else
        {
            std::cout << "missed!" << std::endl;
        }
    }

    std::cout << "raycast from the far point" << std::endl;
    {
        octomap::point3d start(10, 0, 0);
        octomap::point3d dir(-1, 0, 0);
        octomap::point3d hit;
        if (map->castRay(start, dir, hit, true, 10))
        {
            std::cout << "hit norm: " << hit.norm() << std::endl;
        }
        else
        {
            std::cout << "missed!" << std::endl;
        }
    }

    std::cout << "outward raycast inside the cell" << std::endl;
    {
        octomap::point3d start(1.1, 0, 0);
        octomap::point3d dir(1, 0, 0);
        octomap::point3d hit;
        if (map->castRay(start, dir, hit, true, 10))
        {
            std::cout << "hit norm: " << hit.norm() << std::endl;
        }
        else
        {
            std::cout << "missed!" << std::endl;
        }
    }

    std::cout << "inward raycast inside the cell" << std::endl;
    {
        octomap::point3d start(1.1, 0, 0);
        octomap::point3d dir(-1, 0, 0);
        octomap::point3d hit;
        if (map->castRay(start, dir, hit, true, 10))
        {
            std::cout << "hit norm: " << hit.norm() << std::endl;
        }
        else
        {
            std::cout << "missed!" << std::endl;
        }
    }

    delete map;

    return 0;
}
