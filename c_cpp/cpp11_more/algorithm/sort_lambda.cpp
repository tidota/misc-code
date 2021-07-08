#include <algorithm>
#include <iostream>
#include <tuple>
#include <vector>

int main()
{
    std::vector< std::tuple<double, double> > lst = { {4, 6}, {1, 0}, {-10, 100}, {50, -10} };

    std::cout << "original" << std::endl;
    for (auto& item: lst)
    {
        std::cout << std::get<0>(item) << ", " << std::get<1>(item) << std::endl;
    }

    std::cout << std::endl << "sorting..." << std::endl << std::endl;
    std::sort(lst.begin(), lst.end(), [&](std::tuple<double, double>& t1, std::tuple<double, double>& t2){ return std::get<0>(t1) < std::get<0>(t2); });

    std::cout << "sorted" << std::endl;
    for (auto& item: lst)
    {
        std::cout << std::get<0>(item) << ", " << std::get<1>(item) << std::endl;
    }

    return 0;
}
