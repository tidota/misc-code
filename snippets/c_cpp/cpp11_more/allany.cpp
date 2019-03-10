#include <algorithm>
#include <iostream>
#include <vector>

int main()
{
  std::vector<int> vec1(5, 0);
  std::vector<int> vec2 = {0, 1, 0, 0, 0};

  std::cout << "All of them are 0?: ";
  std::cout
    << std::all_of(vec1.begin(), vec1.end(), [](auto n){ return n == 0; });
  std::cout << std::endl;

  std::cout << "Any of them is 1?: ";
  std::cout
    << std::any_of(vec2.begin(), vec2.end(), [](auto n){ return n == 1; });
  std::cout << std::endl;

  std::cout << "None of them is 5?: ";
  std::cout
    << std::none_of(vec1.begin(), vec2.end(), [](auto n){ return n == 5; });
  std::cout << std::endl;

  return 0;
}
