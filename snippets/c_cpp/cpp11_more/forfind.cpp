#include <algorithm>
#include <iostream>
#include <vector>


void func(int num)
{
  std::cout << num << ", ";
}

bool comp(int n1, int n2)
{
  return ((n1 % 2) == (n2 % 2));
}

int main()
{
  std::vector<int> vec1 = {1, 2, 3, 4, 5, 6, 7};
  std::vector<int> vec2 = {10, 11, 12};
  std::vector<int> vec3 = {11, 12, 13, 15, 16};
  std::vector<int> vec4 = {21, 20, 19, 18, 17, 16, 15};
  std::vector<int> vec5 = {3, 5, 7, 2, 6, 1, 4};

  std::cout << "vec1: ";
  for_each (vec1.begin(), vec1.end(), func);
  std::cout << std::endl;
  std::cout << "vec2: ";
  for_each (vec2.begin(), vec2.end(), [](auto n){ std::cout << n << "| "; });
  std::cout << std::endl;
  std::cout << "vec3: ";
  for_each (vec3.begin(), vec3.end(), [](auto n){ std::cout << n << "| "; });
  std::cout << std::endl;
  std::cout << "vec4: ";
  for_each (vec4.begin(), vec4.end(), [](auto n){ std::cout << n << "| "; });
  std::cout << std::endl;
  std::cout << "vec5: ";
  for_each (vec5.begin(), vec5.end(), [](auto n){ std::cout << n << "| "; });
  std::cout << std::endl;

  std::vector<int>::iterator it;
  it = find (vec1.begin(), vec1.end(), 3);
  std::cout << "find (= 3): " << *it << std::endl;
  it = find_if (vec1.begin(), vec1.end(), [](auto n){ return n > 3; });
  std::cout << "find_if (> 3): " << *it << std::endl;
  it = find_if_not (vec1.begin(), vec1.end(), [](auto n){ return n == 1; });
  std::cout << "find_if_not (!= 1): " << *it << std::endl;

  it = find_first_of (vec1.begin(), vec1.end(), vec2.begin(), vec2.end(), comp);
  std::cout << "find_first_of: " << *it << std::endl;

  it = adjacent_find (vec3.begin(), vec3.end(), comp);
  std::cout << "adjacent_find: " << *it << std::endl;

  it = search (vec1.begin(), vec1.end(), vec2.begin(), vec2.end(), comp);
  std::cout << "search: " << *it << std::endl;
  it = search_n (vec3.begin(), vec3.end(), 2, 99, comp);
  std::cout << "search_n: " << *it << std::endl;
  it = find_end (vec1.begin(), vec1.end(), vec2.begin(), vec2.end(), comp);
  std::cout << "find_end: " << *it << std::endl;

  std::cout << "# of 2: " << count(vec1.begin(), vec1.end(), 2) << std::endl;
  std::cout << "# of evens: "
    << count_if (vec1.begin(), vec1.end(), [](auto n){ return n % 2 == 0; })
    << std::endl;
  auto pair
    = mismatch (vec1.begin(), vec1.end(), vec3.begin(), vec3.end(), comp);
  std::cout << "mismatch in vec1 and vec3: "
    << *pair.first << ", " << *pair.second << std::endl;

  std::cout << "vec1 == vec4 for the first three: "
    << equal (vec1.begin(), vec1.end(), vec4.begin(), comp) << std::endl;

  std::cout << "vec1 vec5 permutation: "
    << is_permutation (vec1.begin(), vec1.end(), vec5.begin()) << std::endl;

  std::cout << "vec4 vec5 permutation: "
    << is_permutation (vec4.begin(), vec4.end(), vec5.begin(), comp)
    << std::endl;
}