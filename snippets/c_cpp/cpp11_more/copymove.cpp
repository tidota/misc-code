#include <algorithm>
#include <iostream>
#include <vector>

int main()
{
  std::vector<int> vec1 = {5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
  std::vector<int> vec2(10);
  std::vector<int> vec3;
  std::vector<int> vec4;
  std::vector<int> vec5(20, 0);
  std::vector<int> vec6;

  std::copy(vec1.begin(), vec1.end(), vec2.begin());

  std::cout << "copy vec1 to vec2: ";
  for_each(vec2.begin(), vec2.end(), [](auto n){ std::cout << n << ", "; });
  std::cout << std::endl;

  vec3.resize(4);
  std::copy_n(vec1.begin() + 1, 4, vec3.begin());
  std::cout << "copy_n vec1 (2nd - 5th) to vec3: ";
  for_each(vec3.begin(), vec3.end(), [](auto n){ std::cout << n << ", "; });
  std::cout << std::endl;

  vec4.resize(10);
  auto it = std::copy_if(
    vec1.begin(), vec1.end(), vec4.begin(), [](int n){ return n % 2 == 0; });
  std::cout << "copy_if vec1 to vec4: ";
  vec4.resize(std::distance(vec4.begin(), it));
  for_each(vec4.begin(), vec4.end(), [](auto n){ std::cout << n << ", "; });
  std::cout << std::endl;
  std::cout << "std::distance: " << std::distance(vec4.begin(), it) << "/ ";
  std::cout << "\"it - begin()\": " << it - vec4.begin() << std::endl;

  copy_backward(vec1.begin(), vec1.end(), vec5.end());
  std::cout << "copy_backward vec1 to vec5: ";
  for_each(vec5.begin(), vec5.end(), [](auto n){ std::cout << n << ", "; });
  std::cout << std::endl;

  vec6.resize(vec1.size());
  std::move(vec1.begin(), vec1.end(), vec6.begin());
  std::cout << "move of vec1 to vec6 by range " << std::endl;
  std::cout << "vec6: ";
  for_each(vec6.begin(), vec6.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;
  std::cout << "move vec6 to vec1 directly" << std::endl;
  vec1.clear();
  vec1 = std::move(vec6);
  std::cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;

  vec6.clear();
  vec6.resize(vec1.size() + 1);
  std::move_backward(vec1.begin(), vec1.begin() + 5, vec6.end());
  std::cout << "move backward vec1 to vec6" << std::endl;
  std::cout << "vec6: ";
  for_each(vec6.begin(), vec6.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;
  std::move(vec6.end() - 5, vec6.end(), vec1.begin());
  std::cout << "move vec6 to vec1" << std::endl;
  std::cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;

  vec6.clear();
  vec6.resize(5);
  std::swap(vec1, vec6);
  std::cout << "swap vec1 and vec6" << std::endl;
  std::cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;
  std::cout << "vec6: ";
  for_each(vec6.begin(), vec6.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;

  std::swap_ranges(vec1.begin(), vec1.end(), vec6.begin());
  std::cout << "swap range" << std::endl;
  std::cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;
  std::cout << "vec6: ";
  for_each(vec6.begin(), vec6.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;

  std::cout
    << "std::distance: " << std::distance(vec6.begin(), vec6.end()) << "/ ";
  std::cout
    << "\"end() - begin()\": " << vec6.end() - vec6.begin() << std::endl;

  std::cout << "recover vec1" << std::endl;
  std::cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;
  std::swap_ranges(vec1.begin(), vec1.end(), vec6.begin());
  vec1 = std::move(vec6);
  std::cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;

  std::cout << "swap 2nd and 4th in vec1" << std::endl;
  std::iter_swap(vec1.begin() + 1, vec1.begin() + 3);
  std::cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;

  std::cout << "Transform" << std::endl;
  vec2.resize(vec1.size());
  std::transform(
    vec1.begin(), vec1.end(), vec2.begin(), [](int n){ return n*n; });
  std::cout << "vec2: ";
  for_each(vec2.begin(), vec2.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;

  std::replace(vec1.begin(), vec1.end(), 8, -1);
  std::cout << "replace 8 with -1 in vec1" << std::endl;
  std::cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;

  std::replace_if(vec1.begin(), vec1.end(),
    [](int n){ return n % 2 == 0; }, -1);
  std::cout << "replace evens with -1 in vec1" << std::endl;
  std::cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;

  std::replace_copy(vec2.begin(), vec2.end(), vec1.begin(), 100, -1);
  std::cout << "replace_copy 100 with -1 in vec2 and copy to vec1" << std::endl;
  std::cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;

  std::replace_copy_if(vec2.begin(), vec2.end(), vec1.begin(),
    [](int n){ return n == 100; }, 1000);
  std::cout
    << "replace_copy_if 100 with 1000 in vec2 and copy to vec1" << std::endl;
  std::cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ std::cout << n << ", "; });
  std::cout << std::endl;



}