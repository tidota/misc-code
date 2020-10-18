#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

template <template <class> class T, class N>
ostream& operator<<(ostream& out, T<N> container)
{
  for (auto item: container)
  {
    out << item << ", ";
  }
  return out;
}

int main()
{
  vector<int> vec1 = {3, 5, 6, 1, 55, 57, 51};
  vector<int> vec2 = {11, 19, 15, 27, 21, 23};
  vector<int> vec3;

  cout << "vec1: " << vec1 << endl;
  cout << "vec2: " << vec2 << endl;

  cout << "sort the vectors and merge them" << endl;
  sort(vec1.begin(), vec1.end());
  sort(vec2.begin(), vec2.end());
  vec3.resize(vec1.size() + vec2.size());
  merge(vec1.begin(), vec1.end(), vec2.begin(), vec2.end(), vec3.begin());
  cout << "vec3: " << vec3 << endl;

  copy(vec1.begin(), vec1.end(), vec3.begin());
  copy(vec2.begin(), vec2.end(), vec3.begin() + vec1.size());
  inplace_merge(vec3.begin(), vec3.begin() + vec1.size(), vec3.end());
  cout << "inpace_merge: " << endl;
  cout << "vec3: " << vec3 << endl;

  if (includes(vec3.begin(), vec3.end(), vec2.begin(), vec2.end()))
    cout << "vec3 includes vec2" << endl;
  else
    cout << "vec3 does not include vec2" << endl;

  vector<int> vec4 = {-5, -10, 0, 11, 57, 5, 3};
  cout << "vec4: " << vec4 << endl;

  vector<int> vec5;
  vec5.resize(vec3.size() + vec4.size());
  auto it = set_union(
    vec3.begin(), vec3.end(), vec4.begin(), vec4.end(), vec5.begin());
  vec5.resize(it - vec5.begin());
  cout << "union (vec3 | vec4): " << endl;
  cout << "vec5: " << vec5 << endl;

  vector<int> vec6;
  vec6.resize(vec3.size() + vec4.size());
  it = set_intersection(
    vec3.begin(), vec3.end(), vec4.begin(), vec4.end(), vec6.begin());
  vec6.resize(it - vec6.begin());
  cout << "intersection (vec3 & vec4): " << endl;
  cout << "vec6: " << vec6 << endl;

  vector<int> vec7;
  vec7.resize(vec3.size() + vec4.size());
  it = set_difference(
    vec3.begin(), vec3.end(), vec4.begin(), vec4.end(), vec7.begin());
  vec7.resize(it - vec7.begin());
  cout << "difference (vec3 - vec4): " << endl;
  cout << "vec7: " << vec7 << endl;

  vector<int> vec8;
  vec8.resize(vec3.size() + vec4.size());
  it = set_symmetric_difference(
    vec3.begin(), vec3.end(), vec4.begin(), vec4.end(), vec8.begin());
  vec8.resize(it - vec8.begin());
  cout << "symmetric difference (vec3 - vec4): " << endl;
  cout << "vec8: " << vec8 << endl;
}