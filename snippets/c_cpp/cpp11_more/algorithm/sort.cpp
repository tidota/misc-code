#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

template <class T>
ostream& operator<<(ostream& out, vector<T> vec)
{
  for (auto& n: vec)
  {
    out << n << ", ";
  }

  return out;
}

int main()
{
  vector<int> vec1 = {3, 4, 1, 9, 0, 6};
  cout << "vec1: " << vec1 << endl;
  cout << "is sorted: "
       << is_sorted(vec1.begin(), vec1.end(), [](int a, int b){ return a > b; })
       << endl;
  cout << "sorting..." << endl;
  sort(vec1.begin(), vec1.end(), [](int a, int b){ return a > b; });
  cout << "vec1: " << vec1 << endl;
  cout << "is sorted: "
       << is_sorted(vec1.begin(), vec1.end(), [](int a, int b){ return a > b; })
       << endl;

  vector<double> vecd = {2.2, 1.5, 0.5, 1.2, 0.1, 1.8, 2.5, 3.9};
  cout << "vecd: " << vecd << endl;
  cout << "stable sorting..." << endl;
  stable_sort(
    vecd.begin(), vecd.end(),
    [](double a, double b){ return (int)a > (int)b; });
  cout << "vecd: " << vecd << endl;

  vector<int> vec2 = {4, 2, 3, 1, 6, 5};
  cout << "vec2: " << vec2 << endl;
  cout << "partial sorting..." << endl;
  partial_sort(
    vec2.begin(), vec2.begin() + 3, vec2.end(),
    [](int a, int b){ return a > b; });
  cout << "vec2: " << vec2 << endl;
  cout << "sorted until: "
       << *(is_sorted_until(vec2.begin(), vec2.end(),
            [](int a, int b){ return a > b; }))
       << endl;

  cout << "initializing vec2" << endl;
  vec2 = {4, 2, 3, 1, 6, 5};
  cout << "vec2: " << vec2 << endl;
  cout << "partial sorting and copying..." << endl;
  vector<int> vec3(3);
  partial_sort_copy(
    vec2.begin(), vec2.end(), vec3.begin(), vec3.end(),
    [](int a, int b){ return a > b; });
  cout << "vec3: " << vec3 << endl;

  vector<int> vec4 = {1, 2, 3, 4, 5, 6, 7, 8};
  random_shuffle(vec4.begin(), vec4.end());
  cout << "vec4: " << vec4 << endl;
  cout << "nth element sorting..." << endl;
  nth_element(
    vec4.begin(), vec4.begin() + 3, vec4.end(),
    [](int a, int b){ return a > b; });
  cout << "vec4: " << vec4 << endl;
}