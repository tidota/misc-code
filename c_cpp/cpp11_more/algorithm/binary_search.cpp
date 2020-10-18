#include <algorithm>
#include <iostream>
#include <set>
#include <vector>

using namespace std;

template <template <class> class T, class N>
ostream& operator<<(ostream& out, T<N> container)
{
  for (auto item: container)
    out << item << ", ";
  return out;
}

int main()
{
  vector<int> vec1 = {2, 3, 10, 5, 6, 5, 5};
  set<int> s;
  for (auto item: vec1) s.insert(item);

  cout << "vec1: " << vec1 << endl;
  cout << "set: " << s << endl;

  {
    cout << "set (4 <= item < 6): ";
    auto it_start = lower_bound(s.begin(), s.end(), 4);
    auto it_end = upper_bound(s.begin(), s.end(), 6);
    for (auto it = it_start; it != it_end; ++it)
      cout << *it << ", ";
    cout << endl;
  }
  {
    cout << "set (item == 5): ";
    auto range = equal_range(s.begin(), s.end(), 5);
    for (auto it = range.first; it != range.second; ++it)
      cout << *it << ", ";
    cout << endl;
  }

  auto item = binary_search(vec1.begin(), vec1.end(), 4, [](int a, int b){ return a + 1 == b; });
  cout << "found for 4?: " << item << endl;


}