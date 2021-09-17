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
  cout << "================================================" << endl;
  {
    vector<int> vec = {2, 3, 10, 5, 6, 5, 5, 4, 4, 6, 6};
    cout << " vector: " << endl << vec << endl;
    std::sort(vec.begin(), vec.end());
    cout << " vector (sorted): " << endl << vec << endl;
    {
      cout << "vec (4 <= item <= 6): ";
      auto it_start = lower_bound(vec.begin(), vec.end(), 4);
      auto it_end = upper_bound(vec.begin(), vec.end(), 6);
      for (auto it = it_start; it != it_end; ++it)
        cout << *it << ", ";
      cout << endl;
      cout << "beginnin: " << (it_start - vec.begin()) << endl;
      cout << "last:     " << (it_end - vec.begin()) << endl;
    }
    cout << endl;
    auto item = binary_search(vec.begin(), vec.end(), 4, [](int a, int b){ return a + 1 == b; });
    cout << "found for 4?: " << item << endl << endl;
  }

  {
    vector<int> vec = {2, 3, 10, 5, 5, 5, 1, 9, 1};
    cout << " vector: " << endl << vec << endl;
    std::sort(vec.begin(), vec.end());
    cout << " vector (sorted): " << endl << vec << endl;
    {
      cout << "vec (4 <= item <= 6): ";
      auto it_start = lower_bound(vec.begin(), vec.end(), 4);
      auto it_end = upper_bound(vec.begin(), vec.end(), 6);
      for (auto it = it_start; it != it_end; ++it)
        cout << *it << ", ";
      cout << endl;
      cout << "beginnin: " << (it_start - vec.begin()) << endl;
      cout << "last:     " << (it_end - vec.begin()) << endl;
    }
    cout << endl;
    auto item = binary_search(vec.begin(), vec.end(), 4, [](int a, int b){ return a + 1 == b; });
    cout << "found for 4?: " << item << endl << endl;
  }

  cout << "================================================" << endl;
  {
    set<int> s;
    for (auto item: {2, 3, 10, 5, 5, 5, 1, 9, 1}) s.insert(item);

    cout << "set: " << s << endl << endl;
    {
      cout << "set (4 <= item <= 6): ";
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
  }

  return 0;
}
