#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

template <template <class> class T, class N>
ostream& operator<<(ostream& out, T<N> cont)
{
  for (N& item: cont)
    out << item << ", ";
  return out;
}

int main()
{
  cout << "min(1, 3): " << min(1, 3) << endl;
  cout << "max(3, 2): " << max(3, 2) << endl;

  vector<int> vec = {3, 2, 5, 6};

  cout << "min({3, 2, 5, 6}): " << min({3, 2, 5, 6}) << endl;
  cout << "max({3, 2, 5, 6}): " << max({3, 2, 5, 6}) << endl;

  auto result = minmax({3, 2, 5, 6});
  cout << "minmax ({3, 2, 5, 6}): "
       << result.first << ", " << result.second << endl;

  auto min_val = min_element(vec.begin(), vec.end());
  auto max_val = max_element(vec.begin(), vec.end());
  cout << "min at " << distance(vec.begin(), min_val) << endl;
  cout << "max at " << distance(vec.begin(), max_val) << endl;

  auto minmax_elements = minmax_element(vec.begin(), vec.end());
  cout << "minmax_elements.first: " << *minmax_elements.first << endl;
  cout << "minmax_elements.second: " << *minmax_elements.second << endl;
}