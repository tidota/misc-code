#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

template <template <class> class T, class N>
ostream& operator<<(ostream& out, T<N> container)
{
  for (N& item: container)
  {
    out << item << ", ";
  }
  return out;
}

int main()
{
  vector<int> vec1 = {10,20,30,5,15};

  cout << "original vec1: " << endl;
  cout << vec1 << endl;

  make_heap(vec1.begin(), vec1.end());
  cout << "after make_heap: " << endl;
  cout << vec1 << endl;

  pop_heap(vec1.begin(), vec1.end());
  cout << "after pop_heap: " << endl;
  cout << vec1 << endl;

  vec1.pop_back();
  vec1.push_back(99);
  cout << "pop_back and push_back(99)" << endl;
  cout << vec1 << endl;

  cout << "is vec1 heap?: " << is_heap(vec1.begin(), vec1.end()) << endl;

  auto it = is_heap_until(vec1.begin(), vec1.end());
  cout << "vec1 is heap until " << *it
       << " at [" << distance(vec1.begin(), it) << "]" << endl;

  push_heap(vec1.begin(), vec1.end());
  cout << "after push_heap: ";
  cout << vec1 << endl;

  cout << "is vec1 heap?: " << is_heap(vec1.begin(), vec1.end()) << endl;

  sort_heap(vec1.begin(), vec1.end());
  cout << "after sort_heap: ";
  cout << vec1 << endl;
}