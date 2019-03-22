#include <iostream>
#include <vector>

using namespace std;

void func(const vector<int>& vec)
{
  cout << "vector: ";
  for (auto n: vec)
  {
    cout << n << ", ";
  }
  cout << endl;
}

int main()
{
  int n = 10;
  func({n});

  return 0;
}