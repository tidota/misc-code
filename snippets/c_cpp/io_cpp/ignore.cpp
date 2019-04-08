#include <iostream>

using namespace std;

int main()
{
  int val;

  while (true)
  {
    cout << "integer? >";
    cin >> val;
    if (cin.good())
      break;

    cout << "give a number!" << endl;
    cin.clear();
    cin.ignore(256, '\n');
  }
  cout << "val: " << val << endl;

  return 0;
}