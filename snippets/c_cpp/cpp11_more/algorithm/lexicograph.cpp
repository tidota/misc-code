#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

int main()
{
  string str1 = "Aabbb";
  string str2 = "abb";
  cout << str1 << " < " << str2 << ": "
       << lexicographical_compare(
         str1.begin(), str1.end(), str2.begin(), str2.end()) << endl;
  cout << "Next of " << str1 << ": ";
  next_permutation(str1.begin(), str1.end());
  cout << str1 << endl;
  cout << "Prev of " << str2 << ": ";
  prev_permutation(str2.begin(), str2.end());
  cout << str2 << endl;
}