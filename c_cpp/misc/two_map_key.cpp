#include <iostream>

#include <map>
#include <string>

std::map<std::pair<std::string, std::string>, int> m;

inline void setVal(std::string str1, std::string str2, int val)
{
  if (str1 < str2)
    m[std::make_pair(str1, str2)] = val;
  else
    m[std::make_pair(str2, str1)] = val;
}

inline int getVal(std::string str1, std::string str2)
{
  int val;
  if (str1 < str2)
    val = m[std::make_pair(str1, str2)];
  else
    val = m[std::make_pair(str2, str1)];
  return val;
}

int main()
{
  std::cout << "aaa, bbb <= 1" << std::endl;
  setVal("aaa", "bbb", 1);
  std::cout << "bbb, aaa <= 2" << std::endl;
  setVal("bbb", "aaa", 2);

  std::cout << "aaa, bbb = " << getVal("aaa", "bbb") << std::endl;
  std::cout << "bbb, aaa = " << getVal("bbb", "aaa") << std::endl;
}
