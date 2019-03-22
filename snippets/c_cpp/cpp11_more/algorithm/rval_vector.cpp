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


class BASE
{
public:
  BASE()
  {
    cout << "BASE's constructor" << endl;
  }
  virtual ~BASE()
  {
    cout << "BASE's destructor" << endl;
  }
};

class FOO: public BASE
{
private:
  int val;
public:
  FOO(const int& param): val(param)
  {
    cout << "FOO's constructor" << endl;
  }
  FOO(const FOO& src): val(src.val)
  {
    cout << "FOO's copy constructor" << endl;
  }
  FOO(const FOO&& src): val(move(src.val))
  {
    cout << "FOO's move constructor" << endl;
  }
  virtual ~FOO()
  {
    cout << "FOO's destructor" << endl;
  }
  void operator=(const FOO& src)
  {
    cout << "FOO's copy assignment" << endl;
    val = src.val;
  }
  void operator=(const FOO&& src)
  {
    cout << "FOO's move assignment" << endl;
    val = move(src.val);
  }
  void func()
  {
    cout << "FOO's func (val = " << val << ")" << endl;
  }
};


FOO foo(int i)
{
  if (i == 0)
  {
    cout << "making FOO with -1" << endl;
    return FOO(-1);
  }

  {
    cout << "making FOO with " << i << endl;
    return FOO(i);
  }
}

int main()
{
  int n = 10;
  func({n});

  auto f = foo(0);
  f.func();
  cout << "------------------" << endl;
  auto b = foo(10);
  b.func();

  cout << "------------------" << endl;
  return 0;
}