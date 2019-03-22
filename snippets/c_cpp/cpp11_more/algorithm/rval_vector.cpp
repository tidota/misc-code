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
private:
  static int id_count;
protected:
  int id;
public:
  BASE(): id(id_count++)
  {
    cout << "BASE's constructor (" << id << ")" << endl;
  }
  virtual ~BASE()
  {
    cout << "BASE's destructor (" << id << ")" << endl;
  }
};

int BASE::id_count = 0;

class FOO: public BASE
{
private:
  int val;
public:
  FOO(const int& param): val(param)
  {
    cout << "FOO's constructor (" << id << ")" << endl;
  }
  FOO(const FOO& src): val(src.val)
  {
    cout << "FOO's copy constructor (" << id << ")" << endl;
  }
  FOO(const FOO&& src): val(move(src.val))
  {
    cout << "FOO's move constructor (" << id << ")" << endl;
  }
  virtual ~FOO()
  {
    cout << "FOO's destructor (" << id << ")" << endl;
  }
  void operator=(const FOO& src)
  {
    cout << "FOO's copy assignment (" << id << ")" << endl;
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


FOO&& foo(FOO&& src)
{
  cout << "returning a FOO object" << endl;
  src.func();
  return move(src);
}

int main()
{
  cout << "Giving a rvalue to a function" << endl;
  int n = 10;
  func({n});

  cout << "------------------" << endl;
  {
    cout << "Passing a rvalue to a function and returning it" << endl;
    auto f = foo(FOO(-1));
    f.func();
  }
  cout << "------------------" << endl;
  {
    cout << "Converting a glvalue to a xvalue, passing it to a function"
         << "and returning it" << endl;
    auto f_temp = FOO(10);
    cout << &f_temp << endl;
    auto&& b = foo(move(f_temp));
    cout << &b << endl;
    b.func();
  }
  cout << "------------------" << endl;

  return 0;
}