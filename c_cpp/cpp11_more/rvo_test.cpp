#include <iostream>

using namespace std;

class FOO
{
public:
  FOO()
  {
    cout << "constructor" << endl;
  }
  ~FOO()
  {
    cout << "destructor" << endl;
  }
  FOO(FOO& src)
  {
    cout << "copy constructor" << endl;
  }
  void operator=(FOO& src)
  {
    cout << "copy assignment" << endl;
  }
};

FOO func1()
{
  FOO foo;
  return foo;
}

FOO func2(int n)
{
  FOO foo1, foo2;
  if (n == 0)
    return foo1;
  else
    return foo2;
}


int main()
{
  {
    cout << "========= func1 ===========" << endl;
    cout << "---- RVO works ----" << endl;
    FOO foo = func1();
    cout << "---- end of the scope ----" << endl;
  }
  {
    cout << "========= func2 ===========" << endl;
    cout << "---- RVO does not work for this ----" << endl;
    FOO bar = func2(3);
    cout << "---- end of the scope ----" << endl;
  }

  return 0;
}