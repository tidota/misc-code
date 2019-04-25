#include <iostream>

using namespace std;

class FOO
{
public:
  FOO(){}
  ~FOO(){}
  void caller()
  {
    func();
  }
private:
  virtual void func()
  {
    cout << "FOO's func" << endl;
  }
};

class BAR: public FOO
{
public:
  BAR(){}
  ~BAR(){}
private:
  void func()
  {
    cout << "BAR's func" << endl;
  }
};

int main()
{
  {
    FOO foo;
    foo.caller();
  }
  cout << "------------------" << endl;
  {
    BAR bar;
    bar.caller();
  }
}