#include <iostream>
#include <functional>

using namespace std;

class NUM
{
private:
  int num;
  function<void(void)> func;

public:
  NUM(): num(0)
  {
    cout << "NUM's constructor with num = " << num << endl;

    func = [&](){ num = 100; };
  }
  void call()
  {
    cout << "calling the lambda function" << endl;
    func();
  }
  void print()
  {
    cout << "num = " << num << endl;
  }
};

int main()
{
  cout << "This is demonstrating to store and call a lambda function as a"
       << "member variable in a class" << endl << endl;
  NUM numobj;
  numobj.print();
  numobj.call();
  numobj.print();

  return 0;
}
