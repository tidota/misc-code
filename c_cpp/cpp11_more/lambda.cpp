#include <iostream>

using namespace std;

class NUM
{
public:
  NUM(int n): num(n)
  {
    cout << "NUM's constructor with num = " << n << endl;
  }
  NUM(NUM& src)
  {
    cout << "NUM's copy constructor with num = " << src.num << endl;
    this->num = src.num;
  }
  void operator=(NUM& src)
  {
    cout << "NUM's assignment operator with num = " << src.num << endl;
    this->num = src.num;
  }
  int num;
};

int main()
{
    auto f = [](){ cout << "success!" << endl; };

    f();

    auto f2 = [](auto&& a, auto&& b){ return a + b; };

    cout << "f2(3,7)= " << f2(3,7) << endl;

    int a = 5;

    cout << "f2(a,7)= " << f2(a,7) << endl;

    auto f3 = [](auto& a, auto& b){ return a.num + b.num; };
    NUM c(7);
    NUM d(8);
    int result = f3(c,d);
    cout << "f3(NUM(7), NUM(8))= " << result << endl;

    return 0;
}

