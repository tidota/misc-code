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
    {
        cout << "========================================================" << endl;
        cout << "calling a lambda function accessing an outside item" << endl;
        std::string str = "outside message";
        auto f = [&](){ cout << str << endl; cout << "success!" << endl; };
        f();
    }

    {
        cout << "========================================================" << endl;
        cout << "calling a lambda function taking two rvalues" << endl;
        auto f = [](auto&& a, auto&& b){ return a + b; };
        cout << "f2(3,7)= " << f(3,7) << endl;
        int a = 5;
        cout << "f2(a,7)= " << f(a,7) << endl;
    }

    {
        cout << "========================================================" << endl;
        cout << "calling a lambda function taking two lvalues" << endl;
        auto f = [](auto& a, auto& b){ return a.num + b.num; };
        NUM c(7);
        NUM d(8);
        int result = f(c,d);
        cout << "f3(NUM(7), NUM(8))= " << result << endl;
    }

    {
        cout << "========================================================" << endl;
        NUM c(7);
        cout << "c.num = " << c.num << endl;
        cout << "calling a lambda function modifying a copied item" << endl;
        auto f = [=]() mutable { c.num = 10; };
        f();
        cout << "c.num = " << c.num << endl;
    }
    
    {
        cout << "========================================================" << endl;
        NUM c(7);
        cout << "c.num = " << c.num << endl;
        cout << "calling a lambda function modifying a reference" << endl;
        auto f = [&](){ c.num = 10; };
        f();
        cout << "c.num = " << c.num << endl;
    }
    
    return 0;
}

