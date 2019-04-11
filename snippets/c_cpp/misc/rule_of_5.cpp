#include <iostream>

using namespace std;

class FOO
{
public:
  FOO()
  {
    cout << "FOO's default constructor" << endl;
  }
  FOO(const FOO& f)
  {
    cout << "FOO's copy constructor" << endl;
  }
  FOO(FOO&& f)
  {
    cout << "FOO's move constructor" << endl;
  }
  virtual ~FOO()
  {
    cout << "FOO's destructor" << endl;
  }

  FOO& operator=(const FOO& f)
  {
    cout << "FOO's copy assignment" << endl;
    return *this;
  }
  FOO& operator=(FOO&& f)
  {
    cout << "FOO's move assignment" << endl;
    return *this;
  }
};

int main()
{
  {
    cout << " ========================= " << endl;
    cout << " FOO f1, f2;" << endl;
    cout << " f1 = f2;" << endl;
    FOO f1, f2;
    f1 = f2;
  }
  {
    cout << " ========================= " << endl;
    cout << " FOO f1, f2;" << endl;
    cout << " f1 = move(f2);" << endl;
    FOO f1, f2;
    f1 = move(f2);
  }
  {
    cout << " ========================= " << endl;
    cout << " FOO f1;" << endl;
    cout << " FOO f2(f1);" << endl;
    FOO f1;
    FOO f2(f1);
  }
  {
    cout << " ========================= " << endl;
    cout << " FOO f1;" << endl;
    cout << " FOO f2(move(f1));" << endl;
    FOO f1;
    FOO f2(move(f1));
  }
  {
    cout << " ========================= " << endl;
    cout << " FOO f1 = FOO();" << endl;
    cout << " FOO f2(FOO());" << endl;
    cout << " FOO f3 = f1;" << endl;
    FOO f1 = FOO();
    FOO f2(FOO());
    FOO f3 = f1;
  }
  return 0;
}