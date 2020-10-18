#include <iostream>

using namespace std;

class FOO
{
private:
  int val;
public:
  FOO()
  {
    val = 10;
  }
  friend ostream& operator<<(ostream&, FOO&);
};

ostream& operator<<(ostream& out, FOO& f)
{
  out << "FOO!!!: " << f.val;
  return out;
}

int main()
{
  FOO foo;
  cout << "yahoooooo!, " << foo << endl;

  return 0;
}