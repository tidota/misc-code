#include <iostream>
#include <memory>

using namespace std;

class FOO
{
public:
    FOO()
    {
        cout << "FOO's constructor" << endl;
    }
    ~FOO()
    {
        cout << "FOO's destructor" << endl;
    }
};

int main()
{
    //auto ptr = unique_ptr<FOO>(new FOO);
    auto ptr = make_unique<FOO>();

    return 0;
}

