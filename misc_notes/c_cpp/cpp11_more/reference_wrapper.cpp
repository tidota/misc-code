#include <iostream>
#include <memory>
#include <vector>
#include <functional>

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
    auto foo = make_unique<FOO>();
    vector< reference_wrapper<decltype(foo)> > v = {foo};

    cout << "the middle of the main" << endl;

    auto bar = move(foo);

    return 0;
}


