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
    //auto ptr1 = shared_ptr<FOO>(new FOO);
    auto ptr1 = make_shared<FOO>();
    {
        auto ptr2 = ptr1;
    }
    cout << "end" << endl;
    return 0;
}



