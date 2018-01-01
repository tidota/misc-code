#include <iostream>

using namespace std;

class FOO
{
public:
    void func();
};

void FOO::func()
{
    cout << "This is FOO's func function." << endl;
}


template<class T>
void test(T*,void(T::*)());

int main()
{
    FOO foo;
    
    test(&foo,&FOO::func);
    
    return 0;
}

template<class T>
void test(T* p, void (T::*f)())
{
    (p->*f)();
}

