// std_thread.cpp
//
// this code demonstrates std::thread to perform multi-threading.
// 
#include <iostream>
#include <thread>
#include <mutex>

using namespace std;

class FOO
{
private:
    thread t;
    mutex mtx;
public:
    FOO();
    ~FOO();

    void func(int val);
};

FOO::FOO()
{
    cout << "constructor" << endl;
    t = thread(&FOO::func,this,0);
}

FOO::~FOO()
{
    mtx.lock();
    cout << "destructor" << endl;
    cout << "waiting for the thread to end" << endl;
    mtx.unlock();
    t.join();
    cout << "end" << endl;
}

void FOO::func(int val)
{
    for(int i = 0; i < 100; i++)
    {
        mtx.lock();
        cout << "No. " << i << ": func() with the value: " << val << endl;
        mtx.unlock();
    }
}

int main()
{
    FOO foo;

    foo.func(1);

    return 0;
}

    
    
