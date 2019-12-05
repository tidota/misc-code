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
    static mutex mtx;
public:
    FOO(int num);
    ~FOO();

    void func(int val);
};

mutex FOO::mtx;

FOO::FOO(int num)
{
    cout << "constructor" << endl;
    t = thread(&FOO::func,this,num);
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
	cout << "[" << t.get_id() << "] ";
        cout << "No. " << i << ": func() with the value: " << val << endl;
        mtx.unlock();
    }
}

int main()
{
    FOO foo1(1);
    FOO foo2(2);

    foo1.func(10);
    foo2.func(20);

    return 0;
}

    
    
