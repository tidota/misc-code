// std_thread.cpp
//
// this code demonstrates std::thread to perform multi-threading.
// 
#include <iostream>
#include <thread>
#include <vector>

#define N 100000000
#define T 10

using namespace std;

void update(vector<int>& vec, int start, int end)
{
    for (int i = start; i < end; ++i)
        vec[i] += 100;
}

int main()
{
    vector<int> vec(N, 0);

    cout << "=========================================" << endl;
    cout << "creating " << T << " threads" << endl;

    vector<thread> ths;
    for (unsigned int i = 0; i < T; ++i)
    {
        ths.push_back(thread(update, ref(vec), i*N/T, (i+1)*N/T));
    }

    cout << "=========================================" << endl;
    cout << "waiting..." << endl;

    for (unsigned int i = 0; i < ths.size(); ++i)
    {
        ths[i].join();
    }
    
    cout << "=========================================" << endl;
    cout << "done" << endl;

    return 0;
}

