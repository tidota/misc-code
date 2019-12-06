// openmp.cpp
//
// this code is just for practice.
// 
#include <iostream>
#include <mutex>
#include <omp.h>

using namespace std;

int main()
{
    cout << "start" << endl;

    mutex mtx;

    #pragma omp parallel num_threads(10)
    mtx.lock();
    cout << "id: " << omp_get_thread_num() << endl;
    mtx.unlock();

    return 0;
}

    
    
