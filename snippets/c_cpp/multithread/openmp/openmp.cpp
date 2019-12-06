// openmp.cpp
//
// this code is just for practice.
// 
#include <iostream>
#include <omp.h>

using namespace std;

int main()
{
    cout << "start" << endl;

    #pragma omp parallel num_threads(10)
    {
        #pragma omp critical
        {
            cout << "1st half id: " << omp_get_thread_num() << endl;
        }

        for (int i = 0; i < 100000; ++i){}

        #pragma omp critical
        {
            cout << "2nd half id: " << omp_get_thread_num() << endl;
        }
    }
    cout << "end" << endl;

    return 0;
}

    
    
