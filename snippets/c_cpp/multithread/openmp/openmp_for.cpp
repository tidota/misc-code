// openmp.cpp
//
// this code is just for practice.
// 
#include <iostream>
#include <vector>
#include <omp.h>

using namespace std;

int main()
{
    cout << "start" << endl;

    vector<int> vec(100, 10);

    #pragma omp parallel for
    for (int i = 0; i < 100; ++i)
    {
        vec[i] *= i;
        #pragma omp critical
        cout << "id: " << omp_get_thread_num() << ", vec[" << i << "] = " << vec[i] << endl;
    }

    cout << "end" << endl;

    return 0;
}

    
    
