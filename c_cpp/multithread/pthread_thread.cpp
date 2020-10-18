// pthread.cpp

// this program demonstrate pthread in c++
// each process executes proc function

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <pthread.h>

using namespace std;

pthread_mutex_t *m_lock;
int *list;

void *proc(void *id)
{
    long tid;
    tid = (long)id;
    cout <<  "Hellow World! thread " << tid << endl;
    pthread_exit(NULL);
}

int main(int argc, char *argv[])
{
    if(argc < 2)
        return -1;

    int thrnum;
    stringstream ss(argv[1]);
    ss >> thrnum;

    pthread_t* threads;
    threads = new pthread_t[thrnum];

    

    int rc;
    long t;
    for(t=0;t<thrnum;t++)
    {
        cout << "main thread: creating thread " << t << endl;
        rc = pthread_create(&threads[t], NULL, proc, (void*)t);
        if(rc)
        {
            cout << "ERROR: return code from pthread_create is " << rc << endl;
            exit(-1);
        }
    }

    for(t=0;t<thrnum;t++)
    {
        pthread_join(threads[t],NULL);
    }


    delete threads;
    pthread_exit(NULL);
}

