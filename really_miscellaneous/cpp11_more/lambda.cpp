#include <iostream>

using namespace std;

int main()
{
    auto f = [](){ cout << "success!" << endl; };

    f();

    return 0;
}

