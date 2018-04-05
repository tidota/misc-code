#include <iostream>
#include <tuple>

using namespace std;

int main()
{
    // C++17
    //auto [a, b] = make_tuple(1.5, 3);
    //cout << "a: " << a << ", b: " << b << endl;

    tuple<int, double> t = make_tuple(5, 6.5);
    cout << "t0: " << get<0>(t) << ", t1: " << get<1>(t) << endl;

    return 0;
}

