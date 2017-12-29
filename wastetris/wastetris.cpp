#include <iostream>
#include "noncanonical.hpp"

using namespace std;

int main()
{
    int f_fail = set_input_mode();

    int f_continue = (f_fail == 0)? 1: 0;

    while(f_continue)
    {
        char c = readOneChar();
        if(c != 0x04)
            cout << c << flush;
        else
            f_continue = 0;
    }

    return f_fail;
}


