#include <iostream>
#include "noncanonical.hpp"

using namespace std;

// ============================================================================== //
// main
//
// Description:
//   The main function first sets up the environment to a non-canonical mode.
//   It then repeatedly interacts with the user while updating the state and display.
//   When the user tells to stop or the interaction is supposed to be end,
//   it breaks the loop.
//   At the end, it does the final task including deletion of the executable itself.
// 
// The condition to break the loop:
//   when the interaction is supposed to end, e.g., the game is over.
//   when the user gives Ctrl-D.
//
// ============================================================================== //
int main()
{
    int f_fail = set_input_mode();

    int f_continue = (f_fail == 0)? 1: 0;

    while(f_continue)
    {
        char c = readOneChar();
        if(c != 0x04)
        {
            cout << c << flush;
        {
        else
        {
            f_continue = 0;
        }
    }

    return f_fail;
}


