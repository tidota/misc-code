// game_core.cpp
//
// This file contains the core of the game program.
// It maintains updates of the state and displaying the results based on the given character input.
//

#include <iostream>
#include <iomanip>

using namespace std;

// ================================================================================= //
// init_game
//
// This function initializes the state of the game, and restarts the thread.
// 
// ================================================================================= //
void init_game()
{
}

// ================================================================================= //
// play_game
//
// This function updates the state of the game based on the given character.
//
// input:
//   char c: a character provided by the user
// output:
//   the state to continue the game
//       1: running
//       0: stopped
//       others: error or the game is not running correctly
//       
// ================================================================================= //
int play_game(char c)
{
    int ret_value = 1;

    if(c == '\x04')
    {
        ret_value = 0;
        cout << "Ctrl-D received" << endl;
    }
    else
    {
        cout << "a character received: " << c << endl;
    }

    return ret_value;
}
