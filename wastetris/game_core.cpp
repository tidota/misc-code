// game_core.cpp
//
// This file contains the core of the game program.
// It maintains updates of the state and displaying the results based on the given character input.
//
// The object of the GAME class must be singleton, so its constructor/destructor are not public.
// To start a game, it is required to call the init_game method.
// The user also must call the end_game method to finish.
// But if the game is over, the object is automatically destroyed.
//

#include "game_core.hpp"
#include "format_macro.hpp"
#include <iostream>
#include <iomanip>

using namespace std;

// ================================================================================= //
// the pointer to the object is initialized with NULL.
// so the program can tell if there is an existing one.
// ================================================================================= //
GAME* GAME::game = NULL;

// ================================================================================= //
// Constructor
// ================================================================================= //
GAME::GAME()
{
    cout << "constructor" << endl;
}

// ================================================================================= //
// Destructor
// ================================================================================= //
GAME::~GAME()
{
    cout << "destructor" << endl;
}

// ================================================================================= //
// init_game
//
// This function initializes the state of the game, and restarts the thread.
// 
// ================================================================================= //
void GAME::init_game()
{
    if(game != NULL)
        delete game;
    game = new GAME();
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
int GAME::play_game(char c)
{
    int ret_value = 1;

    if(c == '\x04')
    {
        ret_value = 0;
        CHANGE_COLOR_BLACK();
        cout << "Ctrl-D received" << endl;
        delete game;
        game = NULL;
    }
    else
    {
        CLEAR_SCREEN();
        CHANGE_COLOR_GREEN();
        cout << "a character received: " << c << endl;
    }

    return ret_value;
}
