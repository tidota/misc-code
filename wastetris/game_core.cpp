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
#include <pthread.h>

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
    CLEAR_SCREEN();
    cout << "constructor" << endl;
    pthread_create(&update_thread, NULL, &GAME::update, (void*)1);
}

// ================================================================================= //
// Destructor
// ================================================================================= //
GAME::~GAME()
{
    cout << "destructor" << endl;
    
    pthread_join(update_thread,NULL);
    pthread_exit(NULL);
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
        CHANGE_COLOR_GREEN();
        MOVE_CURSOR(3,3);
        cout << "a character received: " << c << endl;
        DRAW_RECT_CELL(4,4,10,10);
        DRAW_RECT_CELL(12,4,14,8);
        cout << flush;
    }

    return ret_value;
}
// ================================================================================= //
// update
//
// This method is supposed to run on another thread.
// It updates the game status periodically.
// ================================================================================= //
void *GAME::update(void* v)
{
    long id = (long)v;

    if(id == 1)
    {
        cout << "id: 1" << endl;

    }
    else
    {
        cout << "id: " << id << endl;
    }

    pthread_exit(NULL);
}


