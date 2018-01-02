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
#include <unistd.h>

using namespace std;

// ================================================================================= //
// the pointer to the object is initialized with NULL.
// so the program can tell if there is an existing one.
// ================================================================================= //
GAME* GAME::game = NULL;

// ================================================================================= //
// Constructor
//
// It prepares a mutex and thread.
//
// ================================================================================= //
GAME::GAME()
{
    CLEAR_SCREEN();
    CURSOR_OFF();
    f_stat = 1;
    pthread_mutex_init(&mtx, NULL);
    pthread_create(&update_thread, NULL, GAME::run, (void*)this);

    temp = 'a';
}

// ================================================================================= //
// Destructor
//
// It waits for the other thread to end, and destroys the mutex.
//
// ================================================================================= //
GAME::~GAME()
{
    pthread_join(update_thread, NULL);
    pthread_mutex_destroy(&mtx);

    int width = 100;
    int height = 50;

    for(int i = height; i >= 1; i--)
    {
        if(i - 4 >= 1)
        {
            DRAW_HLINE_C(i - 4, 1, 1, string(width,'#')); 
            FLUSH();
        }
        if(i - 3 >= 1)
        {
            DRAW_HLINE_C(i - 3, 1, 1, string(width,'*'));
            FLUSH();
        }
        if(i - 2 >= 1)
        {
            DRAW_HLINE_C(i - 2, 1, 1, string(width,';'));
            FLUSH();
        }
        if(i - 1 >= 1)
        {
            DRAW_HLINE_C(i - 1, 1, 1, string(width,'.'));
            FLUSH();
        }
        if(i - 0 >= 1)
        {
            DRAW_HLINE_C(i - 0, 1, 1, string(width,' '));
            FLUSH();
        }
        usleep(40000);
    }

    CURSOR_ON();
    MOVE_CURSOR(1,1);
    cout << "go back to work now" << endl;
}

// ================================================================================= //
// init_game
//
// This function initializes the state of the game, and restarts the thread.
// 
// ================================================================================= //
GAME *GAME::init_game()
{
    if(game != NULL)
    {
        game->kill_game();
    }
    game = new GAME();

    return game;
}

// ================================================================================= //
// kill_game
// This function stops the running game and kills the thread.
// 
// ================================================================================= //
void GAME::kill_game()
{
    if(game != NULL)
    {
        delete game;
        game = NULL;
    }
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
    pthread_mutex_lock(&mtx);
    if(c == '\x04')
    {
        CHANGE_COLOR_BLACK();
        cout << "Ctrl-D received" << endl;
        f_stat = 0;
    }
    else
    {
        CHANGE_COLOR_GREEN();
        MOVE_CURSOR(1,1);
        temp = c;
        cout << temp;
        PUT_CELL(5,5);
        PUT_CELL(5,6);
        PUT_CELL(7,6);
        cout << flush;
    }
    pthread_mutex_unlock(&mtx);

    return f_stat;
}

// ================================================================================= //
// update
//
// This method takes one step to update the game status controlling mutex.
// 
// ================================================================================= //
void GAME::update()
{
    while(isRunning())
    {
        pthread_mutex_lock(&mtx);
        temp = (temp + 1 - 'a')%26+'a';
        MOVE_CURSOR(1,1);
        cout << temp;
        pthread_mutex_unlock(&mtx);
        usleep(1000);
    }
}

// ================================================================================= //
// isRunning
//
// return f_stat to tell if the game is running or not
// 
// ================================================================================= //
int GAME::isRunning()
{
    return f_stat;
}

// ================================================================================= //
// run
//
// This method is supposed to run on another thread.
// It updates the game status periodically.
// ================================================================================= //
void *GAME::run(void* p_gm)
{
    GAME* gm = (GAME*)p_gm;

    gm->update();

    pthread_exit(NULL);
}

