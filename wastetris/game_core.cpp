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

#include <thread>
#include <mutex>

#include <unistd.h> // usleep
#include <stdlib.h> // random number gen
#include <time.h> // random number gen

using namespace std;

// ================================================================================= //
// the pointer to the object is initialized with NULL.
// so the program can tell if there is an existing one.
// ================================================================================= //
GAME* GAME::game = NULL;

// ================================================================================= //
// Constructor
//
// It cleans up the screen for setup.
// Then, it prepares a mutex and a thread to independently run the update function.
// Finally, it initalizes the random number generator.
//
// ================================================================================= //
GAME::GAME()
{
    CLEAR_SCREEN();
    CURSOR_OFF();
    f_stat = 1;
    t_update = thread(&GAME::update,this);

    srand(time(NULL));

    temp = 'a';
}

// ================================================================================= //
// Destructor
//
// It tells the thread to stop, and waits for it.
//
// Finally, it also performs the ending scene which "burns up" the screen.
//
// ================================================================================= //
GAME::~GAME()
{
    mtx.lock();
    f_stat = 0;
    mtx.unlock();
    t_update.join();

    int width = 100;
    int height = 50;

    CHANGE_COLOR_BRED();
    char symbols[5] = {' ', '.', ';', '*', '#'};
    int thresholds[5] = {0, 90, 30, 5, 0};
    for(int i = height; i >= 1; i--)
    {
        for(int isym = 0; isym < 5; isym++)
        {
            if(i - isym >= 1)
            {
                DRAW_HLINE_C(i - isym, 1, 1, string(width, symbols[isym]));
                for(int icol = 1; icol <= width && thresholds[isym] != 0; icol++)
                {
                    int rand_v = rand()%100;
                    if(rand_v < thresholds[isym])
                    {
                        MOVE_CURSOR(icol,i-isym);
                        cout << ' ';
                    }
                }
            }
        }
        FLUSH();
        usleep(60000);
    }
    CHANGE_COLOR_DEF();

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
    mtx.lock();

    if(c == '\x04')
    {
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
        CHANGE_COLOR_DEF();
    }

    mtx.unlock();

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
        mtx.lock();

        temp = (temp + 1 - 'a')%26+'a';
        MOVE_CURSOR(1,1);
        cout << temp;

        mtx.unlock();

        usleep(1000);
    }
}

// ================================================================================= //
// isRunning
//
// it tells if the game is running or not
// f_stat == 1 indicates it is running. Otherwise, it stops for some reason.
// 
// ================================================================================= //
int GAME::isRunning()
{
    return (f_stat == 1)? 1: 0;
}

