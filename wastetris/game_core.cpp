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
// It also initializes the parameters for the game.
// Then, it initalizes the random number generator.
// Then, it draws the background including boxes.
// Finally, it prepares a thread to independently run the update function.
//
// ================================================================================= //
GAME::GAME()
{
    CLEAR_SCREEN();
    CURSOR_OFF();

    nrow = 12;
    ncol = 10;

    bin_start_x = START_CELL_X;
    bin_start_y = START_CELL_Y;

    next_start_x = bin_start_x + ncol * WCELL + 3;
    next_start_y = bin_start_y;

    screen_width = next_start_x + 1 + 4 * WCELL + 1 + 3;
    screen_height = bin_start_y + nrow * HCELL + 3;

    f_stat = 1;
    
    draw_background();

    srand(time(NULL));

    t_update = thread(&GAME::update,this);


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

    int width = screen_width;
    int height = screen_height;

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
    }

    mtx.unlock();

    return f_stat;
}

// ================================================================================= //
// draw_background
//
// This method draws the background including the bin and the next box.
//
// ================================================================================= //
void GAME::draw_background()
{
    CLEAR_SCREEN();
    CHANGE_COLOR_CYAN();
    DRAW_RECT(bin_start_x-1, bin_start_y-1, bin_start_x+WCELL*ncol, bin_start_y+HCELL*nrow);
    DRAW_RECT(next_start_x-1, next_start_y-1, next_start_x+1+WCELL*4+1, next_start_y+1+HCELL*4+1);
    MOVE_CURSOR(next_start_x+WCELL*2-2, next_start_y+1+HCELL*4+1+1);
    cout << "NEXT";
    FLUSH();
    CHANGE_COLOR_DEF();
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
        mtx.unlock();

        CHANGE_COLOR_GREEN();
        for(int i = 0; i < ncol; i++)
        {
            for(int j = 0; j < nrow; j++)
            {
                PUT_CELL(i,j);
                FLUSH();
            }
        }
        CHANGE_COLOR_DEF();

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

