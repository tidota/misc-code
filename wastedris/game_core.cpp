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

    nrow = NROW_BIN;
    ncol = NCOL_BIN;

    bin_start_x = START_CELL_X;
    bin_start_y = START_CELL_Y;

    next_start_x = START_CELL_NBOX_X;
    next_start_y = START_CELL_NBOX_Y;

    screen_width = next_start_x + 1 + 4 * WCELL + 1 + 3;
    screen_height = bin_start_y + nrow * HCELL + 3;

    srand(time(NULL));
    bin = NULL;

    init_stat();
    rand_next();
    copy_pieces();
    rand_next();
    
    draw_background();
    draw_cells();
    FLUSH();

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
    if(bin != NULL)
    {
        for(int i = 0; i < nrow; i++)
            delete[] bin[i];
        delete[] bin;
    }

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
// init_stat
//
// initializes all internal parameters.
// ================================================================================= //
void GAME::init_stat()
{
    if(bin != NULL)
    {
        for(int i = 0; i < nrow; i++)
            delete[] bin[i];
        delete[] bin;
    }

    bin = new int*[nrow];
    for(int i = 0; i < nrow; i++)
    {
        bin[i] = new int[ncol];
        for(int j =0; j < ncol; j++)
        {
            bin[i][j] = 0;
        }
    }
    for(int i = 0; i < NROW_PIECE; i++)
    {
        for(int j = 0; j < NCOL_PIECE; j++)
        {
            cur_piece[i][j] = 0;
            next_piece[i][j] = 0;
        }
    }

    f_stat = 1;
}

// ================================================================================= //
// rand_next()
//
// It generates a randomized piece in the next box.
// It first generates a square shape. Then, it randomly changes the shape.
// After all, each shape appears at a chance of 1/7.
// Finally, it turns the generated piece randomly.
//
// *This method assumes the size of the piece is 4x4.
// 
// ================================================================================= //
void GAME::rand_next()
{
    next_piece[0][0] = 0; next_piece[0][1] = 0; next_piece[0][2] = 0; next_piece[0][3] = 0;
    next_piece[1][0] = 0; next_piece[1][1] = 1; next_piece[1][2] = 1; next_piece[1][3] = 0;
    next_piece[2][0] = 0; next_piece[2][1] = 1; next_piece[2][2] = 1; next_piece[2][3] = 0;
    next_piece[3][0] = 0; next_piece[3][1] = 0; next_piece[3][2] = 0; next_piece[3][3] = 0;

    double p_val = (double)rand()/RAND_MAX;
    if(p_val < 3.0/7.0)
    {
        next_piece[1][1] = 0;
        next_piece[0][2] = 1;
        p_val = (double)rand()/RAND_MAX;
        if(p_val < 1.0/6.0)
        {
            next_piece[2][1] = 0;
            next_piece[3][2] = 1;
        }
        else if(p_val < 2.0/6.0)
        {
            next_piece[1][1] = 1;
            next_piece[2][1] = 0;
        }
        else if(p_val < 2.0/3.0)
        {
            next_piece[0][2] = 0;
            next_piece[1][3] = 1;
        }
    }
    else if(p_val < 6.0/7.0)
    {
        next_piece[2][1] = 0;
        next_piece[3][2] = 1;
        p_val = (double)rand()/RAND_MAX;
        if(p_val < 1.0/6.0)
        {
            next_piece[1][1] = 0;
            next_piece[0][2] = 1;
        }
        else if(p_val < 2.0/6.0)
        {
            next_piece[1][1] = 0;
            next_piece[2][1] = 1;
        }
        else if(p_val < 2.0/3.0)
        {
            next_piece[3][2] = 0;
            next_piece[2][3] = 1;
        }
    }

    p_val = (double)rand()/RAND_MAX;
    if(p_val < 1.0/4.0)
        rotL_piece(next_piece);
    else if(p_val < 2.0/4.0)
        rotR_piece(next_piece);
    else if(p_val < 3.0/4.0)
    {
        rotL_piece(next_piece);
        rotL_piece(next_piece);
    }
}

// ================================================================================= //
// copy_pieces()
//
// It copies a piece in the next box to the current box.
// Then, it initializes the curr piece's location.
// ================================================================================= //
void GAME::copy_pieces()
{
    for(int i = 0; i < NROW_PIECE; i++)
    {
        for(int j = 0; j < NCOL_PIECE; j++)
        {
            cur_piece[i][j] = next_piece[i][j];
        }
    }
    cur_p_x = (ncol - NCOL_PIECE)/2;
    cur_p_y = -1*NROW_PIECE;
}

// ================================================================================= //
// rotR_piece
//
// This rotates a piece clockwise.
// ================================================================================= //
void GAME::rotR_piece(int (*piece)[NCOL_PIECE])
{
    int buff[NROW_PIECE][NCOL_PIECE];

    for(int i = 0; i < NROW_PIECE; i++)
        for(int j = 0; j < NCOL_PIECE; j++)
            buff[j][NCOL_PIECE-1-i] = piece[i][j];
    for(int i = 0; i < NROW_PIECE; i++)
        for(int j = 0; j < NCOL_PIECE; j++)
            piece[i][j] = buff[i][j];
}

// ================================================================================= //
// rotL_piece
//
// This rotates a piece anti-clockwise.
// ================================================================================= //
void GAME::rotL_piece(int (*piece)[NCOL_PIECE])
{
    int buff[NROW_PIECE][NCOL_PIECE];

    for(int i = 0; i < NROW_PIECE; i++)
        for(int j = 0; j < NCOL_PIECE; j++)
            buff[NROW_PIECE-1-j][i] = piece[i][j];
    for(int i = 0; i < NROW_PIECE; i++)
        for(int j = 0; j < NCOL_PIECE; j++)
            piece[i][j] = buff[i][j];
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
    DRAW_RECT(next_start_x-1, next_start_y-1, next_start_x+WCELL*4, next_start_y+HCELL*4);
    MOVE_CURSOR(next_start_x+WCELL*2-2, next_start_y+HCELL*4+1+1);
    cout << "NEXT";
    CHANGE_COLOR_DEF();
    FLUSH();
}

// ================================================================================= //
// draw_cells
//
// It draws all cells in the bin and the next box.
// ================================================================================= //
void GAME::draw_cells()
{
    CHANGE_COLOR_MAGENTA();
    for(int i = 0; i < ncol; i++)
    {
        for(int j = 0; j < nrow; j++)
        {
            if(bin[j][i]==0)
            {
                DEL_CELL(i,j);
            }
            else
            {
                PUT_CELL(i,j);
            }
        }
    }
    CHANGE_COLOR_BYELLOW();
    for(int i = 0; i < NCOL_PIECE; i++)
    {
        for(int j = 0; j < NROW_PIECE; j++)
        {
            if(cur_piece[j][i]>0)
            {
                if(0<=cur_p_x+i&&cur_p_x+i<ncol&&0<=cur_p_y+j&&cur_p_y+j<nrow)
                    PUT_CELL(cur_p_x+i,cur_p_y+j);
            }
        }
    }
    CHANGE_COLOR_YELLOW();
    for(int i = 0; i < NCOL_PIECE; i++)
    {
        for(int j = 0; j < NROW_PIECE; j++)
        {
            if(next_piece[j][i]==0)
            {
                DEL_CELL_NBOX(i,j);
            }
            else
            {
                PUT_CELL_NBOX(i,j);
            }
        }
    }
    CHANGE_COLOR_DEF();
    FLUSH();
}

// ================================================================================= //
// update
//
// This method takes one step to update the game status controlling mutex.
//
// It checks if the current piece can fall by one cell.
// If so, just let it go.
// Otherwise, it checks if the current piece is off the area of the bin.
// If so, the game is over.
// Otherwise, it places the current piece in the bin, and evaluates the game.
// 
// ================================================================================= //
void GAME::update()
{
    while(isRunning())
    {
        mtx.lock();
        
        if(isMovable(0,1))
        {
            cur_p_y++; 
            draw_cells();
        }
        else if(cur_p_y < 0)
        {
            f_stat = 0;
        }
        else
        {
            placePiece();
            copy_pieces();
            rand_next();
        }    
        mtx.unlock();

        usleep(500000);
    }
}

// ================================================================================= //
// isMovable
//
// it returns true if the current piece can move by the specified values.
// otherwise, it returns false.
//
// the piece cannot move into the existing cells, the walls, and the floor.
// the method does not care about the ceiling.
// ================================================================================= //
bool GAME::isMovable(int dx, int dy)
{
    int proposed_x = cur_p_x + dx;
    int proposed_y = cur_p_y + dy;
    bool f_movable = true;

    for(int i = 0; i < NROW_PIECE; i++)
    {
        for(int j = 0; j < NCOL_PIECE; j++)
        {
            int x = proposed_x+j;
            int y = proposed_y+i;
            if(cur_piece[i][j]!=0)
            {
                if(0<=x && x<ncol && 0<=y && y<nrow && bin[y][x]!=0)
                    f_movable = false;
                if(x < 0 || ncol <= x || nrow <= y)
                    f_movable = false;
            }
        }
    }

    return f_movable;
}

// ================================================================================= //
// placePiece
//
// It places the current piece into the bin.
// ================================================================================= //
void GAME::placePiece()
{
    for(int i = 0; i < NROW_PIECE; i++)
    {
        for(int j = 0; j < NCOL_PIECE; j++)
        {
            if(cur_piece[i][j] != 0)
                bin[cur_p_y+i][cur_p_x+j] = cur_piece[i][j];
        }
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

