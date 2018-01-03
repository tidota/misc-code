#ifndef _GAME_CORE_HPP
#define _GAME_CORE_HPP

#include <thread>
#include <mutex>
#include "format_macro.hpp"

class GAME
{
private:
    // the size of the entire screen
    int screen_width;
    int screen_height;

    // the top-left corner of the box where cells appear
    int bin_start_x;
    int bin_start_y;

    // # of rows and # of columns of the box in terms of cells 
    int nrow;
    int ncol;

    // the top-left corner of the next box
    int next_start_x;
    int next_start_y;

    // bin: 2D table holding stata of the cells
    int **bin;
    // cur_piece: current piece and its location
    int cur_piece[NROW_PIECE][NCOL_PIECE];
    int cur_p_x, cur_p_y;
    // next_piece: the piece which will be released
    int next_piece[NROW_PIECE][NCOL_PIECE];

    // the status of the game
    // 0: stopped
    // 1: running
    // others: some error or anything else
    int f_stat;

    // thread and mutex
    std::thread t_update;
    std::mutex mtx;
    // step index
    // 0: update the game
    int n_step;
    int i_step;

    // pointer to the object (since this class is supposed to be singleton)
    static GAME* game;

    GAME();
    ~GAME();
    void init_stat();
    void abort();
    void play_endmovie();
    void rand_next();
    void copy_pieces();
    void rotR_piece(int (*piece)[NCOL_PIECE]);
    void rotL_piece(int (*piece)[NCOL_PIECE]);
    void draw_background();
    void draw_cells();
    void update();
    void eval_and_clean();
    bool isMovable(int dx, int dy);
    bool isRotatable(bool clockwise);
    void placePiece();

public:
    static GAME* init_game();
    static void kill_game();

    int play_game(char c);
    int isRunning();
};

#endif //_GAME_CORE_HPP
