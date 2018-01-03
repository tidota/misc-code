#ifndef _GAME_CORE_HPP
#define _GAME_CORE_HPP

#include <thread>
#include <mutex>

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

    // the status of the game
    // 0: stopped
    // 1: running
    // others: some error or anything else
    int f_stat;

    // thread and mutex
    std::thread t_update;
    std::mutex mtx;

    // pointer to the object (since this class is supposed to be singleton)
    static GAME* game;

    GAME();
    ~GAME();
    void draw_background();
    void update();

public:
    static GAME* init_game();
    static void kill_game();

    int play_game(char c);
    int isRunning();
};

#endif //_GAME_CORE_HPP
