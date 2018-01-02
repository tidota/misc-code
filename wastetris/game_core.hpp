#ifndef _GAME_CORE_HPP
#define _GAME_CORE_HPP

#include <thread>
#include <mutex>

class GAME
{
private:
    GAME();
    ~GAME();

    int screen_width;
    int screen_height;

    int bin_start_x;
    int bin_start_y;

    int nrow;
    int ncol;

    int next_start_x;
    int next_start_y;

    void draw_background();

    int f_stat;

    void update();
    std::thread t_update;
    std::mutex mtx;

    static GAME* game;
public:
    static GAME* init_game();
    static void kill_game();

    int play_game(char c);
    int isRunning();
};

#endif //_GAME_CORE_HPP
