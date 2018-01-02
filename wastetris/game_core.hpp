#ifndef _GAME_CORE_HPP
#define _GAME_CORE_HPP

#include <thread>
#include <mutex>

class GAME
{
private:
    GAME();
    ~GAME();

    int f_stat;

    void update();
    std::thread t_update;
    std::mutex mtx;

    static GAME* game;

    char temp;
public:
    static GAME* init_game();
    static void kill_game();

    int play_game(char c);
    int isRunning();
};

#endif //_GAME_CORE_HPP
