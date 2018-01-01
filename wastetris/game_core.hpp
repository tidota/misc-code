#ifndef _GAME_CORE_HPP
#define _GAME_CORE_HPP

#include <pthread.h>

class GAME
{
private:
    GAME();
    ~GAME();
    void update();

    int f_stat;

    pthread_t update_thread;
    pthread_mutex_t mtx;

    static GAME* game;
    static void *run(void*);
public:
    static GAME* init_game();
    static void kill_game();

    int play_game(char c);
    int isRunning();
};

#endif //_GAME_CORE_HPP
