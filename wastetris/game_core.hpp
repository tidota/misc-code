#ifndef _GAME_CORE_HPP
#define _GAME_CORE_HPP

#include <pthread.h>

class GAME
{
private:
    static GAME* game;
    GAME();
    ~GAME();
    static void *update(void *);
    pthread_t update_thread;
public:
    static void init_game();
    static int play_game(char c);
};

#endif //_GAME_CORE_HPP
