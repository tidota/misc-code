#ifndef _GAME_CORE_HPP
#define _GAME_CORE_HPP

class GAME
{
private:
    static GAME* game;
    GAME();
    ~GAME();
public:
    static void init_game();
    static int play_game(char c);
};

#endif //_GAME_CORE_HPP
