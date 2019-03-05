// noncanonical.cpp
//
// This is a template to offer a non-canonical mode,
// in which the program immediately receives any character from the user

// reference
// 17.7 Noncanonical mode example
// https://www.gnu.org/software/libc/manual/html_node/Noncanon-Example.html

#include <iostream>
#include <unistd.h>
#include <termios.h>

#include <cstdlib> // for atexit
//#include <stdio.h>
//#include <stdlib.h>

using namespace std;

/* Use this variable to remember original terminal attributes. */
struct termios saved_attributes;

void reset_input_mode (void)
{
    tcsetattr (STDIN_FILENO, TCSANOW, &saved_attributes);
}

int set_input_mode (void)
{
    struct termios tattr;
    char *name;
    int f_fail = 0;

    /* Make sure stdin is a terminal. */
    if (!isatty (STDIN_FILENO))
    {
        cout << "Not a terminal." << endl;
        f_fail = -1;
    }

    if(f_fail == 0)
    {
        /* Save the terminal attributes so we can restore them later. */
        tcgetattr (STDIN_FILENO, &saved_attributes);
        atexit (reset_input_mode);

        /* Set the funny terminal modes. */
        tcgetattr (STDIN_FILENO, &tattr);
        tattr.c_lflag &= ~(ICANON|ECHO); /* Clear ICANON and ECHO. */
        tattr.c_cc[VMIN] = 1;
        tattr.c_cc[VTIME] = 0;
        tcsetattr (STDIN_FILENO, TCSAFLUSH, &tattr);
    }

    return f_fail;
}

int main (void)
{
    char c;
    int f_fail;

    f_fail = set_input_mode ();

    while (f_fail == 0)
    {
        read (STDIN_FILENO, &c, 1);
        if (c == '\004')                    /* C-d */
            break;
        else
            cout << c << flush;
    }

    return f_fail;
}

