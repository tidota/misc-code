#ifndef _FORMAT_MACRO_HPP
#define _FORMAT_MACRO_HPP

// ====================================================================== //
// definitions for escape sequences to provide formats
// ====================================================================== //

// starting character for escape sequences
#define ESC '\x001B'

// === positions === //
// the initial position
#define START_LOC "[0;0H"

// === colors === //
// color: black
#define BLACK "[0m"
// color: green
#define GREEN "[32m"

// === misc === //
// indentation
#define IND "   "
// clear the screen
#define CLEAR "[2J"

// === macro func === //
#define CLEAR_SCREEN() \
do{ cout << ESC << CLEAR; }while(0)

#define CHANGE_COLOR_BLACK() \
do{ cout << ESC << BLACK; }while(0)

#define CHANGE_COLOR_GREEN() \
do{ cout << ESC << GREEN; }while(0)


#endif //_FORMAT_MACRO_HPP
