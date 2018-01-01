#ifndef _FORMAT_MACRO_HPP
#define _FORMAT_MACRO_HPP

// ====================================================================== //
// basic definitions for escape sequences to provide formats
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

// ====================================================================== //
// Macros
// ====================================================================== //
// === move the cursor to the given position === //
// note: the position is specified by 1-index (it starts with 1, not 0)
#define MOVE_CURSOR(x,y) \
do{ cout << ESC << "[" << (y) << ";" << (x) << "H"; }while(0)

// === clear the screen === //
#define CLEAR_SCREEN() \
do{ cout << ESC << START_LOC << ESC << CLEAR; }while(0)

#define CHANGE_COLOR_BLACK() \
do{ cout << ESC << BLACK; }while(0)

#define CHANGE_COLOR_GREEN() \
do{ cout << ESC << GREEN; }while(0)

#endif //_FORMAT_MACRO_HPP
