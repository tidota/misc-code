#ifndef _FORMAT_MACRO_HPP
#define _FORMAT_MACRO_HPP

#include <iostream>
#include <sstream>
using namespace std;

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
do{\
    ostringstream sy; sy << (y);\
    ostringstream sx; sx << (x);\
    cout << ESC << "[" << sy.str() << ";" << sx.str() << "H";\
}while(0)

// === clear the screen === //
#define CLEAR_SCREEN() \
do{ cout << ESC << START_LOC << ESC << CLEAR; }while(0)

#define CHANGE_COLOR_BLACK() \
do{ cout << ESC << BLACK; }while(0)

#define CHANGE_COLOR_GREEN() \
do{ cout << ESC << GREEN; }while(0)

// === draw a horizontal line === //
#define DRAW_HLINE(y,x1,x2) \
do{\
    MOVE_CURSOR(x1,y);\
    cout << string((x2)-(x1)+1,'-');\
}while(0)
// === draw a vertical line === //
#define DRAW_VLINE(x,y1,y2) \
do{\
    for(int i = 0; i <= (y2) - (y1); i++)\
    {\
        MOVE_CURSOR(x,(y1)+i);\
        cout << '|';\
    }\
}while(0)

// === draw a rectangle === //
#define DRAW_RECT(x1,y1,x2,y2) \
do{\
    DRAW_HLINE(y1,x1,x2);\
    DRAW_HLINE(y2,x1,x2);\
    DRAW_VLINE(x1,y1,y2);\
    DRAW_VLINE(x2,y1,y2);\
    MOVE_CURSOR(x1,y1); cout << '+';\
    MOVE_CURSOR(x1,y2); cout << '+';\
    MOVE_CURSOR(x2,y1); cout << '+';\
    MOVE_CURSOR(x2,y2); cout << '+';\
}while(0)
    
// ====================================================================== //
// Macros for cells
// ====================================================================== //
// width of a cell
#define WCELL 2
// height of a cell
#define HCELL 1
// drawing lines based on cells
#define DRAW_HLINE_CELL(cy,cx1,cx2) DRAW_HLINE(HCELL*(cy),WCELL*(cx1),WCELL*(cx2+1)-1)
#define DRAW_VLINE_CELL(cx,cy1,cy2) DRAW_HLINE(HCELL*(cx),WCELL*(cy1),WCELL*(cy2+1)-1)
#define DRAW_RECT_CELL(cx1,cy1,cx2,cy2)\
   DRAW_RECT(WCELL*(cx1),HCELL*(cy1),WCELL*(cx2+1)-1,HCELL*(cy2+1)-1)


#endif //_FORMAT_MACRO_HPP
