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
// color: default
#define CLR_DEF "[0m"
// color: black
#define BLACK "[30m"
#define BBLACK "[90m"
// color: green
#define GREEN "[32m"
#define BGREEN "[92m"
// color: red
#define RED "[31m"
#define BRED "[91m"
// color: yellow
#define YELLOW "[33m"
#define BYELLOW "[93m"
// color: blue
#define BLUE "[34m"
#define BBLUE "[94m"
// color: bagenta
#define MAGENTA "[35m"
#define BMAGENTA "[95m"
// color: cyan
#define CYAN "[36m"
#define BCYAN "[96m"
// color: white
#define WHITE "[37m"
#define BWHITE "[97m"

// === misc === //
// indentation
#define IND "   "
// clear the screen
#define CLEAR "[2J"

// ====================================================================== //
// Macros
// ====================================================================== //
// === cursor on === //
#define CURSOR_ON()  do{ cout << ESC << "[?25h"; }while(0)
// === cursor off === //
#define CURSOR_OFF() do{ cout << ESC << "[?25l"; }while(0)

// === move the cursor to the given position === //
// note: the position is specified by 1-index (it starts with 1, not 0)
#define MOVE_CURSOR(x,y) \
do{\
    ostringstream sy; sy << (y);\
    ostringstream sx; sx << (x);\
    cout << ESC << "[" << sy.str() << ";" << sx.str() << "H";\
}while(0)

// === flush the output buffer === //
#define FLUSH() do { cout << flush; }while(0)

// === clear the screen === //
#define CLEAR_SCREEN() \
do{ cout << ESC << START_LOC << ESC << CLEAR; }while(0)

// === change the color of characters === //
#define CHANGE_COLOR_DEF() do{ cout << ESC << CLR_DEF; }while(0)
#define CHANGE_COLOR_BLACK() do{ cout << ESC << BLACK; }while(0)
#define CHANGE_COLOR_BBLACK() do{ cout << ESC << BBLACK; }while(0)
#define CHANGE_COLOR_WHITE() do{ cout << ESC << WHITE; }while(0)
#define CHANGE_COLOR_BWHITE() do{ cout << ESC << BWHITE; }while(0)
#define CHANGE_COLOR_RED() do{ cout << ESC << RED; }while(0)
#define CHANGE_COLOR_BRED() do{ cout << ESC << BRED; }while(0)
#define CHANGE_COLOR_GREEN() do{ cout << ESC << GREEN; }while(0)
#define CHANGE_COLOR_BGREEN() do{ cout << ESC << BGREEN; }while(0)
#define CHANGE_COLOR_BLUE() do{ cout << ESC << BLUE; }while(0)
#define CHANGE_COLOR_BBLUE() do{ cout << ESC << BBLUE; }while(0)
#define CHANGE_COLOR_YELLOW() do{ cout << ESC << YELLOW; }while(0)
#define CHANGE_COLOR_BYELLOW() do{ cout << ESC << BYELLOW; }while(0)
#define CHANGE_COLOR_MAGENTA() do{ cout << ESC << MAGENTA; }while(0)
#define CHANGE_COLOR_BMAGENTA() do{ cout << ESC << BMAGENTA; }while(0)
#define CHANGE_COLOR_CYAN() do{ cout << ESC << CYAN; }while(0)
#define CHANGE_COLOR_BCYAN() do{ cout << ESC << BCYAN; }while(0)

// === draw a horizontal line === //
#define DRAW_HLINE_C(y,x1,x2,c) \
do{\
    for(int i_draw_hline = (x1); i_draw_hline <= (x2); i_draw_hline++)\
    {\
        MOVE_CURSOR(i_draw_hline,y);\
        cout << c;\
    }\
}while(0)
#define DRAW_HLINE(y,x1,x2) DRAW_HLINE_C(y,x1,x2,'-')

// === draw a vertical line === //
#define DRAW_VLINE_C(x,y1,y2,c) \
do{\
    for(int i_draw_vline = (y1); i_draw_vline <= (y2); i_draw_vline++)\
    {\
        MOVE_CURSOR(x,i_draw_vline);\
        cout << c;\
    }\
}while(0)
#define DRAW_VLINE(x,y1,y2) DRAW_VLINE_C(x,y1,y2,'|')

// === draw a rectangle === //
#define DRAW_RECT_C(x1,y1,x2,y2,ch,cv,cc) \
do{\
    DRAW_HLINE_C(y1,x1,x2,ch);\
    DRAW_HLINE_C(y2,x1,x2,ch);\
    DRAW_VLINE_C(x1,y1,y2,cv);\
    DRAW_VLINE_C(x2,y1,y2,cv);\
    MOVE_CURSOR(x1,y1); cout << cc;\
    MOVE_CURSOR(x1,y2); cout << cc;\
    MOVE_CURSOR(x2,y1); cout << cc;\
    MOVE_CURSOR(x2,y2); cout << cc;\
}while(0)
#define DRAW_RECT(x1,y1,x2,y2) DRAW_RECT_C(x1,y1,x2,y2,'-','|','+')

// === fill a rectangle === //
#define FILL_RECT_C(x1,y1,x2,y2,c) \
do{\
    for(int j = (y1); j <= (y2); j++)\
        DRAW_HLINE_C(j,x1,x2,c);\
}while(0)
#define FILL_RECT(x1,y1,x2,y2) FILL_RECT_C(x1,y1,x2,y2,"▮")

// ====================================================================== //
// Macros for cells
// ====================================================================== //
// width of a cell
#define WCELL 4
// height of a cell
#define HCELL 3
// drawing lines based on cells
#define DRAW_HLINE_C_CELL(cy,cx1,cx2,c) DRAW_HLINE_C(HCELL*cy,WCELL*cx1,WCELL*(cx2+1)-1,c)
#define DRAW_HLINE_CELL(cy,cx1,cx2) DRAW_HLINE_C_CELL(cy,cx1,cx2,'-')
#define DRAW_VLINE_C_CELL(cx,cy1,cy2,c) DRAW_VLINE_C(WCELL*cx,HCELL*cy1,HCELL*(cy2+1)-1,c)
#define DRAW_VLINE_CELL(cx,cy1,cy2) DRAW_VLINE_C_CELL(cx,cy1,cy2,'|')
#define DRAW_RECT_C_CELL(cx1,cy1,cx2,cy2,ch,cv,cc)\
   DRAW_RECT_C(WCELL*(cx1),HCELL*(cy1),WCELL*(cx2+1)-1,HCELL*(cy2+1)-1,ch,cv,cc)
#define DRAW_RECT_CELL(cx1,cy1,cx2,cy2) DRAW_RECT_C_CELL(cx1,cy1,cx2,cy2,'-','|','+')
#define FILL_RECT_C_CELL(cx1,cy1,cx2,cy2,c)\
   FILL_RECT_C(WCELL*(cx1),HCELL*(cy1),WCELL*(cx2+1)-1,HCELL*(cy2+1)-1,c)
#define FILL_RECT_CELL(cx1,cy1,cx2,cy2) FILL_RECT_C_CELL(cx1,cy1,cx2,cy2,"▮")
#define PUT_C_CELL(cx,cy,ch,cv,cc,cf)\
do{\
   FILL_RECT_C_CELL(cx,cy,cx,cy,cf);\
   DRAW_RECT_C_CELL(cx,cy,cx,cy,ch,cv,cc);\
}while(0)
#define PUT_CELL(cx,cy) PUT_C_CELL(cx,cy,'-','|','+',"▮")


#endif //_FORMAT_MACRO_HPP