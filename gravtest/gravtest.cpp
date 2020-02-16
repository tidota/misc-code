// gravtest.cpp
// 170126
//
// this program simulates the collision time of two objects in space.
// the two objects are affected by the gravitational force of each other.
//
// update 170203: visualization of location was added
// update 170204: showing conditions
// update 170205: improved appearance
// update 170206: improved appearance, macros without parameters
// update 170903: updates on the appearance
// update 170904: macros for setting objects

#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>

using namespace std;

#define ESC '\x001B'
#define START_LOC "[10;0H"
#define IND "   "

#define showCond()\
do{\
  cout << ESC << "[0;0H"\
       << ESC << "[2J"\
       << endl\
       << fixed << setprecision(2)\
       << IND << "initial distance: " << L << " m" << endl\
       << IND << "========== " << str1 << " ===========" << endl\
       << IND << "mass : " << setw(8) << m1 << " kg" << endl\
       << IND << "reach: " << setw(8) << reach1 << " m" << endl\
       << IND << "========== " << str2 << " ===========" << endl\
       << IND << "mass : " << setw(8) << m2 << " kg" << endl\
       << IND << "reach: " << setw(8) << reach2 << " m" << endl\
       << endl;\
}while(0)

#define vis()\
do{\
  int loc1 = (int)(x1/L*num_spaces);\
  int r1 = (int)(reach1/L*num_spaces);\
  int r2 = (int)(reach2/L*num_spaces);\
  int len = (int)((L-x2-reach2-reach1)/L*num_spaces) - loc1;\
  const char r_mes[] = "-reach-";\
  int str_len = strlen(r_mes);\
  char r1_str[50];\
  char r2_str[50];\
  strcpy(r1_str,"");\
  if(r1 >= str_len + 2)\
      strcat(strcat(strcat(r1_str,r_mes),string(r1-str_len-2,'-').c_str()),">|");\
  else if(r1 >= 2)\
      strcat(strcat(r1_str,string(r1-2,'-').c_str()),">|");\
  else if(r1 >= 1)\
      strcat(r1_str,"|");\
  strcpy(r2_str,"");\
  if(r2 >= str_len + 2)\
      strcat(strcat(strcat(r2_str,"|<"),string(r2-str_len-2,'-').c_str()),r_mes);\
  else if(r2 >= 2)\
      strcat(strcat(r2_str,"|<"),string(r2-2,'-').c_str());\
  else if(r2 >= 1)\
      strcat(r2_str,"|");\
  cout << ESC << START_LOC\
       << ESC << "[0J"\
       << string(loc1,' ')\
       << IND\
       << ESC << "[32m" << str1 << ESC << "[0m" << r1_str\
       << (( len > 0 )? string(len,' '): "")\
       << r2_str << ESC << "[32m" << str2 << ESC << "[0m"\
       << endl << endl\
       << IND << "current distance:     " << setw(10) << setprecision(2) << L - x1 - x2 << " m" << endl\
       << IND << "current relative vel: " << setw(10) << setprecision(2) << (v1 + v2)*3600*100 << " cm/h" << endl << endl\
       << IND << "current gravitational force: " << endl\
       << IND << setw(8) << setprecision(2) << F*1000000000000 << " x 10^-12 Newton" << endl << endl\
       << IND << "elapsed time: " << setw(10) << t << " sec" << endl\
       << IND\
       << setw(5) << (int)(t/3600/24) << " days "\
       << setw(2) << setfill('0') << (int)(t/3600)%24 << " hour "\
       << setw(2) << setfill('0') << (int)(t/60)%60 << " min "\
       << setw(2) << setfill('0') << (int)t%60 << " sec" << endl << endl\
       << setfill(' ');\
}while(0)

#define SET1(str,m,r) \
  const char str1[] = str;\
  const double m1 = m;       /*weight (kg)*/\
  const double reach1 = r;   /*the length which the first object reaches*/

#define SET2(str,m,r) \
  const char str2[] = str;\
  const double m2 = m;       /*weight (kg)*/\
  const double reach2 = r;   /*the length which the first object reaches*/

int main(void)
{
  const double L = 20; // initial length (m)
  const double G = 6.674/100000000000.0; // gravitational constant
  const double deltaT = 0.01; // time step (s)
  double t = 0; // current time (s)

  //SET1("Me",60,0.85)

  //SET2("Sandwich",0.1,0.03)
  //SET2("Konoshiki",285,0.935)
  //SET2("Akebono",233,1.015)
  //SET2("Musashimaru",235,0.96)

  SET1("Konoshiki",285,0.935)
  SET2("Akebono",233,1.015)

  double x1 = 0; // length moved (m)
  double x2 = 0; // length moved (m)
  double v1 = 0; // velocity m/s
  double v2 = 0; // velocity m/s
  double a1 = 0; // accelleration m/s^2
  double a2 = 0; // accelleration m/s^2
  double F = 0; // gravitational force
  double dist = L; // current distance
  const double dist2term = reach1 + reach2; // the length to terminate 

  const int num_spaces = 30;
  
  cout << ESC << "[?25l";
  showCond();
  int count = 0;
  while(dist > dist2term)
  {
    F = G*m1*m2/dist/dist;
    x1 = x1 + v1*deltaT;
    x2 = x2 + v2*deltaT;
    v1 = v1 + a1*deltaT;
    v2 = v2 + a2*deltaT;
    a1 = F/m1;
    a2 = F/m2;
    dist = L - x1 - x2;
    t += deltaT;

    count ++;

    if(count % 100000 == 0)
    {
      vis();
    }
  }

  vis();

  cout << ESC << "[?25h";

  return 0;
}

