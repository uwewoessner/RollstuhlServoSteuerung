#ifndef _INC_MY_TIME_H
#define _INC_MY_TIME_H
#include <math.h>
extern long myTimeCT;
inline long currentTime()
{
     return myTimeCT;
}
inline void incTime()
{
    myTimeCT+=1000;
}
#endif
