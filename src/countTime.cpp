
#include "countTime.h"


void countDown(int& t1, bool& notified) {
    if(t1){
        t1--;
        notified = false;
    } else {
        if(!notified){
            Serial.println("Time's up for listening in!");
            notified = true;
        }
    }
 }

