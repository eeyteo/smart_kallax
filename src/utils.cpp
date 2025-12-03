#include "utils.h"


int minmax(int a, int min, int max){
    // This function returns the integer inputted value if between the integer values min and max.
    // it returns the outer bouds if outside
    if (a < min) return min;
    else if (a > max) return max;
    else return a;
}

int inbound(int value, int a, int b){
    // This function checks if the integer value is between a and b (inclusive)
    // it returns the value if value is between a and b, otherwise it returns the closest bound
    int minVal = (a < b) ? a : b;
    int maxVal = (a > b) ? a : b;
    if (value < minVal) return minVal;
    else if (value > maxVal) return maxVal;
    else return value;
}
