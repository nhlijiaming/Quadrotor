/*
 * board.c
 *
 *  Created on: 2014Äê8ÔÂ7ÈÕ
 *      Author: Ljm
 */

#include "board.h"

int32_t constrain(int32_t amt, int32_t low, int32_t high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

float constrainf(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

int32_t applyDeadband(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

float applyDeadbandf(float value, float deadband)
{
    if (abs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}
