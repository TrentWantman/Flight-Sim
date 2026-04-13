#ifndef BUS_H
#define BUS_H

#include "DoubleCircularBuffer.h"

class Bus {
public:
    DoubleCircularBuffer massChannel;
    DoubleCircularBuffer throttleChannel;
    DoubleCircularBuffer rotationChannel;
    DoubleCircularBuffer attitudeChannel;
    DoubleCircularBuffer posXChannel;
    DoubleCircularBuffer posYChannel;
    DoubleCircularBuffer posZChannel;
    DoubleCircularBuffer velXChannel;
    DoubleCircularBuffer velYChannel;
    DoubleCircularBuffer velZChannel;
    DoubleCircularBuffer accXChannel;
    DoubleCircularBuffer accYChannel;
    DoubleCircularBuffer accZChannel;
};

#endif