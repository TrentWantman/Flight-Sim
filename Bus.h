#ifndef BUS_H
#define BUS_H

#include "DoubleCircularBuffer.h"

class Bus {
public:
    DoubleCircularBuffer altitudeChannel;
    DoubleCircularBuffer velocityChannel;
    DoubleCircularBuffer massChannel;
    DoubleCircularBuffer throttleChannel;
    DoubleCircularBuffer rotationChannel;
    DoubleCircularBuffer attitudeChannel;
};

#endif