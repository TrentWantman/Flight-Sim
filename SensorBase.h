#ifndef SENSORBASE_H
#define SENSORBASE_H

#include "Bus.h"
#include <thread>
#include <chrono>
#include <string>
#include <cstdlib>
#include <cmath>

class SensorBase{
    protected:
        std::string name;
        Bus& bus;

        float noise(float magnitude) {
            return ((rand() % 100) - 50) * magnitude;
        }

        float vote(float s1, float s2, float s3) {
            bool oneTwo = std::abs(s1 - s2) <= 0.5f;
            bool oneThree = std::abs(s1 - s3) <= 0.5f;
            bool twoThree = std::abs(s2 - s3) <= 0.5f;
            int agree = (int)oneTwo + (int)oneThree + (int)twoThree;
            if (agree > 1) return (s1 + s2 + s3) / 3.0f;
            if (oneTwo) return (s1 + s2) / 2.0f;
            if (oneThree) return (s1 + s3) / 2.0f;
            if (twoThree) return (s2 + s3) / 2.0f;
            return s1;
        }

        float voted(float trueValue, float magnitude = 0.01f) {
            return vote(trueValue + noise(magnitude), trueValue + noise(magnitude), trueValue + noise(magnitude));
        }

    public:
        bool stopped = false;
        
        SensorBase(const std::string& name_, Bus& bus_) : name(name_), bus(bus_){}
        virtual ~SensorBase() = default;

        virtual void run() = 0;
};

#endif
