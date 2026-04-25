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

        double noise(double magnitude) {
            return ((rand() % 100) - 50) * magnitude;
        }

        double vote(double s1, double s2, double s3) {
            bool oneTwo = std::abs(s1 - s2) <= 0.5;
            bool oneThree = std::abs(s1 - s3) <= 0.5;
            bool twoThree = std::abs(s2 - s3) <= 0.5;
            int agree = (int)oneTwo + (int)oneThree + (int)twoThree;
            if (agree > 1) return (s1 + s2 + s3) / 3.0;
            if (oneTwo) return (s1 + s2) / 2.0;
            if (oneThree) return (s1 + s3) / 2.0;
            if (twoThree) return (s2 + s3) / 2.0;
            return s1;
        }

        double voted(double trueValue, double magnitude = 0.01) {
            return vote(trueValue + noise(magnitude), trueValue + noise(magnitude), trueValue + noise(magnitude));
        }

    public:
        bool stopped = false;

        SensorBase(const std::string& name_, Bus& bus_) : name(name_), bus(bus_){}
        virtual ~SensorBase() = default;

        virtual void run() = 0;
};

#endif
