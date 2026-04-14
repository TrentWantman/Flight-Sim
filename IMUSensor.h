#ifndef IMUSENSOR_H
#define IMUSENSOR_H

#include "Rocket.h"
#include "Bus.h"
#include <thread>
#include <chrono>
#include <string>
#include <cstdlib>
#include <cmath>

class IMUSensor {
private:
    const Rocket& rocket;
    Bus& bus;
    std::string name;

    float noise() {
        return ((rand() % 100) - 50) * 0.001f;
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

    float voted(float trueValue) {
        return vote(trueValue + noise(), trueValue + noise(), trueValue + noise());
    }

public:
    bool stopped = false;

    IMUSensor(const std::string& name_, Bus& bus_, const Rocket& rocket_)
        : name(name_), bus(bus_), rocket(rocket_) {}

    void run() {
        auto cycleTime = std::chrono::milliseconds(1); // 1000Hz
        while (!stopped) {
            auto start = std::chrono::steady_clock::now();

            Vec3 accel = rocket.GetAcceleration();
            Vec3 fwd = rocket.GetForwardDirection();

            bus.accXChannel.write(voted(accel.getX())); bus.accXChannel.swapBuffers();
            bus.accYChannel.write(voted(accel.getY())); bus.accYChannel.swapBuffers();
            bus.accZChannel.write(voted(accel.getZ())); bus.accZChannel.swapBuffers();
            bus.orientXChannel.write(fwd.getX()); bus.orientXChannel.swapBuffers();
            bus.orientZChannel.write(fwd.getZ()); bus.orientZChannel.swapBuffers();

            std::this_thread::sleep_until(start + cycleTime);
        }
    }
};

#endif