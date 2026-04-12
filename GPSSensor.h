#ifndef GPSSENSOR_H
#define GPSSENSOR_H

#include "Rocket.h"
#include "Bus.h"
#include <thread>
#include <chrono>
#include <string>
#include <cstdlib>
#include <cmath>

class GPSSensor {
private:
    const Rocket& rocket;
    Bus& bus;
    std::string name;

    float noise() {
        return ((rand() % 100) - 50) * 0.01f;
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

    GPSSensor(const std::string& name_, Bus& bus_, const Rocket& rocket_) : name(name_), bus(bus_), rocket(rocket_) {}

    void run() {
        auto cycleTime = std::chrono::milliseconds(100); // 10Hz
        while (!stopped) {
            auto start = std::chrono::steady_clock::now();

            Vec3 pos = rocket.GetPosition();
            Vec3 vel = rocket.GetVelocity();

            bus.posXChannel.write(voted(pos.getX())); bus.posXChannel.swapBuffers();
            bus.posYChannel.write(voted(pos.getY())); bus.posYChannel.swapBuffers();
            bus.posZChannel.write(voted(pos.getZ())); bus.posZChannel.swapBuffers();
            bus.velXChannel.write(voted(vel.getX())); bus.velXChannel.swapBuffers();
            bus.velYChannel.write(voted(vel.getY())); bus.velYChannel.swapBuffers();
            bus.velZChannel.write(voted(vel.getZ())); bus.velZChannel.swapBuffers();

            std::this_thread::sleep_until(start + cycleTime);
        }
    }
};

#endif