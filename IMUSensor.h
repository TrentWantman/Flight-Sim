#ifndef IMUSENSOR_H
#define IMUSENSOR_H

#include "SensorBase.h"
#include "Rocket.h"
#include "Bus.h"
#include <thread>
#include <chrono>
#include <string>
#include <cstdlib>
#include <cmath>

class IMUSensor : public SensorBase {
private:
    const Rocket& rocket;

public:
    IMUSensor(const std::string& name_, Bus& bus_, const Rocket& rocket_)
        : SensorBase(name_, bus_), rocket(rocket_) {}

    void run() override {
        auto cycleTime = std::chrono::milliseconds(1); // 1000Hz
        while (!stopped) {
            auto start = std::chrono::steady_clock::now();

            Vec3 accel = rocket.GetAcceleration();
            Vec3 fwd = rocket.GetForwardDirection();

            bus.accXChannel.write(static_cast<float>(voted(accel.getX(), 0.001))); bus.accXChannel.swapBuffers();
            bus.accYChannel.write(static_cast<float>(voted(accel.getY(), 0.001))); bus.accYChannel.swapBuffers();
            bus.accZChannel.write(static_cast<float>(voted(accel.getZ(), 0.001))); bus.accZChannel.swapBuffers();
            bus.orientXChannel.write(fwd.getX()); bus.orientXChannel.swapBuffers();
            bus.orientZChannel.write(fwd.getZ()); bus.orientZChannel.swapBuffers();

            std::this_thread::sleep_until(start + cycleTime);
        }
    }
};

#endif