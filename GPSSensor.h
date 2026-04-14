#ifndef GPSSENSOR_H
#define GPSSENSOR_H

#include "SensorBase.h"
#include "Rocket.h"
#include "Bus.h"
#include <thread>
#include <chrono>
#include <string>
#include <cstdlib>
#include <cmath>

class GPSSensor : public SensorBase {
private:
    const Rocket& rocket;

public:
    GPSSensor(const std::string& name_, Bus& bus_, const Rocket& rocket_) 
        : SensorBase(name_, bus_), rocket(rocket_) {}

    void run() override {
        auto cycleTime = std::chrono::milliseconds(100); // 10Hz
        while (!stopped) {
            auto start = std::chrono::steady_clock::now();

            Vec3 pos = rocket.GetPosition();
            Vec3 vel = rocket.GetVelocity();

            bus.posXChannel.write(voted(pos.getX(), 0.01f)); bus.posXChannel.swapBuffers();
            bus.posYChannel.write(voted(pos.getY(), 0.01f)); bus.posYChannel.swapBuffers();
            bus.posZChannel.write(voted(pos.getZ(), 0.01f)); bus.posZChannel.swapBuffers();
            bus.velXChannel.write(voted(vel.getX(), 0.01f)); bus.velXChannel.swapBuffers();
            bus.velYChannel.write(voted(vel.getY(), 0.01f)); bus.velYChannel.swapBuffers();
            bus.velZChannel.write(voted(vel.getZ(), 0.01f)); bus.velZChannel.swapBuffers();

            std::this_thread::sleep_until(start + cycleTime);
        }
    }
};

#endif