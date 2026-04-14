#ifndef FUELSENSOR_H
#define FUELSENSOR_H

#include "SensorBase.h"
#include "Rocket.h"
#include "Bus.h"
#include <thread>
#include <chrono>
#include <string>
#include <cstdlib>
#include <cmath>

class FuelSensor : public SensorBase{
private:
    const Rocket& rocket;

public:
    FuelSensor(const std::string& name_, Bus& bus_, const Rocket& rocket_)
        : SensorBase(name_, bus_), rocket(rocket_) {}

    void run() override {
        auto cycleTime = std::chrono::milliseconds(1000); // 1Hz
        while (!stopped) {
            auto start = std::chrono::steady_clock::now();

            bus.massChannel.write(voted(rocket.GetMass(), 0.01f));
            bus.massChannel.swapBuffers();

            std::this_thread::sleep_until(start + cycleTime);
        }
    }
};

#endif