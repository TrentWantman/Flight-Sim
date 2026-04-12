#ifndef IMUSENSOR_H
#define IMUSENSOR_H

#include "Rocket.h"
#include "Bus.h"
#include <thread>
#include <chrono>
#include <string>
#include <cstdlib>

class IMUSensor {
private:
    const Rocket& rocket;
    Bus& bus;
    std::string name;
    
    float noise() {
        return ((rand() % 100) - 50) * 0.001f;
    }
    
public:
    bool stopped = false;
    
    IMUSensor(const std::string& name_, Bus& bus_, const Rocket& rocket_) : name(name_), bus(bus_), rocket(rocket_) {}
    
    void run() {
        auto cycleTime = std::chrono::milliseconds(1); // 1000Hz
        while (!stopped) {
            auto start = std::chrono::steady_clock::now();
            
            Vec3 accel = rocket.GetAcceleration();
            
            bus.accXChannel.write(accel.getX() + noise()); bus.accXChannel.swapBuffers();
            bus.accYChannel.write(accel.getY() + noise()); bus.accYChannel.swapBuffers();
            bus.accZChannel.write(accel.getZ() + noise()); bus.accZChannel.swapBuffers();
            
            std::this_thread::sleep_until(start + cycleTime);
        }
    }
};

#endif