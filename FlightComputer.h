#ifndef FLIGHTCOMPUTER_H
#define FLIGHTCOMPUTER_H

#include "LaunchSequence.h"
#include "DoubleCircularBuffer.h"
#include <chrono>
#include <thread>
#include <iostream>

class FlightComputer {
private:
    DoubleCircularBuffer& altBuffer;
    DoubleCircularBuffer& velBuffer;
    DoubleCircularBuffer& commandBuffer;
    LaunchSequence ls;
    static constexpr float dt = 0.1f;

public:
    bool stopped = false;

    FlightComputer(DoubleCircularBuffer& alt, DoubleCircularBuffer& vel, DoubleCircularBuffer& cmd)
        : altBuffer(alt), velBuffer(vel), commandBuffer(cmd) {}

    void setThrottle(float throttle) {
        commandBuffer.write(throttle);
        commandBuffer.swapBuffers();
    }

    float readAltitude() {
        float val;
        if (altBuffer.read(val)) return val;
        return 0.0f;
    }

    float readVelocity() {
        float val;
        if (velBuffer.read(val)) return val;
        return 0.0f;
    }

    void run() {
        auto cycleTime = std::chrono::milliseconds(100);
        auto startAllCycles = std::chrono::steady_clock::now();
        auto start = startAllCycles;

        ls.transition(ls.PRELAUNCH);
        ls.transition(ls.IGNITION);
        ls.transition(ls.LIFTOFF);

        setThrottle(0.95f);

        int cycle = 0;

        while (ls.getState() != "SAFED") {
            cycle++;

            float alt = readAltitude();
            float velocity = readVelocity();

            if (ls.getState() == "LIFTOFF") {
                if (alt >= 1200.0) {
                    ls.transition(ls.MAX_Q);
                    setThrottle(0.004f);
                }
            }
            else if (ls.getState() == "MAX_Q") {
                if (alt >= 1500.0) {
                    ls.transition(ls.MECO);
                    setThrottle(0.0f);
                }
            }
            else if (ls.getState() == "MECO") {
                if (alt <= 1000.0 && velocity < 0) {
                    ls.transition(ls.LANDING);
                    setThrottle(0.29f);
                }
            }
            else if (ls.getState() == "LANDING") {
                if (alt <= 0.0) {
                    ls.transition(ls.SAFED);
                    setThrottle(0.0f);
                }
                else if (velocity < -5.0f) {
                    setThrottle(0.95f);
                }
                else {
                    setThrottle(0.72f);
                }
            }

            auto end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            if (elapsed > cycleTime) {
                std::cout << "OVERRUN cycle " << cycle << ": " << elapsed.count() << "ms" << std::endl;
            }
            else {
                while (std::chrono::steady_clock::now() < start + std::chrono::milliseconds(100)) {}
                auto totalCycle = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
                std::cout << "Cycle " << cycle << ": State=" << ls.getState()
                    << " Altitude: " << alt << "ft"
                    << " Velocity: " << velocity << "ft/sec"
                    << " Throttle: " << commandBuffer.reader[0]
                    << " work=" << elapsed.count() << "ms total=" << totalCycle.count() << "ms" << std::endl;
            }

            start = std::chrono::steady_clock::now();
        }

        std::cout << "All Cycles total=" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startAllCycles).count() << "ms" << std::endl;
        stopped = true;
    }
};

#endif