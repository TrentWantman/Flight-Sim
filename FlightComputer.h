#ifndef FLIGHTCOMPUTER_H
#define FLIGHTCOMPUTER_H

#include "LaunchSequence.h"
#include "DoubleCircularBuffer.h"
#include "PID.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <algorithm>


class FlightComputer {
private:
    Bus& bus;
    LaunchSequence ls;
    static constexpr float dt = 0.1f;
    PID landingPID;


public:
    bool stopped = false;

    FlightComputer(Bus& bus_)
        : bus(bus_), landingPID(0.02f, 0.0f, 0.0f) {}

    FlightComputer(Bus& bus_, LaunchSequence::State startState)
        : bus(bus_), landingPID(0.02f, 0.0f, 0.0f) { 
            ls.setState(startState);
        }

    void setThrottle(float throttle) {
        bus.throttleChannel.write(throttle);
        bus.throttleChannel.swapBuffers();
    }

    float readAltitude() {
        float val;
        if (bus.altitudeChannel.read(val)) return val;
        return 0.0f;
    }

    float readVelocity() {
        float val;
        if (bus.velocityChannel.read(val)) return val;
        return 0.0f;
    }

    float readMass() {
        float val;
        if (bus.massChannel.read(val)) return val;
        return 0.0f;
    }

    void run() {
        auto cycleTime = std::chrono::milliseconds(100);
        auto startAllCycles = std::chrono::steady_clock::now();
        auto start = startAllCycles;

        if (ls.getState() == "IDLE"){
            ls.transition(ls.PRELAUNCH);
            ls.transition(ls.IGNITION);
            ls.transition(ls.LIFTOFF);

            setThrottle(0.95f);
        }

        int cycle = 0;

        while (ls.getState() != "SAFED") {
            cycle++;

            float alt = readAltitude();
            float velocity = readVelocity();
            float mass = readMass();

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
                if (alt <= 1400.0 && velocity < 0) {
                    ls.transition(ls.LANDING);
                    setThrottle(1.0f);
                }
            }
            else if (ls.getState() == "LANDING") {
                if (alt <= 0.0) {
                    ls.transition(ls.SAFED);
                    setThrottle(0.0f);
                }
                else {
                    float targetVel = -0.05f * alt - 1.0f;
                    float hoverThrottle = (9.8 * mass) / 70000000;
                    float throttle = hoverThrottle + landingPID.Compute(targetVel, velocity, dt);
                    if (throttle > 1.0f) throttle = 1.0f;
                    if (throttle < 0.0f) throttle = 0.0f;                   
                    setThrottle(throttle);
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
                    << " Throttle: " << bus.throttleChannel.reader[0]
                    << " Mass: " << mass
                    << " work=" << elapsed.count() << "ms total=" << totalCycle.count() << "ms" << std::endl;
            }

            start = std::chrono::steady_clock::now();
        }

        std::cout << "All Cycles total=" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startAllCycles).count() << "ms" << std::endl;
        stopped = true;
    }
};

#endif