#ifndef FLIGHTCOMPUTER_H
#define FLIGHTCOMPUTER_H

#include "LaunchSequence.h"
#include "Bus.h"
#include "PID.h"
#include "AttitudeMode.h"
#include "Vec3.h"
#include "WebSocketServer.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <algorithm>
#include <sstream>


class FlightComputer {
private:
    Bus& bus;
    LaunchSequence ls;
    static constexpr float dt = 0.1f;
    PID landingPID;
    WebSocketServer* wsServer = nullptr;
    bool gravityTurnStarted = false;
    bool ascentFollowStarted = false;
    float lastMass = 5000000.0f;
    Vec3 lastPosition;
    Vec3 lastVelocity;


public:
    bool stopped = false;

    FlightComputer(Bus& bus_)
        : bus(bus_), landingPID(0.02f, 0.0f, 0.0f) {}

    FlightComputer(Bus& bus_, LaunchSequence::State startState)
        : bus(bus_), landingPID(0.02f, 0.0f, 0.0f) { 
            ls.setState(startState);
        }

    void setWebSocketServer(WebSocketServer* s) { wsServer = s; }

    void setThrottle(float throttle) {
        bus.throttleChannel.write(throttle);
        bus.throttleChannel.swapBuffers();
    }

    Vec3 readPosition() {
        float posX, posY, posZ;
        if (bus.posXChannel.read(posX) && bus.posYChannel.read(posY) && bus.posZChannel.read(posZ)) {
            lastPosition = Vec3(posX, posY, posZ);
        }
        return lastPosition;
    }

    Vec3 readVelocity() {
        float velX, velY, velZ;
        if (bus.velXChannel.read(velX) && bus.velYChannel.read(velY) && bus.velZChannel.read(velZ)) {
            lastVelocity = Vec3(velX, velY, velZ);
        }
        return lastVelocity;
    }

    float readMass() {
        float val;
        if (bus.massChannel.read(val)) lastMass = val;
        return lastMass;
    }

    void setAttitudeMode(AttitudeMode mode) {
        bus.attitudeChannel.write((float)mode);
        bus.attitudeChannel.swapBuffers();
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

            Vec3 pos = readPosition();
            float alt = pos.getZ();
            Vec3 velocity = readVelocity();
            float mass = readMass();

            if (ls.getState() == "LIFTOFF") {
                if (alt >= 500.0f && !gravityTurnStarted) {
                    setAttitudeMode(LIFTOFF_KICK);
                    gravityTurnStarted = true;
                }
                
                if (gravityTurnStarted && !ascentFollowStarted && std::abs(velocity.getX()) > 5.0f) {
                    setAttitudeMode(ASCENT_FOLLOW_VELOCITY);
                    ascentFollowStarted = true;
                }
                
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
                if (alt <= 1400.0 && velocity.getZ() < 0) {
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
                    float throttle = hoverThrottle + landingPID.Compute(targetVel, velocity.getZ(), dt);
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
                    << " Velocity: (" << velocity.getX() << ", " << velocity.getY() << ", " << velocity.getZ() << ") ft/sec"
                    << " Throttle: " << bus.throttleChannel.reader[0]
                    << " Mass: " << mass
                    << " work=" << elapsed.count() << "ms total=" << totalCycle.count() << "ms" << std::endl;

                if (wsServer) {
                    std::ostringstream js;
                    js << "{\"cycle\":" << cycle
                       << ",\"state\":\"" << ls.getState() << "\""
                       << ",\"altitude\":" << alt
                       << ",\"velocity\":" "(" << velocity.getX() << ", " << velocity.getY() << ", " << velocity.getZ() << ") ft/sec"
                       << ",\"throttle\":" << bus.throttleChannel.reader[0]
                       << ",\"mass\":" << mass << "}";
                    wsServer->broadcast(js.str());
                }
            }

            start = std::chrono::steady_clock::now();
        }

        std::cout << "All Cycles total=" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startAllCycles).count() << "ms" << std::endl;
        stopped = true;
    }
};

#endif