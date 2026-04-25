#ifndef FLIGHTCOMPUTER_H
#define FLIGHTCOMPUTER_H

#include "LaunchSequence.h"
#include "Bus.h"
#include "PID.h"
#include "KalmanFilter1D.h"
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
    KalmanFilter1D kalman;
    WebSocketServer* wsServer = nullptr;
    bool gravityTurnStarted = false;
    bool ascentFollowStarted = false;
    float lastMass = 5000000.0f;
    float lastOrientX = 0.0f;
    float lastOrientZ = 1.0f;
    Vec3 lastPosition;
    Vec3 lastVelocity;
    Vec3 lastAcceleration;
    bool gpsFresh = false;
    static constexpr float Isp = 330.0f;
    static constexpr float g0 = 9.80665f;
    static constexpr float dryMass = 1200000.0f;
    float gravityLoss = 0.0f;


public:
    bool stopped = false;

    FlightComputer(Bus& bus_)
        : bus(bus_), landingPID(0.02f, 0.0f, 0.0f) {}

    FlightComputer(Bus& bus_, LaunchSequence::State startState)
        : bus(bus_), landingPID(0.02f, 0.0f, 0.0f) { 
            ls.setState(startState);
        }

    void setWebSocketServer(WebSocketServer* s) { wsServer = s; }

    float computeDeltaV() {
        if (lastMass <= dryMass) return 0.0f;
        return Isp * g0 * std::log(lastMass / dryMass);
    }

    void calculateGravityLoss() {
        float mu = 3.986e14f;
        float earthRadius = 6.371e6f;
        float r = earthRadius + (computeAltitude(lastPosition));
        float g = mu / (r * r);
        gravityLoss += g * dt;
    }

    float computeAltitude(Vec3 pos){
        return pos.Magnitude() - 6371000.0f;
    }

    void setThrottle(float throttle) {
        bus.throttleChannel.write(throttle);
        bus.throttleChannel.swapBuffers();
    }

    void updateKalman() {
        kalman.Predict(lastAcceleration.getZ(), dt);
        
        if (gpsFresh) {
            kalman.Update(lastPosition.getZ());
        }
    }

    Vec3 readPosition() {
        float posX, posY, posZ;
        gpsFresh = false;
        if (bus.posXChannel.read(posX) && bus.posYChannel.read(posY) && bus.posZChannel.read(posZ)) {
            lastPosition = Vec3(posX, posY, posZ);
            gpsFresh = true;
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

    Vec3 readAcceleration() {
        float accX, accY, accZ;
        if (bus.accXChannel.read(accX) && bus.accYChannel.read(accY) && bus.accZChannel.read(accZ)) {
            lastAcceleration = Vec3(accX, accY, accZ);
        }
        return lastAcceleration;
    }

    float readMass() {
        float val;
        if (bus.massChannel.read(val)) lastMass = val;
        return lastMass;
    }

    void readOrientation(float& ox, float& oz) {
        float val;
        if (bus.orientXChannel.read(val)) lastOrientX = val;
        if (bus.orientZChannel.read(val)) lastOrientZ = val;
        ox = lastOrientX;
        oz = lastOrientZ;
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
            Vec3 velocity = readVelocity();
            Vec3 acceleation = readAcceleration();
            updateKalman();
            float alt = computeAltitude(lastPosition);
            float velZ = velocity.getZ();
            float mass = readMass();
            float deltaV = computeDeltaV();
            float orientX, orientZ;
            readOrientation(orientX, orientZ);

            if (ls.getState() == "LIFTOFF") {
                calculateGravityLoss();
                
                if (alt >= 152.0f && !gravityTurnStarted) {
                    setAttitudeMode(LIFTOFF_KICK);
                    gravityTurnStarted = true;
                }
                
                if (gravityTurnStarted && !ascentFollowStarted && std::abs(velocity.getX()) > 1.5f) {
                    setAttitudeMode(ASCENT_FOLLOW_VELOCITY);
                    ascentFollowStarted = true;
                }
                
                if (alt >= 366.0f) {
                    ls.transition(ls.MAX_Q);
                    setThrottle(0.004f);
                }
            }
            else if (ls.getState() == "MAX_Q") {
                calculateGravityLoss();

                if (alt >= 457.0f) {
                    ls.transition(ls.MECO);
                    setThrottle(0.0f);
                }
            }
            else if (ls.getState() == "MECO") {
                if (alt <= 427.0 && velZ < 0) {
                    ls.transition(ls.LANDING);
                    setAttitudeMode(LANDING_RETROGRADE);
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
                    float throttle = hoverThrottle + landingPID.Compute(targetVel, velZ, dt);
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
                    << " Altitude: " << alt << "m"
                    << " Velocity: (" << velocity.getX() << ", " << velocity.getY() << ", " << velZ << ") m/sec"
                    << " Throttle: " << bus.throttleChannel.reader[0]
                    << " Mass: " << mass
                    << " DeltaV: " << deltaV << " m/s"
                    << " GravityLoss: " << gravityLoss << " m/s"
                    << " Orient X: " << orientX
                    << " Orient Z: " << orientZ
                    << " work=" << elapsed.count() << "ms total=" << totalCycle.count() << "ms" << std::endl;
                    

                if (wsServer) {
                    std::ostringstream js;
                    js << "{\"cycle\":" << cycle
                        << ",\"state\":\"" << ls.getState() << "\""
                        << ",\"posX\":" << pos.getX()
                        << ",\"posZ\":" << pos.getZ()
                        << ",\"altitude\":" << alt
                        << ",\"velX\":" << velocity.getX()
                        << ",\"velZ\":" << velocity.getZ()
                        << ",\"velZ_filtered\":" << velZ
                        << ",\"orientX\":" << orientX
                        << ",\"orientZ\":" << orientZ
                        << ",\"deltaV\":" << deltaV
                        << ",\"gravityLoss\":" << gravityLoss
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