#ifndef FLIGHTCOMPUTER_H
#define FLIGHTCOMPUTER_H

#include "LaunchSequence.h"
#include "Bus.h"
#include "PID.h"
#include "KalmanFilter1D.h"
#include "AttitudeMode.h"
#include "Vec3.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <cmath>


class FlightComputer {
private:
    Bus& bus;
    LaunchSequence ls;
    static constexpr double dt = 0.1;
    static constexpr double EPSILON = 1e-6;
    PID landingPID;
    KalmanFilter1D kalman;
    bool gravityTurnStarted = false;
    bool ascentFollowStarted = false;
    double lastMass = 5000000.0;
    float lastOrientX = 0.0f;
    float lastOrientZ = 1.0f;
    Vec3 lastPosition;
    Vec3 lastVelocity;
    Vec3 lastAcceleration;
    double lastThrottle = 0.0;
    bool gpsFresh = false;
    static constexpr double Isp = 330.0;
    static constexpr double g0 = 9.80665;
    static constexpr double dryMass = 1200000.0;
    double gravityLoss = 0.0;


public:
    bool stopped = false;

    FlightComputer(Bus& bus_)
        : bus(bus_), landingPID(0.02, 0.0, 0.0) {}

    FlightComputer(Bus& bus_, LaunchSequence::State startState)
        : bus(bus_), landingPID(0.02, 0.0, 0.0) {
            ls.setState(startState);
        }

    double computeDeltaV() const {
        if (lastMass <= dryMass) return 0.0;
        return Isp * g0 * std::log(lastMass / dryMass);
    }

    void calculateGravityLoss() {
        double mu = 3.986e14;
        double earthRadius = 6.371e6;
        double r = earthRadius + computeAltitude(lastPosition);
        double g = mu / (r * r);
        gravityLoss += g * dt;
    }

    double computeAltitude(Vec3 pos) const {
        double px = pos.getX(), py = pos.getY(), pz = pos.getZ();
        return std::sqrt(px*px + py*py + pz*pz) - 6371000.0;
    }

    void setThrottle(float throttle) {
        lastThrottle = static_cast<double>(throttle);
        bus.throttleChannel.write(throttle);
        bus.throttleChannel.swapBuffers();
    }

    void updateKalman() {
        kalman.Predict(static_cast<double>(lastAcceleration.getZ()), dt);

        if (gpsFresh) {
            kalman.Update(static_cast<double>(lastPosition.getZ()));
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

    double readMass() {
        float val;
        if (bus.massChannel.read(val)) lastMass = static_cast<double>(val);
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
            double alt = computeAltitude(lastPosition);
            double velZ = static_cast<double>(velocity.getZ());
            double mass = readMass();
            double deltaV = computeDeltaV();
            float orientX, orientZ;
            readOrientation(orientX, orientZ);

            if (ls.getState() == "LIFTOFF") {
                calculateGravityLoss();

                if (alt >= 152.0 && !gravityTurnStarted) {
                    setAttitudeMode(LIFTOFF_KICK);
                    gravityTurnStarted = true;
                }

                if (gravityTurnStarted && !ascentFollowStarted && std::abs(velocity.getX()) > 1.5f) {
                    setAttitudeMode(ASCENT_FOLLOW_VELOCITY);
                    ascentFollowStarted = true;
                }

                if (alt >= 366.0) {
                    ls.transition(ls.MAX_Q);
                    setThrottle(0.004f);
                }
            }
            else if (ls.getState() == "MAX_Q") {
                calculateGravityLoss();

                if (alt >= 457.0) {
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
                if (alt < EPSILON) {
                    ls.transition(ls.SAFED);
                    setThrottle(0.0f);
                }
                else {
                    double targetVel = -0.05 * alt - 1.0;
                    double hoverThrottle = (9.8 * mass) / 70000000.0;
                    double throttle = hoverThrottle + landingPID.Compute(targetVel, velZ, dt);
                    if (throttle > 1.0) throttle = 1.0;
                    if (throttle < 0.0) throttle = 0.0;
                    setThrottle(static_cast<float>(throttle));
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


            }

            start = std::chrono::steady_clock::now();
        }

        std::cout << "All Cycles total=" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startAllCycles).count() << "ms" << std::endl;
        stopped = true;
    }

    std::string getState() const { return ls.getState(); }
    double getDeltaV() const { return computeDeltaV(); }
    double getGravityLoss() const { return gravityLoss; }
    double getAltitudeEstimate() const { return computeAltitude(lastPosition); }
    double getVelZEstimate() const { return static_cast<double>(lastVelocity.getZ()); }
    double getVelXEstimate() const { return static_cast<double>(lastVelocity.getX()); }
    double getMassEstimate() const { return lastMass; }
    float getFswThrottle() const { return lastThrottle; }
};

#endif