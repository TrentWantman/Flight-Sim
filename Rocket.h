#ifndef ROCKET_H
#define ROCKET_H

#include "Mat3x3.h"
#include "Engine.h"
#include "FuelTank.h"
#include "Bus.h"
#include "AttitudeMode.h"
#include "Integrator.h"
#include "World.h"
#include "State.h"
#include <cmath>

class Rocket {
private:
    static constexpr double EPSILON = 1e-6;

    Bus& bus;
    const double DRAG_COEFF = 0.3; //fix with real drag coefficent class later on?
    const double AREA = 70.0;
    double dryMass;
    AttitudeMode currentMode = ATTITUDE_HOLD;
    Integrator& integrator;
    World& world;
    State state;
    Mat3x3 orientation;
    Vec3 forward;
    Vec3 lastAcceleration;
    double currentTime = 0.0;

    FuelTank fuelTank;
    Engine engine;

    void Rotate(const Mat3x3& rotation) {
        orientation = rotation * orientation;
    }

    void SetThrottle(double t) { engine.SetThrottle(t); }

    void ApplyAttitudeMode() {
        switch (currentMode) {
            case ATTITUDE_HOLD:
                break;

            case LIFTOFF_KICK:
                orientation = Mat3x3::RotateY(5.0) * orientation;
                currentMode = ATTITUDE_HOLD;
                break;

            case ASCENT_FOLLOW_VELOCITY:
                if (GetVelocity().Magnitude() > 10.0) {
                    // Compute pitch angle from velocity direction (2D approximation in x-z plane)
                    double pitchDegrees = atan2(GetVelocity().getX(), GetVelocity().getZ()) * 180.0 / M_PI;
                    orientation = Mat3x3::RotateY(pitchDegrees);
                }
                break;

            case LANDING_RETROGRADE:
                if (GetVelocity().Magnitude() > 1.0) {
                    Vec3 retrograde = GetVelocity() * -1.0;
                    double pitchDegrees = atan2(retrograde.getX(), retrograde.getZ()) * 180.0 / M_PI;
                    orientation = Mat3x3::RotateY(pitchDegrees);
                }
                break;
        }
    }

public:
    Rocket(Bus& bus_, Integrator& integrator_, World& world_)
        : orientation(), dryMass(1200000.0), forward(0,0,1),
          fuelTank(), bus(bus_), engine(bus_, fuelTank),
          integrator(integrator_), world(world_), state{0, 0, 6371000.0, 0, 0, 0, 5000000.0} {}

    Rocket(Bus& bus_, Integrator& integrator_, World& world_, double throttle_, double fuel_, Vec3 startPos = Vec3(0,0,6371000), Vec3 startVel = Vec3(0,0,0), Vec3 startAccel = Vec3(0,0,0))
        : orientation(), dryMass(1200000.0), forward(0,0,1),
        fuelTank(fuel_), bus(bus_), engine(bus_, fuelTank, throttle_),
        integrator(integrator_), world(world_),
        state{startPos.getX(), startPos.getY(), startPos.getZ(),
              startVel.getX(), startVel.getY(), startVel.getZ(),
              dryMass + fuel_} {}

    void Update(double dt) {
        engine.Update(dt);

        // Read attitude commands
        float modeCmd;
        if (bus.attitudeChannel.read(modeCmd)) {
            currentMode = (AttitudeMode)(int)modeCmd;
        }
        ApplyAttitudeMode();

        Vec3 thrustDir = orientation * forward;
        double thrust = engine.GetThrust();
        double burnRate = engine.GetBurnRate();
        double throttle = engine.GetThrottle();
        double Cd = DRAG_COEFF;
        double A = AREA;
        World& w = world;

        auto derivFn = [&](double t, const State& s) -> State {
            Vec3 gravity = w.ComputeGravity(s);
            Vec3 drag = w.ComputeDrag(s, Cd, A);
            Vec3 thrustForce = thrustDir * thrust;
            double m = s[6];

            Vec3 totalForce = gravity + drag + thrustForce;
            double ax = totalForce.getX() / m;
            double ay = totalForce.getY() / m;
            double az = totalForce.getZ() / m;
            double dmdt = -burnRate * throttle;

            return {s[3], s[4], s[5], ax, ay, az, dmdt};
        };

        State currentDeriv = derivFn(currentTime, state);
        lastAcceleration = Vec3(currentDeriv[3], currentDeriv[4], currentDeriv[5]);

        state = integrator.step(state, derivFn, currentTime, dt);
        currentTime += dt;

        fuelTank.SetFuel(state[6] - dryMass);

        // ground collision clamp
        double px = state[0], py = state[1], pz = state[2];
        double r = std::sqrt(px*px + py*py + pz*pz);
        if (r <= World::EARTH_RADIUS + EPSILON) {
            double invR = 1.0 / r;
            double dirX = px * invR, dirY = py * invR, dirZ = pz * invR;
            state[0] = dirX * World::EARTH_RADIUS;
            state[1] = dirY * World::EARTH_RADIUS;
            state[2] = dirZ * World::EARTH_RADIUS;
            // Kill radial velocity
            double vx = state[3], vy = state[4], vz = state[5];
            double radialVel = vx*dirX + vy*dirY + vz*dirZ;
            if (radialVel < 0) {
                state[3] = vx - dirX * radialVel;
                state[4] = vy - dirY * radialVel;
                state[5] = vz - dirZ * radialVel;
            }
        }
    }

    double GetThrottle() const { return engine.GetThrottle(); }

    double GetDragCoef() const { return DRAG_COEFF; }

    double GetArea() const { return AREA; }

    double GetMass() const { return state[6]; }

    double GetFuel() const { return fuelTank.GetFuel(); }

    Vec3 GetPosition() const { return Vec3(state[0], state[1], state[2]); }

    Vec3 GetVelocity() const { return Vec3(state[3], state[4], state[5]); }

    Vec3 GetAcceleration() const { return lastAcceleration; }

    Vec3 GetForwardDirection() const { return orientation * forward; }

    void Print() const {
        printf("Rocket State\n");
        printf("Position: ");
        GetPosition().Print();
        printf("Velocity: ");
        GetVelocity().Print();
        printf("Throttle: %f\n", engine.GetThrottle());
        printf("Fuel: %f kg\n", fuelTank.GetFuel());
        printf("Burn Rate: %f kg/s\n", engine.GetBurnRate());
        printf("Mass: %f\n", dryMass + fuelTank.GetFuel());
        printf("Thrust Direction: ");
        (orientation * forward).Print();
        printf("Orientation:\n");
        orientation.Print();
        printf("\n");
        printf("\n");
    }
};

#endif