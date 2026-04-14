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

class Rocket {
private:
    Bus& bus;
    const float DRAG_COEFF = 0.3; //fix with real drag coefficent class later on? 
    const float AREA = 70;
    float dryMass;
    AttitudeMode currentMode = ATTITUDE_HOLD;
    Integrator& integrator;
    World& world;
    State state;
    Mat3x3 orientation;
    Vec3 forward;
    Vec3 lastAcceleration;
    float currentTime = 0.0f;

    FuelTank fuelTank;
    Engine engine;

    void Rotate(const Mat3x3& rotation) {
        orientation = rotation * orientation;
    }

    void SetThrottle(float t) { engine.SetThrottle(t); }

    void ApplyAttitudeMode() {
        switch (currentMode) {
            case ATTITUDE_HOLD:
                break;

            case LIFTOFF_KICK:
                orientation = Mat3x3::RotateY(5.0f) * orientation;
                currentMode = ATTITUDE_HOLD;
                break;

            case ASCENT_FOLLOW_VELOCITY:
                if (GetVelocity().Magnitude() > 10.0f) {
                    // Compute pitch angle from velocity direction (2D approximation in x-z plane)
                    float pitchDegrees = atan2(GetVelocity().getX(), GetVelocity().getZ()) * 180.0f / M_PI;
                    orientation = Mat3x3::RotateY(pitchDegrees);
                }
                break;

            case LANDING_RETROGRADE:
                if (GetVelocity().Magnitude() > 1.0f) {
                    Vec3 retrograde = GetVelocity() * -1.0f;
                    float pitchDegrees = atan2(retrograde.getX(), retrograde.getZ()) * 180.0f / M_PI;
                    orientation = Mat3x3::RotateY(pitchDegrees);
                }
                break;
        }
    }

public:
    Rocket(Bus& bus_, Integrator& integrator_, World& world_) 
        : orientation(), dryMass(1200000.0f), forward(0,0,1),
          fuelTank(), bus(bus_), engine(bus_, fuelTank),
          integrator(integrator_), world(world_), state{0, 0, 0, 0, 0, 0, 5000000.0f} {}
    
    Rocket(Bus& bus_, Integrator& integrator_, World& world_, float throttle_, float fuel_, Vec3 startPos = Vec3(0,0,0), Vec3 startVel = Vec3(0,0,0), Vec3 startAccel = Vec3(0,0,0)) 
        : orientation(), dryMass(1200000.0f), forward(0,0,1),
        fuelTank(fuel_), bus(bus_), engine(bus_, fuelTank, throttle_),
        integrator(integrator_), world(world_), 
        state{startPos.getX(), startPos.getY(), startPos.getZ(), startVel.getX(), startVel.getY(), startVel.getZ(), 1200000.0f + fuel_} {}

    void Update(float dt) {
        engine.Update(dt);

        // Read attitude commands
        float modeCmd;
        if (bus.attitudeChannel.read(modeCmd)) {
            currentMode = (AttitudeMode)(int)modeCmd;
        }
        ApplyAttitudeMode();

        Vec3 thrustDir = orientation * forward;
        float thrust = engine.GetThrust();
        float burnRate = engine.GetBurnRate();
        float throttle = engine.GetThrottle();
        float Cd = DRAG_COEFF;
        float A = AREA;
        World& w = world;

        auto derivFn = [&](float t, const State& s) -> State {
            Vec3 gravity = w.ComputeGravity(s);
            Vec3 drag = w.ComputeDrag(s, Cd, A);
            Vec3 thrustForce = thrustDir * thrust;
            float m = s[6];

            Vec3 accel = (gravity + drag + thrustForce) * (1.0f / m);
            float dmdt = -burnRate * throttle;

            return {s[3], s[4], s[5],
                    accel.getX(), accel.getY(), accel.getZ(),
                    dmdt};
        };

        State currentDeriv = derivFn(currentTime, state);
        lastAcceleration = Vec3(currentDeriv[3], currentDeriv[4], currentDeriv[5]);

        state = integrator.step(state, derivFn, currentTime, dt);
        currentTime += dt;

        fuelTank.SetFuel(state[6] - dryMass);

        // ground collision clamp
        if (state[2] <= 0.0f) {
            state[2] = 0.0f;
            if (state[5] < 0.0f) state[5] = 0.0f;
        }
    }

    float GetDragCoef() const { return DRAG_COEFF; }

    float GetArea() const { return AREA; }

    float GetMass() const { return state[6]; }

    float GetFuel() const { return fuelTank.GetFuel(); }

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