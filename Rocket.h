#ifndef ROCKET_H
#define ROCKET_H

#include "Mat3x3.h"
#include "Engine.h"
#include "FuelTank.h"
#include "Bus.h"
#include "AttitudeMode.h"

class Rocket {
private:
    Bus& bus;
    const float DRAG_COEFF = 0.3; //fix with real drag coefficent class later on? 
    const float AREA = 70;
    float dryMass;
    AttitudeMode currentMode = ATTITUDE_HOLD;
    Mat3x3 orientation;
    Vec3 forward;
    Vec3 position;
    Vec3 velocity;
    Vec3 acceleration;
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
                if (velocity.Magnitude() > 10.0f) {
                    // Compute pitch angle from velocity direction (2D approximation in x-z plane)
                    float pitchDegrees = atan2(velocity.getX(), velocity.getZ()) * 180.0f / M_PI;
                    orientation = Mat3x3::RotateY(pitchDegrees);
                }
                break;

            case LANDING_RETROGRADE:
                // TODO
                break;
        }
    }

public:
    Rocket(Bus& bus_) 
        : orientation(), dryMass(1200000.0f), forward(0,0,1), 
          position(0,0,0), velocity(0,0,0), acceleration (0,0,0),
          fuelTank(), bus(bus_), engine(bus_, fuelTank) {}
    
    Rocket(Bus& bus_, float throttle_, float fuel_, Vec3 startPos = Vec3(0,0,0), Vec3 startVel = Vec3(0,0,0), Vec3 startAccel = Vec3(0,0,0)) 
        : orientation(), dryMass(1200000.0f), forward(0,0,1), 
        position(startPos), velocity(startVel), acceleration(startAccel), 
        fuelTank(fuel_), bus(bus_), engine(bus_, fuelTank, throttle_) {}

    void Update(Vec3 externalForces, float dt) {
        engine.Update(dt);

        float modeCmd;
        if (bus.attitudeChannel.read(modeCmd)) {
            currentMode = (AttitudeMode)(int)modeCmd;
            std::cout << "Rocket received attitude mode: " << (int)currentMode << std::endl;
        }
        ApplyAttitudeMode();

        Vec3 thrustDirection = orientation * forward;
        Vec3 thrustForce = thrustDirection * engine.GetThrust();
        acceleration = (thrustForce + externalForces) * (1.0f / (dryMass + fuelTank.GetFuel()));
        velocity = velocity + acceleration * dt;
        position = position + velocity * dt;

        if (position.getZ() <= 0.0f) {
            position = Vec3(position.getX(), position.getY(), 0.0f);
            if (velocity.getZ() < 0.0f) {
                velocity = Vec3(velocity.getX(), velocity.getY(), 0.0f);
            }
        }
    }

    float GetDragCoef() const { return DRAG_COEFF; }

    float GetArea() const { return AREA; }

    float GetMass() const { return dryMass + fuelTank.GetFuel(); }

    float GetFuel() const { return fuelTank.GetFuel(); }

    Vec3 GetPosition() const { return position; }

    Vec3 GetVelocity() const { return velocity; }

    Vec3 GetAcceleration() const { return acceleration; }

    Vec3 GetForwardDirection() const { return orientation * forward; }

    void Print() const {
        printf("Rocket State\n");
        printf("Position: ");
        position.Print();
        printf("Velocity: ");
        velocity.Print();
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