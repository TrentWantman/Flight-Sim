#ifndef ROCKET_H
#define ROCKET_H

#include "Mat3x3.h"
#include "Engine.h"
#include "FuelTank.h"

class Rocket {
private:
    const float DRAG_COEFF = 0.3;
    const float AREA = 70;
    float dryMass;
    Mat3x3 orientation;
    Vec3 forward;
    Vec3 position;
    Vec3 velocity;
    FuelTank fuelTank;
    Engine engine;

    void Rotate(const Mat3x3& rotation) {
        orientation = rotation * orientation;
    }

    void SetThrottle(float t) { engine.SetThrottle(t); }

public:
    Rocket(DoubleCircularBuffer& cmdBuffer) 
        : orientation(), dryMass(1200000.0f), forward(0,0,1), 
          position(0,0,0), velocity(0,0,0), 
          fuelTank(), engine(cmdBuffer, fuelTank) {}
    
    Rocket(DoubleCircularBuffer& cmdBuffer, float throttle_, float fuel_, float startAlt, float startVel) 
    : orientation(), dryMass(1200000.0f), forward(0,0,1), 
      position(0,0,startAlt), velocity(0,0,startVel), fuelTank(fuel_), engine(cmdBuffer, fuelTank, throttle_) {}

    void Update(Vec3 externalForces, float dt) {
        engine.Update(dt);
        Vec3 thrustDirection = orientation * forward;
        Vec3 thrustForce = thrustDirection * engine.GetThrust();
        Vec3 acceleration = (thrustForce + externalForces) * (1.0f / (dryMass + fuelTank.GetFuel()));
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