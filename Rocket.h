#ifndef ROCKET_H
#define ROCKET_H

#include "Mat3x3.h"
#include "Engine.h"

class Rocket {
private:
    Mat3x3 orientation;
    float mass;
    Vec3 forward;
    Vec3 position;
    Vec3 velocity;
    Engine& engine;

    void Rotate(const Mat3x3& rotation) {
        orientation = rotation * orientation;
    }

    void SetThrottle(float t) { engine.SetThrottle(t); }

public:
    Rocket(Engine& engine_) : orientation(), mass(5000000.0f), forward(0,0,1), position(0,0,0), velocity(0,0,0), engine(engine_) {}

    void Update(Vec3 externalForces, float dt) {
        engine.ReadCommands();
        Vec3 thrustDirection = orientation * forward;
        Vec3 thrustForce = thrustDirection * engine.GetThrust();
        Vec3 acceleration = (thrustForce + externalForces) * (1.0f / mass);
        velocity = velocity + acceleration * dt;
        position = position + velocity * dt;

        if (position.getZ() <= 0.0f) {
            position = Vec3(position.getX(), position.getY(), 0.0f);
            if (velocity.getZ() < 0.0f) {
                velocity = Vec3(velocity.getX(), velocity.getY(), 0.0f);
            }
        }
    }

    float GetMass() const { return mass; }

    Vec3 GetPosition() const { return position; }

    Vec3 GetVelocity() const { return velocity; }

    void Print() const {
        printf("Rocket State\n");
        printf("Position: ");
        position.Print();
        printf("Velocity: ");
        velocity.Print();
        printf("Mass: %f\n", mass);
        printf("Throttle: %f\n", engine.GetThrottle());
        printf("Thrust Direction: ");
        (orientation * forward).Print();
        printf("Orientation:\n");
        orientation.Print();
        printf("\n");
    }
};

#endif