#ifndef WORLD_H
#define WORLD_H

#include "Rocket.h"

class World {
private:
    Vec3 gravity;

public:
    World() : gravity(0, 0, -9.8f) {}

    Vec3 ComputeForces(const Rocket& rocket) {
        return gravity * rocket.GetMass();
    }
};

#endif