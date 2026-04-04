#include "Mat3x3.h"
#include "Engine.h"

class Rocket{
private:
    Mat3x3 orientation;
    float mass;
    Vec3 position;
    Vec3 velocity;
    Engine engine;

public:
    Rocket() : orientation(), mass(1000.0f), position(0,0,0), velocity(0,0,0), engine() {}
    void Update(Vec3 externalForces, float dt);
    void Rotate(Mat3x3 rotation);
};