#include "FlightComputer.h"
#include "World.h"
#include "SensorUnit.h"
#include "Rocket.h"
#include "Engine.h"
#include "Vec3.h"
#include "Mat3x3.h"

using namespace std;

int main() {
    World world;
    DoubleCircularBuffer velocityBuffer;
    DoubleCircularBuffer altitudeBuffer;
    DoubleCircularBuffer engineCommandBuffer;
    Engine engine(engineCommandBuffer);
    Rocket rocket(engine);
    SensorUnit altitudeSensors("SU-10-ALT", altitudeBuffer, rocket, 0);
    SensorUnit velocitySensors("SU-10-VEL", velocityBuffer, rocket, 1);
    FlightComputer fc(altitudeBuffer, velocityBuffer, engineCommandBuffer);

    std::thread altThread(&SensorUnit::run, &altitudeSensors);
    std::thread velThread(&SensorUnit::run, &velocitySensors);
    std::thread fcThread(&FlightComputer::run, &fc);

    while (!fc.stopped) {
        Vec3 forces = world.ComputeForces(rocket);
        rocket.Update(forces, 0.001f);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    altitudeSensors.stopped = true;
    velocitySensors.stopped = true;
    altThread.join();
    velThread.join();
    fcThread.join();
        
    return 0;
}