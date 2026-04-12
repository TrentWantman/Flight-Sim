#include "FlightComputer.h"
#include "LaunchSequence.h"
#include "World.h"
#include "SensorUnit.h"
#include "Rocket.h"
#include "Engine.h"
#include "Vec3.h"
#include "Mat3x3.h"

using namespace std;

int main() {
    World world;
    Bus bus;
        
    // Full Launch Test
    Rocket rocket(bus);
    SensorUnit altitudeSensors("SU-10-ALT", bus, rocket, 0);
    SensorUnit velocitySensors("SU-10-VEL", bus, rocket, 1);
    SensorUnit massSensor("SU-10-FUEL", bus, rocket, 2);
    FlightComputer fc(bus);

    //Meco->Landing Test
    // Rocket rocket(bus, 0.0f, 3294760.0f, 1418.07f, -81.4478f);
    // SensorUnit altitudeSensors("SU-10-ALT", bus, rocket, 0);
    // SensorUnit velocitySensors("SU-10-VEL", bus, rocket, 1);
    // SensorUnit massSensor("SU-10-FUEL", bus, rocket, 2);
    // FlightComputer fc(bus, LaunchSequence::MECO);

    std::thread altThread(&SensorUnit::run, &altitudeSensors);
    std::thread velThread(&SensorUnit::run, &velocitySensors);
    std::thread massThread(&SensorUnit::run, &massSensor);
    std::thread fcThread(&FlightComputer::run, &fc);


    int count = 0;
    while (!fc.stopped) {
        count ++;
        Vec3 forces = world.ComputeForces(rocket);
        rocket.Update(forces, 0.001f);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    altitudeSensors.stopped = true;
    velocitySensors.stopped = true;
    altThread.join();
    velThread.join();
    massThread.join();
    fcThread.join();
        
    return 0;
}