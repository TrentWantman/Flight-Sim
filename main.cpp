#include "FlightComputer.h"
#include "LaunchSequence.h"
#include "World.h"
#include "SensorUnit.h"
#include "Rocket.h"
#include "Engine.h"
#include "Vec3.h"
#include "Mat3x3.h"
#include "WebSocketServer.h"

using namespace std;

int main() {
    WebSocketServer wsServer;
    if (!wsServer.start(9002)) {
        std::cerr << "Warning: failed to start WebSocket server on port 9002" << std::endl;
    } else {
        std::cout << "WebSocket server listening on port 9002" << std::endl;
    }

    World world;
    DoubleCircularBuffer velocityBuffer;
    DoubleCircularBuffer altitudeBuffer;
    DoubleCircularBuffer engineCommandBuffer;
    DoubleCircularBuffer massBuffer;

    //Full Launch Test
    Rocket rocket(engineCommandBuffer);
    SensorUnit altitudeSensors("SU-10-ALT", altitudeBuffer, rocket, 0);
    SensorUnit velocitySensors("SU-10-VEL", velocityBuffer, rocket, 1);
    SensorUnit massSensor("SU-10-FUEL", massBuffer, rocket, 2);
    FlightComputer fc(altitudeBuffer, velocityBuffer, engineCommandBuffer, massBuffer);
    fc.setWebSocketServer(&wsServer);

    //Meco->Landing Test
    // Rocket rocket(engineCommandBuffer, 0.004f, 12000000, 1202.78f, 91.5942f);
    // SensorUnit altitudeSensors("SU-10-ALT", altitudeBuffer, rocket, 0);
    // SensorUnit velocitySensors("SU-10-VEL", velocityBuffer, rocket, 1);
    // SensorUnit massSensor("SU-10-FUEL", massBuffer, rocket, 2);
    // FlightComputer fc(altitudeBuffer, velocityBuffer, engineCommandBuffer, massBuffer, LaunchSequence::MAX_Q);

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

    wsServer.stop();

    return 0;
}