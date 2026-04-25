#include "FlightComputer.h"
#include "LaunchSequence.h"
#include "World.h"
#include "Rocket.h"
#include "Engine.h"
#include "Vec3.h"
#include "Mat3x3.h"
#include "IMUSensor.h"
#include "GPSSensor.h"
#include "FuelSensor.h"
#include "WebSocketServer.h"
#include "EulerIntegrator.h"
#include "RK4Integrator.h"
#include <sstream>
#include <cmath>

using namespace std;

int main() {
    WebSocketServer wsServer;
    if (!wsServer.start(9002)) {
        std::cerr << "Warning: failed to start WebSocket server on port 9002" << std::endl;
    } else {
        std::cout << "WebSocket server listening on port 9002" << std::endl;
    }

    World world;
    Bus bus;
    RK4Integrator integrator;

    // Full Launch Test
    Rocket rocket(bus, integrator, world);
    IMUSensor imu("IMU-1", bus, rocket);
    GPSSensor gps("GPS-1", bus, rocket);
    FuelSensor fuel("FUEL-1", bus, rocket);
    FlightComputer fc(bus);

    // MECO->Landing Test
    // Rocket rocket(bus, integrator, world, 0.0, 3294760.0, Vec3(0,0,1418.07f), Vec3(0,0,-81.4478f));
    // IMUSensor imu("IMU-1", bus, rocket);
    // GPSSensor gps("GPS-1", bus, rocket);
    // FuelSensor fuel("FUEL-1", bus, rocket);
    // FlightComputer fc(bus, LaunchSequence::MECO);

    std::thread imuThread(&IMUSensor::run, &imu);
    std::thread gpsThread(&GPSSensor::run, &gps);
    std::thread fuelThread(&FuelSensor::run, &fuel);
    std::thread fcThread(&FlightComputer::run, &fc);

    int count = 0;
    while (!fc.stopped) {
        count++;
        rocket.Update(0.001);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if (count % 100 == 0) {
            Vec3 pos = rocket.GetPosition();
            Vec3 vel = rocket.GetVelocity();
            Vec3 fwd = rocket.GetForwardDirection();
            double px = static_cast<double>(pos.getX());
            double py = static_cast<double>(pos.getY());
            double pz = static_cast<double>(pos.getZ());
            double simAltitude = std::sqrt(px*px + py*py + pz*pz) - 6371000.0;

            std::ostringstream js;
            js << "{"
               << "\"cycle\":" << count / 100
               << ",\"state\":\"" << fc.getState() << "\""
               << ",\"sim_altitude\":" << simAltitude
               << ",\"sim_posX\":" << pos.getX()
               << ",\"sim_velX\":" << vel.getX()
               << ",\"sim_velZ\":" << vel.getZ()
               << ",\"sim_orientX\":" << fwd.getX()
               << ",\"sim_orientZ\":" << fwd.getZ()
               << ",\"sim_throttle\":" << rocket.GetThrottle()
               << ",\"sim_mass\":" << rocket.GetMass()
               << ",\"fsw_altitude\":" << fc.getAltitudeEstimate()
               << ",\"fsw_velX\":" << fc.getVelXEstimate()
               << ",\"fsw_velZ\":" << fc.getVelZEstimate()
               << ",\"fsw_mass\":" << fc.getMassEstimate()
               << ",\"fsw_throttle\":" << fc.getFswThrottle()
               << ",\"fsw_deltaV\":" << fc.getDeltaV()
               << ",\"fsw_gravityLoss\":" << fc.getGravityLoss()
               << "}";
            wsServer.broadcast(js.str());
        }
    }

    imu.stopped = true;
    gps.stopped = true;
    fuel.stopped = true;

    imuThread.join();
    gpsThread.join();
    fuelThread.join();
    fcThread.join();

    return 0;
}