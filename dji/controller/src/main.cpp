/**
 *  @brief Flight controller for Rob's MSc.
 */

#include <iostream>

#include "flight_control_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

/*! main
 *
 */
int main(int argc, char** argv) {
  
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  
  if (vehicle == nullptr) {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Monitored Takeoff + Landing                                |"
    << std::endl;
  std::cout
    << "| [b] Monitored Takeoff + Position Control + Landing             |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch (inputChar)
  {
    case 'a':
      monitoredTakeoff(vehicle);
      monitoredLanding(vehicle);
      break;
    case 'b':
      monitoredTakeoff(vehicle);
      moveByPositionOffset(vehicle, 0, 6, 6, 30);
      moveByPositionOffset(vehicle, 6, 0, -3, -30);
      moveByPositionOffset(vehicle, -6, -6, 0, 0);
      monitoredLanding(vehicle);
      break;
    default:
      break;
  }

  return 0;
}
