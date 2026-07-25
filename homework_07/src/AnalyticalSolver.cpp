#include "AnalyticalSolver.h"
#include "interfaces/Common.hpp"
#include <iostream>

void AnalyticalSolver::solve(const DroneConfig& config, const AmmoParams& ammo, const Coord& targetPos) {
    //Передаємо реальні змінні: з config беремо altittude  і attackSpeed
    float timeOfFlight = calcTimeOfFlight(config.altitude, config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);

    //Рахуємо горизонтальну дистанцію
    float dropDistance = calcHDistance(timeOfFlight, config.attackSpeed, ammo.mass, ammo.drag, ammo.lift);

    std::cout << " [Balistics] Target position: (" << targetPos.x << ", " << targetPos.y << ")\n"
              << " [Balistics] Time of flight: " << timeOfFlight << " s\n"
              << " [Balistics] Drop forward distance: " << dropDistance << " m\n"
              << " ____________________________________________" << std::endl;
}