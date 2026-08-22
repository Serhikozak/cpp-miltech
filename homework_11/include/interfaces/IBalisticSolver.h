#pragma once
#include "Common.hpp"

class IBalisticSolver {
    public:
        //Головний метод розрахунку балістики на основі параметрів буде обчислювати точку зустрічі
        //virtual void solve(const DroneConfig& config, const AmmoParams& ammo, const Coord& targetPos) = 0;
        virtual ~IBalisticSolver() = default;

        virtual float calcTimeOfFlight(float Z0, float V0, float m, float d,float l) = 0;
        virtual float calcHDistance(float t, float V0, float m, float d,float l) = 0;
};