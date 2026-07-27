#pragma once
#include "Common.hpp"

class IBalisticSolver {
    public:
        //Головний метод розрахунку балістики на основі параметрів буде обчислювати точку зустрічі
        virtual void solve(const DroneConfig& config, const AmmoParams& ammo, const Coord& targetPos) = 0;
        virtual ~IBalisticSolver() = default;
};