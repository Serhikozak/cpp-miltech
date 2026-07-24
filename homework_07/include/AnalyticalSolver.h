#pragma once
#include "interfaces/Common.hpp"
#include "interfaces/IBalisticSolver.h"

class AnalyticalSolver : public IBalisticSolver {
    public:
        AnalyticalSolver() = default;
        ~AnalyticalSolver() override = default;

        void solve(const DroneConfig& config, const AmmoParams& ammo, const Coord&targetPos);
};