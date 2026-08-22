#pragma once
#include "interfaces/IBalisticSolver.h"
#include "AnalyticalSolver.h"
#include <string>

enum class SolverType {ANALYTICAL};

inline IBalisticSolver* createSolver(SolverType type) {
    return (type == SolverType::ANALYTICAL) ? new AnalyticalSolver() : nullptr;
}
