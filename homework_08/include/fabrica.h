#pragma once
#include "interfaces/IBalisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"

#include "AnalyticalSolver.h"
#include "JsonTargetProvider.h"
#include "FileConfigLoader.h"

enum class SolverType {ANALYTICAL};
enum class ProviderType {JSON};
enum class LoaderType {FILE};

inline IBalisticSolver* createSolver(SolverType type) {
    return (type == SolverType::ANALYTICAL) ? new AnalyticalSolver() : nullptr;
}
inline ITargetProvider* createProvider(ProviderType type, const char* param) {
    return (type == ProviderType::JSON) ? new JsonTargetProvider(param) : nullptr;
}
inline IConfigLoader* createLoader(LoaderType type) {
    return (type == LoaderType::FILE) ? new FileConfigLoader() : nullptr;
}