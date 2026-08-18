#pragma once
#include "interfaces/IBalisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"

#include "AnalyticalSolver.h"
#include "JsonTargetProvider.h"
#include "FileConfigLoader.h"
#include <string>

enum class SolverType {ANALYTICAL};
enum class ProviderType {JSON};
enum class LoaderType {FILE};

inline IBalisticSolver* createSolver(SolverType type) {
    return (type == SolverType::ANALYTICAL) ? new AnalyticalSolver() : nullptr;
}
// Додаємо другий параметр float arrayTimeStep, який потрібен конструктору ThreadSafeTargetProvider
inline ITargetProvider* createProvider(ProviderType type, const char* param, float arrayTimeStep = 0.1f) {
    return (type == ProviderType::JSON && param != nullptr) 
        ? new JsonTargetProvider(std::string(param), arrayTimeStep) 
        : nullptr;
}
inline IConfigLoader* createLoader(LoaderType type) {
    return (type == LoaderType::FILE) ? new FileConfigLoader() : nullptr;
}