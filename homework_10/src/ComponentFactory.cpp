#include "ComponentFactory.h"
#include <stdexcept>

// Підключаємо конкретні реалізації (додайте ваші класи, якщо назви відрізняються)
#include "JsonTargetProvider.h"
#include "FileConfigLoader.h"
#include "AnalyticalSolver.h"

ComponentFactory::ComponentFactory() {
    registerComponents();
}

void ComponentFactory::registerComponents() {
    // 1. Реєструємо провайдери цілей
    m_providerRegistry["json"] = [](const std::string& param) {
        return std::make_unique<JsonTargetProvider>(param);
    };
    // Якщо у вас є текстовий провайдер, розкоментуйте та додайте за аналогією:
    // m_providerRegistry["text"] = [](const std::string& param) { return std::make_unique<TextTargetProvider>(param); };

    // 2. Реєструємо завантажувачі конфігурацій
    m_loaderRegistry["file"] = [](const std::string& param) {
        return std::make_unique<FileConfigLoader>();
    };

    // 3. Реєструємо балістичні солвери
    m_solverRegistry["analytical"] = []() {
        return std::make_unique<AnalyticalSolver>();
    };
}

std::unique_ptr<ITargetProvider> ComponentFactory::createTargetProvider(const std::string& type, const std::string& param) {
    auto it = m_providerRegistry.find(type);
    if (it != m_providerRegistry.end()) {
        return it->second(param); // Викликаємо лямбду з мапи
    }
    throw std::runtime_error("Unknown TargetProvider type: " + type);
}

std::unique_ptr<IConfigLoader> ComponentFactory::createConfigLoader(const std::string& type, const std::string& param) {
    auto it = m_loaderRegistry.find(type);
    if (it != m_loaderRegistry.end()) {
        return it->second(param);
    }
    throw std::runtime_error("Unknown ConfigLoader type: " + type);
}

std::unique_ptr<IBalisticSolver> ComponentFactory::createBallisticSolver(const std::string& type) {
    auto it = m_solverRegistry.find(type);
    if (it != m_solverRegistry.end()) {
        return it->second();
    }
    throw std::runtime_error("Unknown BallisticSolver type: " + type);
}