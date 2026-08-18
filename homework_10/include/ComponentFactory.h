#pragma once
#include <memory>
#include <string>
#include <map>
#include <functional>

// Підключаємо інтерфейси, які фабрика буде створювати
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/IBalisticSolver.h"

class ComponentFactory {
public:
    ComponentFactory();
    ~ComponentFactory() = default;

    // Фабричні методи для створення компонентів системи
    std::unique_ptr<ITargetProvider> createTargetProvider(const std::string& type, const std::string& param);
    std::unique_ptr<IConfigLoader> createConfigLoader(const std::string& type, const std::string& param);
    std::unique_ptr<IBalisticSolver> createBallisticSolver(const std::string& type);

private:
    // Використовуємо вимогу ДЗ: std::map замість ланцюжків if/else
    // Ключ — назва типу (наприклад, "json"), значення — функція, яка створює об'єкт
    std::map<std::string, std::function<std::unique_ptr<ITargetProvider>(const std::string&)>> m_providerRegistry;
    std::map<std::string, std::function<std::unique_ptr<IConfigLoader>(const std::string&)>> m_loaderRegistry;
    std::map<std::string, std::function<std::unique_ptr<IBalisticSolver>()>> m_solverRegistry;

    void registerComponents();
};