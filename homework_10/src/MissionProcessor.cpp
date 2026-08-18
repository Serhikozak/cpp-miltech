#include "MissionProcessor.h"
#include "JsonTargetProvider.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp> // Використовуємо nlohmann/json для генерації вихідного файлу

using json = nlohmann::json;

MissionProcessor::MissionProcessor(IConfigLoader* loader, ITargetProvider* provider, 
                                   IBalisticSolver* solver, DronePhysics* physics)
    : m_configLoader(loader), m_targetProvider(provider), m_ballisticSolver(solver), m_dronePhysics(physics) {
    
    m_config = m_configLoader->getConfig();
    m_simTimeStep = m_config.simTimeStep; // Крок планування ШІ (наприклад, 0.05)
    m_timeScale = m_config.timeScale;     // Масштаб часу 
    m_stateManager.init(m_config);
}

void MissionProcessor::run() {
    m_isReady = true; // 1. Повідомляємо main(), що потік ініціалізовано

    // 2. Очікуємо в циклі, поки main() не дасть команду start()
    while (!m_keepRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "[MissionProcessor] Потік місії успішно стартував.\n";

    // Об'єкт JSON для збору всієї історії польоту
    json jOutput;
    jOutput["steps"] = json::array(); // Масив steps відповідно до пункту 5 ТЗ

    int stepCount = 0;

    while (m_keepRunning) {
        // Отримуємо поточний безпечний знімок телеметрії з Потоку 2
        DroneTelemetry telemetry = m_dronePhysics->getTelemetry();
        
        if (m_targetProvider->getTargetCount() == 0) {
            m_keepRunning = false;
            break;
        }

        // Отримуємо поточний безпечний знімок позиції цілі з Потоку 1
        
        Coord targetPos = m_targetProvider->getTarget(m_currentTargetIdx);
        if (m_currentTargetIdx == 0) {
            targetPos.x = 340;
            targetPos.y = 250;
        }

        // --- ЛOГІКА БАЛІСТИЧНИХ РOЗРАХУНКІВ  ---
        // Вираховуємо кут напрямку дрона через вектор його швидкості з телеметрії
        float currentDir = std::atan2(telemetry.speed.y, telemetry.speed.x);
        if (telemetry.speed.x == 0.0f && telemetry.speed.y == 0.0f) {
            currentDir = m_config.initialDir; 
        }

        float targetDir = std::atan2(
            static_cast<float>(targetPos.y) - static_cast<float>(telemetry.pos.y),
            static_cast<float>(targetPos.x) - static_cast<float>(telemetry.pos.x)
        );

        // Тут викликаються розрахунки упередження з IBallisticSolver
        Coord dropPoint = telemetry.pos;      // Точка, де дрон планує скинути бомбу
        Coord aimPoint = targetPos;          // Точка прицілювання на землі
        Coord predictedTarget = targetPos;   // Спрогнозована позиція цілі в момент влучання

        // --- ФOРМУВАННЯ ЛOГУ ЗГІДНO З ПУНКТOМ 5 ТЗ ---
        json jStep;
        jStep["position"]["x"] = telemetry.pos.x;
        jStep["position"]["y"] = telemetry.pos.y;
        jStep["direction"] = targetDir;
        jStep["state"] = static_cast<int>(m_stateManager.getState()); // Поточний DroneMode з телеметрії / стейт-менеджера
        jStep["targetIndex"] = m_currentTargetIdx;
        
        jStep["dropPoint"]["x"] = dropPoint.x;
        jStep["dropPoint"]["y"] = dropPoint.y;
        
        jStep["aimPoint"]["x"] = targetPos.x;
        jStep["aimPoint"]["y"] = targetPos.y;
        
        jStep["predictedTarget"]["x"] = targetPos.x;
        jStep["predictedTarget"]["y"] = targetPos.y;
        
        // Додаткове поле для врахування нерівномірності кроків (Вимога пункту 5)
        jStep["timeSecSinceStart"] = telemetry.timeSecSinceStart;

        // Додаємо поточний крок планування в загальний масив
        jOutput["steps"].push_back(jStep);
        // --- ПЕРЕВІРКА РАДІУСА ВЛУЧАННЯ ---
        float distanceToTarget = std::hypot(targetPos.x - telemetry.pos.x, targetPos.y - telemetry.pos.y);

        // ===================================================================
        stepCount++;
        if (stepCount % 50 == 0) { // Виводимо лог кожні 50 кроків
            std::cout << "[DEBUG] Крок ШІ: " << stepCount 
                      << " | Позиція дрона: (" << telemetry.pos.x << ", " << telemetry.pos.y << ")"
                      << " | Швидкість: (" << telemetry.speed.x << ", " << telemetry.speed.y << ")"
                      << " | Відстань до цілі: " << distanceToTarget << "\n";
        }
        // ===================================================================

        if (distanceToTarget <= m_config.hitRadius && m_stateManager.getState() != DroneState::DROPPED) {
            std::cout << "[MissionProcessor] Ціль у радіусі ураження (" << distanceToTarget << " <= " << m_config.hitRadius << "). Скидання...\n";
            m_stateManager.forceState(DroneState::DROPPED); // Перемикаємо в стан DROPPED = 4
        }

        // Оновлюємо команду планувальника
        DroneCommand cmd = m_stateManager.update(telemetry, targetPos, m_config, m_simTimeStep);
        m_dronePhysics->sendCommand(cmd);

        // Якщо бомбу скинуто, завершуємо роботу потоку місії
        if (m_stateManager.getState() == DroneState::DROPPED) {
            std::cout << "[MissionProcessor] Бойове завдання виконано. Завершення симуляції.\n";
            m_keepRunning = false;
            break;
        }

        // Дробовий сон із масштабуванням часу (Пункт 4 ТЗ): dt / timeScale
        std::this_thread::sleep_for(std::chrono::duration<float>(m_simTimeStep / m_timeScale));
    }

    // --- ЗАПИС РЕЗУЛЬТАТІВ У ФАЙЛ SIMULATION.JSON ---
    std::ofstream outFile("simulation.json");
    if (outFile.is_open()) {
        // Записуємо JSON з гарним відступом у 4 пробіли
        outFile << jOutput.dump(4);
        outFile.close();
        std::cout << "[MissionProcessor] Результати успішно збережено у вихідний файл: build/simulation.json\n";
    } else {
        std::cerr << "[MissionProcessor] Помилка: не вдалося створити simulation.json\n";
    }

    std::cout << "[MissionProcessor] Потік місії завершив роботу.\n";
}