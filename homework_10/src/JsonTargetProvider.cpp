#include "JsonTargetProvider.h"
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Конструктор: зчитує сирі траєкторії з JSON та готує масив поточних цілей
JsonTargetProvider::JsonTargetProvider(const std::string& param, float arrayTimeStep)
    : m_arrayTimeStep(arrayTimeStep), m_timeStep(0), m_currentTimeIdx(0), m_currentStepIdx(0) {
    
    std::ifstream file(param);
    if (!file.is_open()) {
        std::cerr << "[Provider] Error: cannot open targets file " << param << std::endl;
        return;
    }

    try {
        json jData;
        file >> jData;

        // Парсимо масив цілей із JSON-файлу
        if (jData.contains("targets") && jData["targets"].is_array()) {
            for (const auto& targetJson : jData["targets"]) {
                TargetTrajectory trajectory;

                if (targetJson.contains("positions") && targetJson["positions"].is_array()) {
                    for (const auto& posJson : targetJson["positions"]) {
                        Coord c;
                        c.x = posJson.value("x", 0.0f);
                        c.y = posJson.value("y", 0.0f);
                        trajectory.positions.push_back(c);
                    }
                }

                // Зберігаємо сиру траєкторію в приватний масив провайдера
                m_rawTrajectories.push_back(trajectory);
                
                // Ініціалізуємо початковий поточний стан цілі (позиція першого вузла, швидкість 0)
                Target initialTarget{};
                if (!trajectory.positions.empty()) {
                    initialTarget.pos = trajectory.positions[0];
                }
                m_currentTargets.push_back(initialTarget);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[Provider] JSON Parsing error: " << e.what() << std::endl;
    }
}

// Повертає кількість цілей (безпечно під м'ютексом)
int JsonTargetProvider::getTargetCount() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return static_cast<int>(m_currentTargets.size());
}

// Інтерфейсний метод: повертає Coord поточної позиції цілі (безпечно під м'ютексом)
Coord JsonTargetProvider::getTarget(int index) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (index >= 0 && index < static_cast<int>(m_currentTargets.size())) {
        return m_currentTargets[index].pos; // Повертаємо тільки позицію, бо інтерфейс вимагає Coord
    }
    return Coord{0.0f, 0.0f};
}

// Старий метод оновлення часу (залишаємо для сумісності з попередніми ДЗ)
void JsonTargetProvider::updateTime(int stepIdx) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_timeStep = stepIdx;
    m_currentTimeIdx = stepIdx;
}

// ГОЛОВНИЙ ЦИКЛ ПОТОКУ ЦІЛЕЙ (Потік 1)
void JsonTargetProvider::run(float targetTimeStep, float timeScale) {
    m_isReady = true; // 1. Повідомляємо main(), що потік ініціалізовано та готовий до старту

    // 2. Очікуємо в циклі, поки в main() не викличуть метод provider->start()
    while (!m_keepRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "[Provider] Thread 1 loop started.\n";

    // 3. Активна фаза роботи Потоку 1
    while (m_keepRunning) {
        {
            // Замикаємо критичну секцію під локом — тільки копіювання та розрахунок даних
            std::lock_guard<std::mutex> lock(m_mutex);

            for (size_t i = 0; i < m_rawTrajectories.size(); ++i) {
                const auto& trajectory = m_rawTrajectories[i].positions;
                if (trajectory.empty()) continue;

                // Індекси поточної та наступної точок з урахуванням зациклення траєкторії (вимога з ТЗ)
                int currIdx = m_currentStepIdx % trajectory.size();
                int nextIdx = (m_currentStepIdx + 1) % trajectory.size();

                Coord currentPos = trajectory[currIdx];
                Coord nextPos = trajectory[nextIdx];

                // Розрахунок поточної швидкості: кінцева різниця сусідніх вузлів, поділена на arrayTimeStep
                Coord velocity{0.0f, 0.0f};
                if (m_arrayTimeStep > 0.0001f) {
                    velocity.x = (nextPos.x - currentPos.x) / m_arrayTimeStep;
                    velocity.y = (nextPos.y - currentPos.y) / m_arrayTimeStep;
                }

                // Оновлюємо поточний знімок (snapshot) для цієї цілі під м'ютексом
                m_currentTargets[i].pos = currentPos;
                m_currentTargets[i].velocity = velocity;
            }
            
            // Рухаємося до наступного кроку траєкторії
            m_currentStepIdx++;
        }

         float timeScale = 1.0f; // Коефіцієнт прискорення часу (за замовчуванням 1.0f)
        std::this_thread::sleep_for(std::chrono::duration<float>(m_arrayTimeStep / timeScale));
    }

    std::cout << "[Provider] Thread 1 loop finished.\n";
}