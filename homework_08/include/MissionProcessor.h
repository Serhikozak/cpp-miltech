#pragma once

#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IBalisticSolver.h"
#include "JsonTargetProvider.h"
#include "DroneStateManager.h"
#include "interfaces/Common.hpp"
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <nlohmann/json.hpp>

class MissionProcessor {
private:
     IConfigLoader* m_loader;
    ITargetProvider* m_provider;
    IBalisticSolver* m_solver;
    DroneStateManager m_stateManager;

    int m_currentStep = 0;
    int m_totalStep = 60;
    const int MAX_STEPS = 10000;
    float m_currentTime = 0.0f;
    bool m_isDropped = false;

    // Вихідні масиви для збереження історії польоту
    std::vector<float> m_historyX;
    std::vector<float> m_historyY;
    std::vector<float> m_historyDir;
    std::vector<int> m_historyState;
    std::vector<int> m_historyTargetIdx;

    // Внутрішня функція інтерполяції (Lead Targeting)
    Coord interpolateTarget(int targetIdx, int timeSteps, float t, float arrayTimeStep) {
        int idx = (int)std::floor(t / arrayTimeStep) % timeSteps;
        int next = (idx + 1) % timeSteps;
        float frac = (t / arrayTimeStep) - std::floor(t / arrayTimeStep);

        auto jsonProvider = dynamic_cast<JsonTargetProvider*>(m_provider);
        if (!jsonProvider) return Coord{0.0, 0.0};

        jsonProvider->updateTime(idx);
        Coord currentPos = m_provider->getTarget(targetIdx);

        jsonProvider->updateTime(next);
        Coord nextPos = m_provider->getTarget(targetIdx);

        return currentPos + (nextPos - currentPos) * frac;
    }

    // Нова функція запису у ПРАВИЛЬНИЙ структурований simulation.json
    void writeSimulationFile() {
        std::ofstream fout("simulation.json");
        if (!fout.is_open()) {
            std::cerr << "Помилка: Не вдалося створити файл simulation.json" << std::endl;
            return;
        }

        nlohmann::json outputJson;
        int N = m_historyDir.size();

        outputJson["steps_count"] = N;

        // Формуємо масив об'єктів для кожного кроку часу (телеметрію)
        nlohmann::json telemetryArray = nlohmann::json::array();
        for (int i = 0; i < N; ++i) {
            nlohmann::json stepData;
            stepData["step_index"] = i;
            
            // Групуємо координати в акуратний підоб'єкт
            stepData["position"] = {
                {"x", m_historyX[i]},
                {"y", m_historyY[i]}
            };

            stepData["direction_rad"] = m_historyDir[i];
            stepData["drone_state"]   = m_historyState[i];
            stepData["target_index"]  = m_historyTargetIdx[i];

            telemetryArray.push_back(stepData);
        }

        outputJson["telemetry"] = telemetryArray;

        // Записуємо у файл із красивими відступами в 4 пробіли
        fout << outputJson.dump(4);
        fout.close();

        std::cout << "[File] Телеметрія місії успішно збережена у красивий simulation.json" << std::endl;
    }

public:
    // Конструктор
    MissionProcessor(IConfigLoader* loader, ITargetProvider* provider, IBalisticSolver* solver)
        : m_loader(loader), m_provider(provider), m_solver(solver), 
          m_currentStep(0), m_currentTime(0.0f), m_isDropped(false) {}

    ~MissionProcessor() = default;

    // Метод ініціалізації
    void init(const char* configSource = nullptr) {
        m_currentStep = 0;
        m_currentTime = 0.0f;
        m_isDropped = false;

        if (m_loader != nullptr) {
            m_loader->load();
            m_stateManager.init(m_loader->getConfig());
        }

        m_historyX.clear(); 
        m_historyY.clear();
        m_historyDir.clear(); 
        m_historyState.clear();
        m_historyTargetIdx.clear();
    }
    // Перевірка наявності наступного кроку
    bool hasNext() {
        return !m_isDropped && m_currentStep < MAX_STEPS;
    }
    // Головний метод кроку симуляції
    void step() {
        if (!hasNext()) return;

        DroneConfig config = m_loader->getConfig();
        AmmoParams* ammo = m_loader->getAmmoParams();
        float arrayTimeStep = config.arrayTimeStep;
        float dt = config.simTimeStep;

        int currentStepIdx = (int)std::floor(m_currentTime / arrayTimeStep) % m_totalStep;
        auto jsonProvider = dynamic_cast<JsonTargetProvider*>(m_provider);
        if (jsonProvider) {
            jsonProvider->updateTime(currentStepIdx);
        }

        int targetCount = m_provider->getTargetCount();
        int bestTargetIdx = -1;
        float minTimeToHit = 999999.0f;
        Coord bestTargetPos{0.0, 0.0};

        // Алгоритм Lead Targeting: пошук найближчої за часом цілі
        for (int i = 0; i < targetCount; ++i) {
            Coord predictedPos = interpolateTarget(i, m_totalStep, m_currentTime, arrayTimeStep);
            
            float dx = predictedPos.x - m_stateManager.getX();
            float dy = predictedPos.y - m_stateManager.getY();
            float distance = std::sqrt(dx * dx + dy * dy);
            float timeToHit = distance / config.attackSpeed;

            if (timeToHit < minTimeToHit) {
                minTimeToHit = timeToHit;
                bestTargetIdx = i;
                bestTargetPos = predictedPos;
            }
        }

        if (bestTargetIdx != -1) {
            // Оновлюємо фізику та стани автомата в менеджері
            m_stateManager.update(bestTargetPos, config, dt);

            // Перевірка досягнення радіуса скидання
            float finalDx = bestTargetPos.x - m_stateManager.getX();
            float finalDy = bestTargetPos.y - m_stateManager.getY();
            float finalDistance = std::sqrt(finalDx * finalDx + finalDy * finalDy);

            if (finalDistance <= config.hitRadius) {
                m_stateManager.forceState(DROPPED);
                m_isDropped = true;
            }
        }

        // Записуємо дані у вихідні масиви історії
        m_historyX.push_back(m_stateManager.getX());
        m_historyY.push_back(m_stateManager.getY());
        m_historyDir.push_back(m_stateManager.getDir());
        m_historyState.push_back((int)m_stateManager.getState());
        m_historyTargetIdx.push_back(bestTargetIdx);

        // Викликаємо стратегію балістичного солвера
        if (bestTargetIdx != -1) {
            m_solver->solve(config, ammo[0], bestTargetPos);
        }

        m_currentStep++;
        m_currentTime += dt;

        // Коли скинули або досягли ліміту — зберігаємо красивий JSON
        if (m_isDropped || m_currentStep >= MAX_STEPS) {
            writeSimulationFile();
        }
    }
    void reset() { init(); }
    
    void changeSolver(IBalisticSolver* s) { 
        if (s != nullptr) m_solver = s; 
    }
    
    int getCurrentStep() const { return m_currentStep; }
};








