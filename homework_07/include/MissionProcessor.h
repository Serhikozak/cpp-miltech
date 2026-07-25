#pragma once
#include "interfaces/Common.hpp"
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IBalisticSolver.h"
#include "JsonTargetProvider.h"
#include <iostream>
#include <cmath>

class MissionProcessor {
    private:
        IConfigLoader* m_loader;
        ITargetProvider* m_provider;
        IBalisticSolver* m_solver;

        int m_currentStep = 0;
        int m_totalStep = 0;
        float m_currentTime = 0.0f;

        Coord interpolateTarget(int targetIdx, int timeSteps,
                        float t, float arrayTimeStep) {
            int idx  = (int)std::floor(t / arrayTimeStep) % timeSteps;
            int next = (idx + 1) % timeSteps;
            float frac = (t / arrayTimeStep) - std::floor(t / arrayTimeStep);

            Coord currentPos = m_provider->getTarget(idx);
            Coord nextPos = m_provider->getTarget(next);

            return currentPos + (nextPos - currentPos) * frac;

            //return targets[targetIdx][idx]
                //+ (targets[targetIdx][next] - targets[targetIdx][idx]) * frac;
        }

        Coord extrapolateTarget(int targetIdx, int timeSteps,
                                float currentTime, float dt, float arrayTimeStep) {
            int idx  = (int)std::floor(currentTime / arrayTimeStep) % timeSteps;
            int next = (idx + 1) % timeSteps;

            Coord currentPos = m_provider->getTarget(idx);
            Coord nextPos = m_provider->getTarget(next);

            //Coord vel = (targets[targetIdx][next] - targets[targetIdx][idx]) / arrayTimeStep;
            Coord vel = (nextPos - currentPos) / arrayTimeStep;
            Coord cur = interpolateTarget(targetIdx, timeSteps, currentTime, arrayTimeStep);

            return cur + vel * dt;
        }
    
    public:
        //Конструктор(паттерн) приймаємо інтерфейси через вказівники
        MissionProcessor(IConfigLoader* loader, ITargetProvider* provider, IBalisticSolver* solver)
        : m_loader(loader), m_provider(provider), m_solver(solver),
          m_currentStep(0), m_totalStep(60), m_currentTime(0.0f) {}

        ~MissionProcessor() = default;

        void init(const char* configSourse = nullptr) {
            m_currentStep = 0;
            m_currentTime = 0.0f;
            if (m_loader != nullptr) {
                m_loader -> load(); //Тут IConfigLoader зчитує config.json nf ammo.json
            }

                //if (m_provider !=nullptr) {
                m_totalStep = 60;

                //}
            }
        
        
    //Метод hasNext перевіряє чи не закінчились кроки симуляції
        bool hasNext() {
            return m_currentStep < m_totalStep;

        }
        //Обробити наступний крок часу з вибором однієї Наайближчої цілі(Lead Targeting)
        void step() {
            if (!hasNext()) return;
            std::cout << "Executing step " << m_currentStep << "..." << std::endl;
            //Отримуємо параметри конфігурації та боєприпасу від лоадера
            if (m_loader != nullptr && m_provider != nullptr && m_solver != nullptr) {
                DroneConfig config = m_loader->getConfig();
                AmmoParams* ammo = m_loader->getAmmoParams();

                //Крок часу симуляції беремо з конфігу (sinTimeStep = 0.1)
                //float dt = config.simTimeStep;
                float arrayTimeStep = config.arrayTimeStep;
                //int timeSteps = 60;

                //Синхронізуємо крок часу симуляції з провайдером
                auto jsonProvider = dynamic_cast< JsonTargetProvider*>(m_provider);
                if (jsonProvider) {
                    jsonProvider->updateTime(m_currentStep);
                }

                //Отримуємо поточну позицію першої цілі
                int targetCount = m_provider->getTargetCount();
                int bestTargetIdx = -1;
                float minTimeToHit = 99999.0f;
                Coord bestTargetPos{0.0, 0.0};

                //Алгоритм випередження. Перебираємо всі цілі і шукаємо ближчу за часом
                for (int i = 0; i < targetCount; ++i) {
                    Coord predictedPos = interpolateTarget(i, m_totalStep, m_currentTime, arrayTimeStep);

                    float dx = predictedPos.x - config.startPos.x;
                    float dy = predictedPos.y - config.startPos.y;
                    float distance = std::sqrt(dx * dx + dy * dy);

                    float timeToHit = distance / config.attackSpeed;

                    if (timeToHit < minTimeToHit) {
                        minTimeToHit = timeToHit;
                        bestTargetIdx = i;
                        bestTargetPos = predictedPos;
                        
                    }
                
                //m_solver->solve(config, ammo[0], exactTargetPos);
                }

                if (bestTargetIdx != -1) {
                    std::cout << "[Target Selected] Ціль ID: " << bestTargetIdx
                              << " | Упередження: (" << bestTargetPos.x << ", " << bestTargetPos.y << ")" << std::endl;
                    m_solver->solve(config, ammo[0], bestTargetPos);
                }
        }
        m_currentStep++;
        if (m_loader != nullptr) {
            m_currentTime += m_loader -> getConfig().simTimeStep;
        }
        else {
            m_currentTime += 0.1f;
        }
    }

    void reset() {
        m_currentStep = 0;
        m_currentTime =0.0f;
    }

    void changeSolver(IBalisticSolver* s) {
        if (s != nullptr) {
            std::cout << "[Strategy] Зміна балістичног солвера на льоту" << std::endl;
            m_solver = s;
        }
    }

    int getCurrentStep() const {
        return m_currentStep;
    }

        
};