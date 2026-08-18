#pragma once
#include "interfaces/ITargetProvider.h"
//#include <nlohmann/json.hpp>
#include "interfaces/Common.hpp"
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

class JsonTargetProvider : public ITargetProvider {
    public:
        //Використовуємо  explicit та std::string замість char*
        explicit JsonTargetProvider(const std::string& param, float arrayTimeSter = 0.1f);
        ~JsonTargetProvider() override = default;
        
        void run(float targetTimeStep = 0.05f, float timeScale = 1.0f);

        void start() {m_keepRunning = true;}
        void stop() {m_keepRunning = false;}
        bool isThreadReady() const {return m_isReady;}     
        
        
        //реалізація інтерфейсних методів
        int getTargetCount() override;
        Coord getTarget(int index) override;

        void updateTime(int stepIdx);
    private:
    std::vector<TargetTrajectory> m_rawTrajectories; // Сирі траєкторії з JSON
    std::vector<Target> m_currentTargets;            // Поточні стани цілей (pos + velocity)
    
    float m_arrayTimeStep = 0.1f;
    int m_currentStepIdx = 0;

    mutable std::mutex m_mutex;
    std::atomic<bool> m_isReady{false};
    std::atomic<bool> m_keepRunning{false};

    // Старі змінні, закладені автором
    int m_timeStep = 0;
    int m_currentTimeIdx = 0;
};