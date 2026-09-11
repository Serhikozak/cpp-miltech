#pragma once
#include "interfaces/Common.hpp"
#include <cmath>

class DroneStateManager {
private:
    
    DroneState m_state = STOPPED;
    float m_timeToStop = 0.0f;
    float m_currentDir = 0.0f;
public:
    DroneStateManager() = default;
    ~DroneStateManager() = default;

    // Ініціалізація початкового стану дрона з конфігу
    void init(const DroneConfig& config) {
        m_state = STOPPED;
        //m_timeToStop = 0.0f;
        m_currentDir = config.initialDir;
    }

               
    // Примусова зміна стану (потрібна після скидання бомби для переходу в DROPPED)
    void forceState(DroneState s) {
        m_state = s;
    }

    // Геттер для отримання поточного стану
    DroneState getState() const { 
        return m_state; 
    }

    // Головний метод розрахунку логіки — приймає телеметрію з потоку фізики та координати цілі
    DroneCommand update(const DroneTelemetry& telemetry, const Coord& bestTargetPos, const DroneConfig& config, float dt);
};   


        
        

