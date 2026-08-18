#pragma once
#include "interfaces/Common.hpp"
#include "ThreadSafeQueue.h"
#include <mutex>
#include <atomic>

class DronePhysics {
private:
    DroneConfig m_config;
    
    // Внутрішній фізичний стан дрона
    Coord m_pos{0.0f, 0.0f};
    Coord m_speed{0.0f, 0.0f};
    float m_dir = 0.0f;
    int m_state = 0;
    float m_angledSpeed = 0.0f;
    float m_linearSpeedMag = 0.0f;

     // Фізичний час та крок інтегрування
    float m_timeSecSinceStart = 0.0f;
    float m_physicsTimeStep = 0.01f; // Рекомендовано робити меншим за simTimeStep
    float m_timeScale = 1.0f;        // Коефіцієнт масштабування часу з ТЗ

    // Мутекси та черга команд відповідно до ТЗ
    mutable std::mutex m_stateMutex;
    ThreadSafeQueue<DroneCommand> m_commandQueue;

    // Атомарні прапорці життєвого циклу
    std::atomic<bool> m_isReady{false};
    std::atomic<bool> m_keepRunning{false};

public:
    DronePhysics(const DroneConfig& config);
    ~DronePhysics() = default;

    // Головний метод потоку, який викликається в main()
    void run();

    // Методи керування життєвим циклом (вимоги ТЗ)
    void start() { m_keepRunning = true; }
    void stop() { m_keepRunning = false; }
    bool isThreadReady() const { return m_isReady; }

    // Потокобезпечні методи зв'язку з іншими потоками
    void sendCommand(const DroneCommand& cmd);
    DroneTelemetry getTelemetry(); // Блокується через mutable м'ютекс
};