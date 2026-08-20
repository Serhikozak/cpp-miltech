#pragma once
#include "interfaces/Common.hpp"
#include "drone_link.h"
#include <mutex>
#include <atomic>
#include <gpiod.h>

class DroneLink {
private:
    int m_uartFD;

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