#pragma once
#include "interfaces/Common.hpp"
#include "drone_link.h"
#include <mutex>
#include <atomic>
#include <gpiod.h>
#include <string>

class DroneLink {
private:
    int m_uartFd;
    struct gpiod_line* m_dropLine{nullptr};

    //Внутрішне сховище бінарних структур протоколу dlink
    dlink::Telemetry m_rawTelemetry{};
    dlink::AmmoCfg m_rawAmmo{};
    dlink::TargetPos m_rawTarget{};
    dlink::DroneCfg m_rawConfig{};

    bool m_hasAmmo   = false;
    bool m_hasTarget = false;
    bool m_hasConfig = false;
    bool m_dropped   = false;

    //Об'єкти низькорівневого парсера
    dlink::Parser m_parser{};
    uint8_t m_buf = 0;
    uint8_t m_payloadBuffer[512];

    mutable std::mutex m_stateMutex;
    // Атомарні прапорці життєвого циклу
    std::atomic<bool> m_isReady{false};
    std::atomic<bool> m_keepRunning{false};

public:
    DroneLink(const std::string& uartDev, gpiod_line* dropLine);
    ~DroneLink() = default;

    // Головний метод потоку, який викликається в main()
    void run();

    // Методи керування життєвим циклом 
    void start() { m_keepRunning = true; }
    void stop() { m_keepRunning = false; }
    bool isThreadReady() const { return m_isReady; }
    bool isOpen() const { return m_uartFd >= 0; }

    DroneTelemetry getTelemetry();
    AmmoParams getAmmoParams();
    Target getTarget();
    DroneConfig getConfig();

    // Потокобезпечні методи зв'язку з іншими потоками
    void sendCommand(const DroneCommand& cmd);
    void triggerDrop();
    bool isDropped() const { return m_dropped; }
};