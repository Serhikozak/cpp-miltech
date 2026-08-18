#pragma once
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IBalisticSolver.h"
#include "DronePhysics.h"
#include "DroneStateManager.h"
#include <atomic>

class MissionProcessor {
private:
    // Інтерфейси та компоненти системи
    IConfigLoader* m_configLoader;
    ITargetProvider* m_targetProvider;
    IBalisticSolver* m_ballisticSolver;
    DronePhysics* m_dronePhysics; // Новий потокобезпечний об'єкт фізики (Потік 2)
    
    // Менеджер логіки 5 станів автомата
    DroneStateManager m_stateManager;
    DroneConfig m_config;

    // Стан місії
    int m_currentTargetIdx = 0;
    float m_simTimeStep = 0.05f;
    float m_timeScale = 1.0f; // Масштабування часу 

    // Атомарні прапорці для синхронізації з main() через методи
    std::atomic<bool> m_isReady{false};
    std::atomic<bool> m_keepRunning{false};

public:
    // Конструктор приймає всі необхідні компоненти, включаючи потік фізики
    MissionProcessor(IConfigLoader* loader, ITargetProvider* provider, 
                     IBalisticSolver* solver, DronePhysics* physics);
    ~MissionProcessor() = default;

    // Головний метод Потоку 3, який викликається безпосередньо в main()
    void run();

    // Методи керування життєвим циклом (вимоги вашої методички для main)
    void start() { m_keepRunning = true; }
    void stop() { m_keepRunning = false; }
    bool isThreadReady() const { return m_isReady; }
};
