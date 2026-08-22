#pragma once
#include <memory>
#include "interfaces/IBalisticSolver.h"
#include "DroneLink.h"

class MissionProcessor {
private:
    std::shared_ptr<DroneLink> m_droneLink;
    std::unique_ptr<IBalisticSolver> m_balisticSolver;
    bool m_keepRunning = false;

public:
    // Конструктор приймає всі необхідні компоненти
    MissionProcessor( std::shared_ptr<DroneLink> m_droneLink,
                      std::unique_ptr<IBalisticSolver> solver);
    ~MissionProcessor() = default;

    // Головний метод Потоку 3, який викликається безпосередньо в main()
    void run();

    // Методи керування життєвим циклом (вимоги вашої методички для main)
    void start() { m_keepRunning = true; }
    void stop() { m_keepRunning = false; }
    
};
