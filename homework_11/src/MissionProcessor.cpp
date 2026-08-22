#include "MissionProcessor.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <unistd.h>



MissionProcessor::MissionProcessor(std::shared_ptr<DroneLink> droneLink, 
                                   std::unique_ptr<IBalisticSolver> solver)
    : m_droneLink(droneLink), m_balisticSolver(std::move(solver)) {
    
    m_keepRunning = true;
    
}

void MissionProcessor::run() {
    std::cout << "[MissionProcessor] Потік ШІ-наведення запущено." << std::endl;
    
    // 2. Запитуємо актуальні дані з UART адаптера DroneLink
    while (m_keepRunning) {
        // Отримуємо поточний безпечний знімок телеметрії з Потоку 2
        DroneTelemetry telemetry = m_droneLink->getTelemetry();
        AmmoParams ammo = m_droneLink->getAmmoParams();
        Target target = m_droneLink->getTarget();
        DroneConfig config = m_droneLink->getConfig();

        // Перевіряєм, чи є вже валідні дані по UART (координати не мають бути нульовими на старті)
        if (telemetry.pos.x == 0.0f && telemetry.pos.y == 0.0f) {
            usleep(10000);
            continue;
    }
    
    // Извлекаем висоту Z, яка тимчасово захована в velocity.x внутрі DroneLink
        float currentZ = target.velocity.x; 

        // 1. АЛГОРИТМ НАВЕДЕННЯ НА ЦіЛЬ
        float targetDir = std::atan2(target.pos.y - telemetry.pos.y, target.pos.x - telemetry.pos.x);
        
        // Поточний курс (передали dir через поле timeSecsSinceStart в DroneLink)
        float currentDir = telemetry.timeSecSinceStart; 

        float angle_error = targetDir - currentDir;
        while (angle_error > M_PI)  angle_error -= 2.0f * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0f * M_PI;

        // Формуємо команду поворота 
        DroneCommand cmd;
        cmd.state = 1; // Активний політ
        cmd.angelSpeed = 3.0f * angle_error;

        if (cmd.angelSpeed > 1.0f)  cmd.angelSpeed = 1.0f;
        if (cmd.angelSpeed < -1.0f) cmd.angelSpeed = -1.0f;

        // Відправляєм сформовану команду обратно в чекер через UART
        m_droneLink->sendCommand(cmd);

        // 2. БАЛІСТИЧНИЙ РОЗРАХУНОК СКИДУ
        float speed_total = std::hypot(telemetry.speed.x, telemetry.speed.y);
        
        // Викликаємо AnalyticalSolver через інтерфейс IBalisticSolver
        float t_fall = m_balisticSolver->calcTimeOfFlight(currentZ, speed_total, ammo.mass, ammo.drag, ammo.lift);

        // Розрахунок зміщення drop point (Полsном Тейлора 5-й степени внутрs AnalyticalSolver)
        float dropDistX = m_balisticSolver->calcHDistance(t_fall, telemetry.speed.x, ammo.mass, ammo.drag, ammo.lift);
        float dropDistY = m_balisticSolver->calcHDistance(t_fall, telemetry.speed.y, ammo.mass, ammo.drag, ammo.lift);

        float impact_x = telemetry.pos.x + dropDistX;
        float impact_y = telemetry.pos.y + dropDistY;

        // Точний розрахунок поточного промаха балістичної траєкторіі від цілі
        float miss_distance = std::hypot(target.pos.x - impact_x, target.pos.y - impact_y);

        // Якщо увійшли в радіус ураження боеприпаса и летимо рівно — генеруем фізичний імпульс скида
        if (miss_distance <= config.hitRadius && std::abs(angle_error) < 0.12f && !m_droneLink->isDropped()) {
            std::cout << "[MissionProcessor] ЦІЛЬ В ЗОНІ УРАЖЕННЯ! Виконується фізичний скид." << std::endl;
            m_droneLink->triggerDrop();
            m_keepRunning = false; // Успішне завершеня міссії
            break;
        }

        usleep(5000); // Такт роботи ШІ (5 мс)
    }
}