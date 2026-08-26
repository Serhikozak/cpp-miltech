#include "MissionProcessor.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

MissionProcessor::MissionProcessor(std::shared_ptr<DroneLink> droneLink, 
                                   std::unique_ptr<IBalisticSolver> solver)
    : m_droneLink(droneLink), m_balisticSolver(std::move(solver)) {
    
    m_keepRunning = true;
    
}

void MissionProcessor::run() {
    std::cout << "[MissionProcessor] Потік ШІ-наведення запущено." << std::endl;
    
    // Головний цикл багатопотоковості
    while (m_keepRunning) 
    {
        // Очікуємо на оновлення пакету телеметрії по UART
        if (!m_droneLink->isDataUpdated()) {
            usleep(2000); // Пауза 2 мс
            continue;
        }

        // 1. Офіційне оголошення змінних усередині поточного scope
        DroneTelemetry telemetry = m_droneLink->getTelemetry();
        Target target = m_droneLink->getTarget();
        AmmoParams ammo = m_droneLink->getAmmoParams();

        // Захист від відсутності файлу ammo.json
        if (ammo.mass <= 0.001f || std::isnan(ammo.mass)) {
            std::strcpy(ammo.name, "VOG-17");
            ammo.mass = 0.280f;
            ammo.drag = 0.025f;
            ammo.lift = 0.00f;
        }

        float currentZ = 100.0f;
        // АВТОНОМНИЙ РОЗРАХУНОК ВЕКТОРА ШВИДКОСТІ ЧЕРЕЗ ДЕЛЬТУ КООРДИНАТ ДРОНА
        static float last_x = telemetry.pos.x;
        static float last_y = telemetry.pos.y;
        static float last_time = telemetry.timeSecSinceStart;

        float dt = telemetry.timeSecSinceStart - last_time;
        float real_vx = 0.0f;
        float real_vy = 0.0f;

        if (dt > 0.001f) {
            real_vx = (telemetry.pos.x - last_x) / dt;
            real_vy = (telemetry.pos.y - last_y) / dt;
        }

        last_x = telemetry.pos.x;
        last_y = telemetry.pos.y;
        last_time = telemetry.timeSecSinceStart;        

        // Повний модуль горизонтальної швидкості дрона
        float speed_total = std::hypot(real_vx, real_vy);
        if (speed_total < 0.2f) {
            speed_total = telemetry.speed.x;
            real_vx = speed_total * 0.707f;
            real_vy = -speed_total * 0.707f; 
        } 

        float currentDir = std::atan2(real_vy, real_vx);
        
        // 2. БАЛІСТИЧНИЙ РОЗРАХУНОК 
        float t_fall = m_balisticSolver->calcTimeOfFlight(currentZ, speed_total, ammo.mass, ammo.drag, ammo.lift);
        if (std::isnan(t_fall) || std::isinf(t_fall) || t_fall <= 0.1f) {
            t_fall = std::sqrt((2.0f * currentZ) / 9.81f);
        }
        // Розкладаємо однакову швидкість по осях через тригонометрію реального курсу
        //float real_vx = telemetry.speed.x * std::cos(currentDir);
        //float real_vy = telemetry.speed.x * std::sin(currentDir);

        float dropDistX = m_balisticSolver->calcHDistance(t_fall, real_vx, ammo.mass, ammo.drag, ammo.lift);
        float dropDistY = m_balisticSolver->calcHDistance(t_fall, real_vy, ammo.mass, ammo.drag, ammo.lift);

        // СТРАХОВКА: Якщо внутрішній Тейлор злетів на нульових швидкостях, підміняємо на базову фізику
        if ((dropDistX != dropDistX) || (dropDistY != dropDistY)) {
            dropDistX = real_vx * t_fall * std::exp(-ammo.drag * t_fall / ammo.mass);
            dropDistY = real_vy * t_fall * std::exp(-ammo.drag * t_fall / ammo.mass);
        }
        // 3. МАТЕМАТИЧНА ЕКСТРАПОЛЯЦІЯ ПОЗИЦІЇ ЦІЛІ (predictedTarget)
        static float last_target_x = target.pos.x;
        static float last_target_y = target.pos.y;
        float target_vx = 0.0f;
        float target_vy = 0.0f;

        if (dt > 0.001f) {
            target_vx = (target.pos.x - last_target_x) / dt;
            target_vy = (target.pos.y - last_target_y) / dt;
        }
        last_target_x = target.pos.x;
        last_target_y = target.pos.y;

        SimStep currentStep;
        currentStep.pos       = telemetry.pos;
        currentStep.direction = currentDir;
        currentStep.targetIdx = 0;
        // Прогнозована позиція цілі в момент приземлення бомби (extrapolateTarget логіка)
        //Coord predictedTarget;
        currentStep.predictedTarget.x = target.pos.x + target_vx * t_fall;
        currentStep.predictedTarget.y = target.pos.y + target_vy * t_fall;

        // Aimpoint tочка, куди приземлиться боєприпас, якщо скинути зараз
        currentStep.aimPoint.x = telemetry.pos.x + dropDistX;
        currentStep.aimPoint.y = telemetry.pos.y + dropDistY;

        // Промах розраховується між точкою влучання та ПРОГНОЗОВАНОЮ позицією цілі
        //float predicted_miss = std::hypot(currentStep.predictedTarget.x - impact_x, predictedTarget.y - impact_y);

        // Нормалізація кута в межах [-PI..PI]
        //if (angle_error > M_PI)  angle_error -= 2.0f * M_PI;
        //if (angle_error < -M_PI) angle_error += 2.0f * M_PI;

        // Фіксуємо прогнозований промах точки влучання відносно цілі
        float predicted_miss = std::hypot(currentStep.predictedTarget.x - currentStep.aimPoint.x, 
                                          currentStep.predictedTarget.y - currentStep.aimPoint.y);

        // 4. АЛГОРИТМ НАВЕДЕННЯ (Кут на випереджену точку)
        float desiredDir = std::atan2(currentStep.predictedTarget.y - telemetry.pos.y, 
                                      currentStep.predictedTarget.x - telemetry.pos.x);
                                      
        float raw_angle_error = desiredDir - currentDir;
        float angle_error = std::atan2(std::sin(raw_angle_error), std::cos(raw_angle_error));
        
        // 5. ТРИГЕР АВТОМАТИЧНОГО СКИДАННЯ
        float hitRadius = 2.8f; 
        if (predicted_miss <= hitRadius && std::abs(angle_error) < 0.25f && !m_droneLink->isDropped()) {
            std::cout << "!!! ЦІЛЬ В ЗОНІ УРАЖЕННЯ! Виконується фізичний скид. !!!" << std::endl;
            m_droneLink->triggerDrop(); 
        }

        // 6. КЕРУВАННЯ КУРСОМ ДРОНА
        static float last_angle_error = angle_error;
        float angle_derivative = angle_error - last_angle_error;
        last_angle_error = angle_error; 
        float turnRate = 3.5f* angle_error + 9.5f * angle_derivative;
        turnRate = std::clamp(turnRate, -1.0f, 1.0f);

        // ИСПОЛЬЗУЕМ ОФИЦИАЛЬНЫЙ ENUM ПОСЛЕ ОБНОВЛЕНИЯ COMMON.HPP
        static DroneState current_state = MOVING;
        float error_deg = std::abs(angle_error) * (180.0f / M_PI);
        if (m_droneLink->isDropped()) {
            current_state = DROPPED;
        }
        switch (current_state) {
            case MOVING:
                if (error_deg > 25.0f) {
                    current_state = TURNING;
                }
                break;

            case TURNING:
                if (error_deg <= 15.0f) {
                    current_state = ACCELERATING;        
                }
                break;

            case ACCELERATING:
                if (speed_total >= 8.5f) {
                    current_state = MOVING;
                }
                if (error_deg > 25.0f) {
                    current_state = TURNING;
                }
                break;

            case STOPPED:
                current_state = TURNING;
                break;

            case DROPPED:
                turnRate = 0.0f;
                break;
        }
        
        // Записуємо фінальний стан у структуру кроку
        currentStep.state = (int)current_state;

        // Розраховуємо dropPoint (точка, куди дрон тримає курс)
        currentStep.dropPoint.x = currentStep.predictedTarget.x - dropDistX;
        currentStep.dropPoint.y = currentStep.predictedTarget.y - dropDistY;

        // Збираємо команду відправки
        DroneCommand cmd;
        cmd.angelSpeed = turnRate;
        cmd.state = currentStep.state;
        
        // ДИНАМІЧНЕ ГАЛЬМУВАННЯ:
        // Якщо помилка курсу більша за ~45 градусів (0.8 рад), 
        // вимикаємо маршовий двигун (state = 0), щоб дрон розвертався ефективніше
        //if (std::abs(angle_error) > 0.8f) {
            //cmd.state = 0; // Гальмуємо/крутимося на місці
        //} else {
            //cmd.state = 1; // Повний вперед, якщо ніс дивиться на ціль
        //}
        std::string state_str = "UNKNOWN";
        if (cmd.state == 0) state_str = "STOPPED";
        if (cmd.state == 1) state_str = "ACCELERATING";
        if (cmd.state == 2) state_str = "MOVING";
        if (cmd.state == 3) state_str = "TURNING";
        if (cmd.state == 4) state_str = "DROPPED";

        // Виведення логів у консоль (використовуємо змінні суворо з нашого scope)
        std::cout << "[SOLVER CHECK] t_fall=" << t_fall 
                  << " | dropDistX=" << dropDistX << " | dropDistY=" << dropDistY 
                  << " | impact_x=" << currentStep.aimPoint.x << " | speed.x=" << telemetry.speed.x << std::endl;

        std::cout << "[AI DEBUG] Час: " << telemetry.timeSecSinceStart 
                  << " | Дрон: (" << telemetry.pos.x << ", " << telemetry.pos.y << ")"
                  << " | СТАН: " << state_str << " (" << cmd.state << ")"
                  << " | Промах: " << predicted_miss << " м | Помилка курсу: " << angle_error << std::endl;

        m_droneLink->sendCommand(cmd);

    } // Кінець циклу while (m_keepRunning)
} // Кінець функції run()