#include "MissionProcessor.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <cstring>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

MissionProcessor::MissionProcessor(std::shared_ptr<DroneLink> droneLink, 
                                   std::unique_ptr<IBalisticSolver> solver)
    : m_droneLink(droneLink), m_balisticSolver(std::move(solver)) {
    
    m_keepRunning = true;
    
}
// Потік 3: Замкнений контур обчислення автопілота, балістики та наведення ШІ
void MissionProcessor::run() {
    std::cout << "[MissionProcessor] Потік ШІ-наведення запущено." << std::endl;

    float last_x = 0.0f, last_y = 0.0f, last_time = -1.0f;
    float last_target_x = 0.0f, last_target_y = 0.0f;
    bool is_first_run = true;

    float real_vx = 0.0f, real_vy = 0.0f;
    float target_vx = 0.0f, target_vy = 0.0f;

    DroneState current_state = DroneState::MOVING;
    
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
        //DroneConfig config = m_droneLink->getConfig();

        // Захист від відсутності файлу ammo.json
        if (ammo.mass <= 0.001f || std::isnan(ammo.mass)) {
            std::strcpy(ammo.name, "VOG-17");
            ammo.mass = 0.350f;
            ammo.drag = 0.070f;
            ammo.lift = 0.00f;
        }

        float currentZ = 100.0f;
        float dt = telemetry.timeSecSinceStart - last_time;
        // АВТОНОМНИЙ РОЗРАХУНОК ВЕКТОРА ШВИДКОСТІ ЧЕРЕЗ ДЕЛЬТУ КООРДИНАТ ДРОНА
        if (is_first_run || dt <= 0.001f) {
            last_x = telemetry.pos.x;
            last_y = telemetry.pos.y;
            last_target_x = target.pos.x;
            last_target_y = target.pos.y;
            last_time = telemetry.timeSecSinceStart;
            is_first_run = false;

            target_vx = 0.0f; target_vy = 0.0f;
            real_vx = 0.0f; real_vy = 0.0f;
            continue; // Пропускаємо перший крок, бо немає попередніх даних
        }

        // Обчислення реальної швидкості
        real_vx = (telemetry.pos.x - last_x) / dt;
        real_vy = (telemetry.pos.y - last_y) / dt;
        
        float raw_target_vx = (target.pos.x - last_target_x) / dt;
        float raw_target_vy = (target.pos.y - last_target_y) / dt;
        // Зберігаємо мітки поточного такту для наступного кола
        last_x = telemetry.pos.x;
        last_y = telemetry.pos.y;
        last_target_x = target.pos.x;
        last_target_y = target.pos.y;
        last_time = telemetry.timeSecSinceStart;
        // Сатурація (обмеження) швидкості цілі від заскоків при зміні індексів цілей
        if (std::abs(raw_target_vx) > 15.0f) raw_target_vx = target_vx; 
        if (std::abs(raw_target_vy) > 15.0f) raw_target_vy = target_vy;

        //Низькочастотний фільтр (Low-Pass Filter) для згладжування екстраполяції рухомої цілі
        float alpha = 0.15f;
        target_vx = target_vx + alpha * (raw_target_vx - target_vx);
        target_vy = target_vy + alpha * (raw_target_vy - target_vy);

        float speed_total = std::hypot(real_vx, real_vy);
        if (speed_total < 0.2f) {
            speed_total = telemetry.speed.x;
            real_vx = speed_total * 0.707f;
            real_vy = -speed_total * 0.707f;
        }    
        
        float currentDir = std::atan2(real_vy, real_vx);
        
        // --- 4. РОЗРАХУНОК ЧАСУ ПАДІННЯ (Time of Flight) ---
        float t_fall = m_balisticSolver->calcTimeOfFlight(currentZ, speed_total, ammo.mass, ammo.drag, ammo.lift);
        if (std::isnan(t_fall) || std::isinf(t_fall) || t_fall <= 0.1f) {
            t_fall = std::sqrt((2.0f * currentZ) / 9.81f);
        }
        // Розкладаємо однакову швидкість по осях через тригонометрію реального курсу
        //float real_vx = telemetry.speed.x * std::cos(currentDir);
        //float real_vy = telemetry.speed.x * std::sin(currentDir);
        // --- 5. КЛАСИЧНИЙ РОЗРАХУНОК БАЛІСТИКИ ПО ОСЯХ
        float dropDistX = m_balisticSolver->calcHDistance(t_fall, real_vx, ammo.mass, ammo.drag, ammo.lift);
        float dropDistY = m_balisticSolver->calcHDistance(t_fall, real_vy, ammo.mass, ammo.drag, ammo.lift);

          // ЛАГ-КОМПЕНСАЦІЯ: Розраховуємо динамічні пороги випередження по кожній осі окремо.
        // Компенсує системну затримку UART/GPIO буфера у 0.28 секунди, не порушуючи базові змінні логів.
        //float optimal_drop_dist_x = dropDistX + (real_vx * 0.28f);
        //float optimal_drop_dist_y = dropDistY + (real_vy * 0.28f);

        // СТРАХОВКА: Якщо внутрішній Тейлор злетів на нульових швидкостях, підміняємо на базову фізику
        if ((dropDistX != dropDistX) || (dropDistY != dropDistY)) {
            dropDistX = real_vx * t_fall * std::exp(-ammo.drag * t_fall / ammo.mass);
            dropDistY = real_vy * t_fall * std::exp(-ammo.drag * t_fall / ammo.mass);
        }
        // 3. МАТЕМАТИЧНА ЕКСТРАПОЛЯЦІЯ ПОЗИЦІЇ ЦІЛІ (predictedTarget)
        //static float last_target_x = target.pos.x;
        //static float last_target_y = target.pos.y;
        //float target_vx = 0.0f;
        //float target_vy = 0.0f;

        //if (dt > 0.001f) {
            //target_vx = (target.pos.x - last_target_x) / dt;
            //target_vy = (target.pos.y - last_target_y) / dt;
        //}
        //last_target_x = target.pos.x;
        //last_target_y = target.pos.y;

        SimStep currentStep;
        currentStep.pos       = telemetry.pos;
        currentStep.direction = currentDir;
        currentStep.targetIdx = 0;
        // Прогнозована позиція цілі в момент приземлення бомби (extrapolateTarget логіка)
        //Coord predictedTarget;
        currentStep.predictedTarget.x = target.pos.x + target_vx * t_fall;
        currentStep.predictedTarget.y = target.pos.y + target_vy * t_fall;

        // Aimpoint tочка, куди приземлиться боєприпас, якщо скинути зараз
        currentStep.aimPoint.x = telemetry.pos.x + dropDistX + (real_vx * 0.28f);
        currentStep.aimPoint.y = telemetry.pos.y + dropDistY + (real_vy * 0.28f);

        // Промах розраховується між точкою влучання та ПРОГНОЗОВАНОЮ позицією цілі
        //float predicted_miss = std::hypot(currentStep.predictedTarget.x - impact_x, predictedTarget.y - impact_y);

        // Нормалізація кута в межах [-PI..PI]
        //if (angle_error > M_PI)  angle_error -= 2.0f * M_PI;
        //if (angle_error < -M_PI) angle_error += 2.0f * M_PI;

        // Фіксуємо прогнозований промах точки влучання відносно цілі
        float predicted_miss = std::hypot(currentStep.predictedTarget.x - currentStep.aimPoint.x, 
                                          currentStep.predictedTarget.y - currentStep.aimPoint.y);

        // Геометрична відстань від поточної позиції дрона до прогнозованої цілі на землі
        //float geo_distance = std::hypot(currentStep.predictedTarget.x - telemetry.pos.x, 
                                        //currentStep.predictedTarget.y - telemetry.pos.y);

        // 4. АЛГОРИТМ НАВЕДЕННЯ (Кут на випереджену точку)
        float desiredDir = std::atan2(currentStep.predictedTarget.y - telemetry.pos.y, 
                                      currentStep.predictedTarget.x - telemetry.pos.x);
                                      
        float raw_angle_error = desiredDir - currentDir;
        float angle_error = std::atan2(std::sin(raw_angle_error), std::cos(raw_angle_error));
        
        // 5. ТРИГЕР АВТОМАТИЧНОГО СКИДАННЯ
        //float dist_x = std::abs(currentStep.predictedTarget.x - telemetry.pos.x);
        //float dist_y = std::abs(currentStep.predictedTarget.y - telemetry.pos.y);

        float hitRadius = m_droneLink->getConfig().hitRadius;
        
        if (predicted_miss <= (hitRadius + 0.5f) && std::abs(angle_error) < 0.25f) {
            std::cout << "[AI] TRIGGER ACTIVATED! Скид вантажу за балістичним контуром." << std::endl;
            m_droneLink->triggerDrop();
            
        }
        
                                

        
        // 6. КЕРУВАННЯ КУРСОМ ДРОНА
        static float last_angle_error = angle_error;
        float angle_derivative = angle_error - last_angle_error;
        last_angle_error = angle_error; 
        float turnRate = 6.5f* angle_error + 9.5f * angle_derivative;
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