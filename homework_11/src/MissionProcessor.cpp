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

    //float real_vx = 0.0f, real_vy = 0.0f;
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
            //real_vx = 0.0f; real_vy = 0.0f;
            continue; // Пропускаємо перший крок, бо немає попередніх даних
        }

        // Обчислення реальної швидкості
        //real_vx = (telemetry.pos.x - last_x) / dt;
        //real_vy = (telemetry.pos.y - last_y) / dt;
        
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

        // 2. ВЕКТОРНИЙ РОЗРАХУНОК ШВИДКОСТІ ДРОНА
        // Використовуємо структуру Coord та функцію length() з Common
        Coord real_move_vec = telemetry.pos - Coord{last_x, last_y};
        float distance_moved = length(real_move_vec);

        float speed_total = 0.0f;
        Coord move_dir;
        if (distance_moved > 0.005f && dt > 0.001f) {
            speed_total = distance_moved / dt;
            move_dir = normalize(real_move_vec);
        } else {
            // Якщо дрон стоїть на місці, використовуємо напрямок до цілі
            //float startDir = std::atan2(target.pos.y - telemetry.pos.y, target.pos.x - telemetry.pos.x);
            //move_dir = Coord{std::cos(startDir), std::sin(startDir)};
        
        //float startDir = std::atan2(target.pos.y - telemetry.pos.y, target.pos.x - telemetry.pos.x);
    
            speed_total = length(telemetry.speed);
            if (speed_total < 0.3f) {
                speed_total = 0.5f;
                float startDir = std::atan2(target.pos.y - telemetry.pos.y, target.pos.x - telemetry.pos.x);
                move_dir = Coord{std::cos(startDir), std::sin(startDir) };
                //real_vy = speed_total * std::sin(startDir);
            } else {
                move_dir = normalize(telemetry.speed);
                }
        }
        
        if (speed_total > 11.0f) speed_total = 10.0f; // Обмеження максимальної швидкості дрона
        
        //float currentDir = std::atan2(real_vy, real_vx);
        // Одиничний вектор напрямку руху літального апарату
        //Coord move_Dir = normalize(telemetry.speed);
        //if (length(move_Dir) < 0.1f) {
            
            //move_Dir = Coord{std::cos(startDir), std::sin(startDir)};
        //}
        
        // --- 4. РОЗРАХУНОК ЧАСУ ПАДІННЯ (Time of Flight) ---
        float t_fall = m_balisticSolver->calcTimeOfFlight(currentZ, speed_total, ammo.mass, ammo.drag, ammo.lift);
        if (std::isnan(t_fall) || std::isinf(t_fall) || t_fall <= 0.1f) {
            float k_over_m = ammo.drag / ammo.mass;
           // float v_term = 9.81f / k_over_m; // Термічна швидкість падіння
            t_fall = std::sqrt((2.0f * currentZ) / 9.81f) * (1.0f + (k_over_m * std::sqrt((2.0f *currentZ) / 9.81f) / 6.0f)); // Корекція на опір повітря

        }
        // Розкладаємо однакову швидкість по осях через тригонометрію реального курсу
        //float real_vx = telemetry.speed.x * std::cos(currentDir);
        //float real_vy = telemetry.speed.x * std::sin(currentDir);
        // --- 5. КЛАСИЧНИЙ РОЗРАХУНОК БАЛІСТИКИ ПО ОСЯХ
        float dropDistX = m_balisticSolver->calcHDistance(t_fall, speed_total, ammo.mass, ammo.drag, ammo.lift);
        //float dropDistY = m_balisticSolver->calcHDistance(t_fall, real_vy, ammo.mass, ammo.drag, ammo.lift);
        Coord drop_offset = move_dir * dropDistX;
        //float k = ammo.drag;
        //if (k < 0.001f) k = 0.070f; // Защита от нульового опору

          // ЛАГ-КОМПЕНСАЦІЯ: Розраховуємо динамічні пороги випередження по кожній осі окремо.
        // Компенсує системну затримку UART/GPIO буфера у 0.28 секунди, не порушуючи базові змінні логів.
        //float optimal_drop_dist_x = dropDistX + (real_vx * 0.28f);
        //float optimal_drop_dist_y = dropDistY + (real_vy * 0.28f);

        // СТРАХОВКА: Якщо внутрішній Тейлор злетів на нульових швидкостях, підміняємо на базову фізику
        //if (std::isnan(dropDistX) || std::isnan(dropDistY) || std::abs(dropDistX) < 0.01f) {
            //dropDistX = (real_vx * ammo.mass / k) * (1.0f - std::exp(-k * t_fall / ammo.mass));
            //dropDistY = (real_vy * ammo.mass / k) * (1.0f - std::exp(-k * t_fall / ammo.mass));
        //}
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
        currentStep.direction = std::atan2(move_dir.y, move_dir.x);
        currentStep.targetIdx = 0;
        // Прогнозована позиція цілі в момент приземлення бомби (extrapolateTarget логіка)
        //Coord predictedTarget;

        // Екстраполяція руху цілі на час падіння вантажу (Прогнозована точка зустрічі)
        Coord target_velocity_vec = Coord{ target_vx, target_vy };
        currentStep.predictedTarget = target.pos + (target_velocity_vec * t_fall);

        
        // Aimpoint tочка, куди приземлиться боєприпас, якщо скинути зараз  
        currentStep.aimPoint = telemetry.pos + drop_offset + (move_dir * (speed_total * 0.28f)); 
        //currentStep.aimPoint.y = telemetry.pos.y + dropDistY + (real_vy * 0.28f);

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
        Coord to_target = currentStep.predictedTarget - telemetry.pos;
        float desiredDir = std::atan2(to_target.y, to_target.x);
        float currentDir = std::atan2(move_dir.y, move_dir.x); 
                                      
                                      
        float raw_angle_error = desiredDir - currentDir;
        float angle_error = std::atan2(std::sin(raw_angle_error), std::cos(raw_angle_error));
        float error_deg = std::abs(angle_error) * (180.0f / M_PI);
        // 5. ТРИГЕР АВТОМАТИЧНОГО СКИДАННЯ
        //float dist_x = std::abs(currentStep.predictedTarget.x - telemetry.pos.x);
        //float dist_y = std::abs(currentStep.predictedTarget.y - telemetry.pos.y);

        float hitRadius = m_droneLink->getConfig().hitRadius;
        
        if (!m_droneLink->isDropped() && predicted_miss <= (hitRadius * 0.4f) && std::abs(angle_error) < 0.10f) {
            std::cout << "[AI] TRIGGER ACTIVATED! Скид вантажу за балістичним контуром hitRadius: " << predicted_miss << "м" << std::endl;
            m_droneLink->triggerDrop();
            
        }
        
                                

        
        // 6. КЕРУВАННЯ КУРСОМ ДРОНА
        static float last_angle_error = angle_error;
        float safe_dt = (dt > 0.001f) ? dt : 0.02f;
        float angle_derivative = (angle_error - last_angle_error) / safe_dt;
        last_angle_error = angle_error; 
        float turnRate = 4.5f* angle_error + 0.15f * angle_derivative;
        turnRate = std::clamp(turnRate, -0.6f, 0.6f);

        // ИСПОЛЬЗУЕМ ОФИЦИАЛЬНЫЙ ENUM ПОСЛЕ ОБНОВЛЕНИЯ COMMON.HPP
       //static DroneState current_state = MOVING;
        
        if (m_droneLink->isDropped()) {
            current_state = DroneState::DROPPED;
            
        }
        switch (current_state) {
            case DroneState::MOVING:
                if (error_deg > 75.0f) {
                    current_state = DroneState::TURNING;
                }
                break;

            case DroneState::TURNING:
                if (error_deg <= 15.0f) {
                    current_state = DroneState::ACCELERATING;        
                }
                break;

            case DroneState::ACCELERATING:
                if (speed_total >= 5.0f) {
                    current_state = DroneState::MOVING;
                }
                if (error_deg > 75.0f) {
                    current_state = DroneState::TURNING;
                }
                break;

            case DroneState::STOPPED:
                current_state = DroneState::TURNING;
                break;

            case DroneState::DROPPED:
                turnRate = 0.0f;
                break;
        }
        
        // Записуємо фінальний стан у структуру кроку
        currentStep.state = static_cast<int>(current_state);

        // Розраховуємо dropPoint (точка, куди дрон тримає курс)
        currentStep.dropPoint = currentStep.predictedTarget - drop_offset; 
              

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
                  << " | dropDist=" << dropDistX   
                  << " | impact_x=" << currentStep.aimPoint.x
                  << " | impact_y=" << currentStep.aimPoint.y
                  << " | speed.x=" << speed_total << std::endl;

        std::cout << "[AI DEBUG] Час: " << telemetry.timeSecSinceStart 
                  << " | Дрон: (" << telemetry.pos.x << ", " << telemetry.pos.y << ")"
                  << " | СТАН: " << state_str << " (" << cmd.state << ")"
                  << " | Промах: " << predicted_miss << " м | Помилка курсу: " << angle_error << std::endl;

        m_droneLink->sendCommand(cmd);

    } // Кінець циклу while (m_keepRunning)
} // Кінець функції run()