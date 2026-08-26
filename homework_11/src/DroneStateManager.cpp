#include "../include/DroneStateManager.h"
#include <cmath>

DroneCommand DroneStateManager::update(const DroneTelemetry& telemetry, const Coord& bestTargetPos, const DroneConfig& config, float dt) {
    // 1. Рахуємо кут на ціль від поточної позиції дрона з телеметрії
    float targetDir = std::atan2(
        static_cast<float>(bestTargetPos.y) - static_cast<float>(telemetry.pos.y),
        static_cast<float>(bestTargetPos.x) - static_cast<float>(telemetry.pos.x)        
    );
    
    // Якщо дрон лежить у дрейфі (швидкість 0) — беремо початковий кут. Інакше — за вектором швидкості.
    //float currentDir = std::atan2(telemetry.speed.y, telemetry.speed.x); 
    //if (telemetry.speed.x == 0.0f && telemetry.speed.y == 0.0f) {
        //currentDir = config.initialDir; 
    //}
    // Замість розрахунку через швидкість, беремо наш збережений внутрішній курс ШІ!
    float currentDir = m_currentDir; 

    float angleDiff = std::abs(targetDir - currentDir);
    if (angleDiff > M_PI) {
        angleDiff = 2 * M_PI - angleDiff;
    }

    float angelSpeedCommand = 0.0f;

    // 2. Перевірка необхідності переходу на розворот
    if (angleDiff > config.turnThreshold && m_state != DroneState::TURNING && m_state != DroneState::DROPPED) {
        m_state = DroneState::TURNING;
        //m_timeToStop = (angleDiff / config.angularSpeed) / config.timeScale;
    }

    // 3. Обробка 5 станів автомата
    switch (m_state) {
        case DroneState::TURNING: {
            // Передаємо точний цільовий кут через angelSpeed у стані розвороту
            //angelSpeedCommand = targetDir; 
            
            //m_timeToStop -= dt;

            // Визначаємо найкоротший бік для повороту
            //m_state = DroneState::ACCELERATING;
            float diff = targetDir - currentDir;
            if (diff < -M_PI) diff += 2 * M_PI;
            if (diff > M_PI) diff -= 2 * M_PI;
            
            angelSpeedCommand = (diff > 0.0f ? 1.0f : -1.0f) * config.angularSpeed;

            // 🔥 ШІ чесно і плавно оновлює свій курс відповідно до кутової швидкості:
            m_currentDir += angelSpeedCommand * dt;

            if (m_currentDir < -M_PI) m_currentDir += 2 * M_PI;
            if (m_currentDir < M_PI) m_currentDir -= 2 * M_PI;
            

            if (angleDiff <= config.turnThreshold) {
                m_currentDir = targetDir;
                m_state = DroneState::ACCELERATING;
                angelSpeedCommand = targetDir;
            }
            break;
        }

        case DroneState::STOPPED:
            m_state = DroneState::ACCELERATING;
            break;
        
        case DroneState::ACCELERATING: {
            // ЧEСНO: Якщо кут на ціль змінився (дрон пролетів повз або ціль змістилася),
            // автомат станів чесно повертає дрон у режим розвороту TURNING!
            //if (angleDiff > config.turnThreshold) {
                //m_state = DroneState::TURNING;

                //float diff = targetDir - currentDir;
                //while (diff < -M_PI) diff += 2 * M_PI;
                //while (diff < M_PI) diff -= 2 * M_PI;
                //angelSpeedCommand = (diff > 0.0f ? 1.0f : -1.0f) * config.angularSpeed;
                //break;
            //}

            float currentSpeedMag = std::hypot(telemetry.speed.x, telemetry.speed.y);
            if (currentSpeedMag >= config.attackSpeed) {
                m_state = DroneState::MOVING; 
            }
            if (angleDiff > config.turnThreshold) {
                m_state = DroneState::TURNING;
            }
            break;
        }

        case DroneState::MOVING: {
            // ЧEСНO: Постійно стежимо за кутом під час польоту.
            // Якщо дельта кутів перевищує turnThreshold — чесно вмикаємо корекцію курсу!
            if (angleDiff > config.turnThreshold) {
                m_state = DroneState::TURNING;
                //float diff = targetDir - currentDir;
                //while (diff < -M_PI) diff += 2 * M_PI;
                //while (diff < M_PI) diff -= 2 * M_PI;
                //angelSpeedCommand = (diff > 0.0f ? 1.0f : -1.0f) * config.angularSpeed;
            }
            break;
        }
           
        case DroneState::DROPPED:
            angelSpeedCommand = 0.0f;
            break;
    }

    // 4. Пакуємо команду для відправки у фізичний потік
    DroneCommand cmd;
    cmd.state = static_cast<int>(m_state);
    cmd.angelSpeed = angelSpeedCommand;
    
    return cmd;
}