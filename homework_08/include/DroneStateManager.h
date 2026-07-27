#pragma once
#include "interfaces/Common.hpp"
#include <cmath>

class DroneStateManager {
private:
    float m_x = 0.0f;
    float m_y = 0.0f;
    float m_speed = 0.0f;
    float m_dir = 0.0f;
    DroneState m_state = STOPPED;
    float m_timeToStop = 0.0f;

public:
    DroneStateManager() = default;
    ~DroneStateManager() = default;

    // Ініціалізація початкового стану дрона з конфігу
    void init(const DroneConfig& config) {
        m_x = config.startPos.x;
        m_y = config.startPos.y;
        m_dir = config.initialDir;
        m_speed = 0.0f;
        m_state = STOPPED;
        m_timeToStop = 0.0f;
    }

    // Оновлення фізики та станів автомата за формулами з ТЗ
    void update(const Coord& bestTargetPos, const DroneConfig& config, float dt) {
        float targetDir = std::atan2(bestTargetPos.y - m_y, bestTargetPos.x - m_x);
        
        // Розраховуємо різницю кутів повороту
        float angleDiff = std::abs(targetDir - m_dir);
        if (angleDiff > M_PI) angleDiff = 2 * M_PI - angleDiff;

        // Перевірка необхідності зміни стану на розворот/гальмування
        if (angleDiff > config.turnThreshold && m_state != TURNING) {
            m_state = TURNING;
            m_timeToStop = angleDiff / config.angularSpeed; 
        }

        // Обробка поточної активної фази автомата станів
        if (m_state == TURNING) {
            m_speed = 0.0f; // під час розвороту швидкість падає
            m_timeToStop -= dt;

           // Плавно повертаємо кут у бік цілі
            if (targetDir > m_dir) m_dir += config.angularSpeed * dt;
            else m_dir -= config.angularSpeed * dt;

            if (m_timeToStop <= 0.0f) {
                m_dir = targetDir; // Розворот завершено
                m_state = ACCELERATING;
                m_timeToStop = config.attackSpeed / config.accelPath; // час до повного розгону
            }
        }

        else if (m_state == STOPPED || m_state == ACCELERATING) {
            m_state = ACCELERATING;
            // Прискорення (імітація лінійного зростання швидкості)
            m_speed += (config.attackSpeed / config.accelPath) * dt;
            if (m_speed >= config.attackSpeed) {
                m_speed = config.attackSpeed;
                m_state = MOVING;
            }
            m_dir = targetDir;
        }
        else if (m_state == MOVING) {
            m_speed = config.attackSpeed;
            m_dir = targetDir;
        }

        // Оновлюємо поточні координати на основі проекцій вектора швидкості
        m_x += m_speed * std::cos(m_dir) * dt;
        m_y += m_speed * std::sin(m_dir) * dt;
    }
    // Геттери для передачі параметрів в історію симуляції
    float getX() const { return m_x; }
    float getY() const { return m_y; }
    float getDir() const { return m_dir; }
    DroneState getState() const { return m_state; }
    
    // Метод для примусової зміни стану (наприклад, при скиданні боєприпасу)
    void forceState(DroneState s) { m_state = s; }
};

        
        

