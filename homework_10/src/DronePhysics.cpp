#include "DronePhysics.h"
#include <chrono>
#include <thread>
#include <cmath>

DroneTelemetry DronePhysics::getTelemetry() {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    
    DroneTelemetry telemetry;
    telemetry.pos = m_pos;
    telemetry.speed = m_speed;
    telemetry.timeSecSinceStart = m_timeSecSinceStart;
    
    return telemetry;
}

// Конструктор: ініціалізуємо початкові координати та параметри з конфігу
DronePhysics::DronePhysics(const DroneConfig& config) : m_config(config) {
    m_pos = config.startPos;
    m_dir = config.initialDir;
    m_physicsTimeStep = config.physicsTimeStep; 
    m_timeScale = config.timeScale;             
}

// Метод для надсилання команд із MissionProcessor напряму в пам'ять фізики
void DronePhysics::sendCommand(const DroneCommand& cmd) {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    m_state = cmd.state;
    m_angledSpeed = cmd.angelSpeed; // Записуємо кут курсу від ШІ напрямую в фізику
}

// Головний цикл потоку фізики (Потік 2)
      
void DronePhysics::run() {
    m_isReady = true; // 1. Сигналізуємо main(), що потік ініціалізовано

    // 2. Очікуємо в циклі, поки main() не викличе метод physics->start()
    while (!m_keepRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    m_linearSpeedMag = 0.0f; // Початкова величина швидкості

    // 3. Активна фаза роботи потоку
    while (m_keepRunning) {
        
        // Обчислюємо фізику руху Ейлера під захистом м'ютексу стану
        {
        std::lock_guard<std::mutex> lock(m_stateMutex);

        m_dir += m_angledSpeed * m_physicsTimeStep;
                        
            // Обробляємо величину лінійної швидкості для всіх станів
            if (m_state == 3) { // TURNING
                m_linearSpeedMag = 0.0f; 
            }
            else if (m_state == 1) { // ACCELERATING
                // Захист від скидання: якщо швидкість ще не максимальна — нарощуємо її
                if (m_linearSpeedMag < m_config.attackSpeed) {
                    m_linearSpeedMag += (m_config.attackSpeed / m_config.accelPath) * m_physicsTimeStep;
                }
                if (m_linearSpeedMag >= m_config.attackSpeed) {
                    m_linearSpeedMag = m_config.attackSpeed;
                }
            }
            else if (m_state == 2 || m_state == 4) { // MOVING або DROPPED
                m_linearSpeedMag = m_config.attackSpeed; 
            }
            else if (m_state == 0) { // STOPPED
                m_linearSpeedMag = 0.0f;
            }

            // Проєктуємо лінійна швидкість на оновлений кут m_dir
            m_speed.x = m_linearSpeedMag * std::cos(m_dir);
            m_speed.y = m_linearSpeedMag * std::sin(m_dir);

            // Інтегруємо поточні координати за Ейлером
            m_pos.x += m_speed.x * m_physicsTimeStep;
             m_pos.y += m_speed.y * m_physicsTimeStep;

            m_timeSecSinceStart += m_physicsTimeStep;
        } // Закриваємо блок lock

        // Дробовий сон із масштабуванням часу
        std::this_thread::sleep_for(std::chrono::duration<float>(m_physicsTimeStep / m_timeScale));
    } // Закриваємо цикл while (m_keepRunning)
} // Закриваємо метод run()