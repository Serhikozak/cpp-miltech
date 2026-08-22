#include "DroneLink.h"
#include <fcntl.h>
#include <termios.h>
#include <cmath>
#include <unistd.h>
#include <cstring>
#include <iostream>

DroneLink::DroneLink(const std::string& uartDev, gpiod_line* dropLine)
    : m_dropLine(dropLine), m_uartFd(-1) {

    //Відкриваємо послідовний порт у неблокуючому режимі
    m_uartFd = open(uartDev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(m_uartFd < 0) {
        perror("[DroneLink] Помилка відкриття послідовного порту UART");
        return;
    }

    //Налаштовуємо порт в бінарному режимі
    struct termios tio;
    if (tcgetattr(m_uartFd, &tio) == 0) {
        cfmakeraw(&tio);
        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);
        tio.c_cflag |= (CLOCAL | CREAD);
        tcsetattr(m_uartFd, TCSANOW, &tio);
    }

    m_isReady = true;
}

void DroneLink::run() {
    while (m_keepRunning) {
        int n = read(m_uartFd, &m_buf, sizeof(m_buf));
        if (n > 0) {
            std::lock_guard<std::mutex> lock(m_stateMutex);
            for (int i = 0; i < n; ++i) {
                uint8_t type = 0;
                uint8_t len  = 0;
                uint8_t* payload = m_payloadBuffer;

                if (m_parser.feed(m_buf, type, payload, len)) {
                    if (type == dlink::PKT_TELEMETRY) {
                        std::memcpy(&m_rawTelemetry, payload,sizeof(m_rawTelemetry));
                    }
                    else if (type == dlink::PKT_AMMO) {
                        std::memcpy(&m_rawAmmo, payload,sizeof(m_rawAmmo));
                        m_hasAmmo = true;
                    }
                    else if (type == dlink::PKT_TARGET) {
                        std::memcpy(&m_rawTarget, payload,sizeof(m_rawTarget));
                        m_hasTarget = true;
                    }
                    else if (type == dlink::PKT_CONFIG) {
                        std::memcpy(&m_rawConfig, payload,sizeof(m_rawConfig));
                        m_hasConfig = true;
                    }
                }
                    
            }    
        }
        usleep(1000);
    }
}

//Повертаєммо структуру DroneTelemetry з Common
DroneTelemetry DroneLink::getTelemetry() {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    
    DroneTelemetry telemetry;
    telemetry.pos.x = m_rawTelemetry.x;
    telemetry.pos.y = m_rawTelemetry.y;
    telemetry.speed.x = m_rawTelemetry.z;
    telemetry.speed.y = 0.0f;
        
    return telemetry;
}
//Повертаєммо структуру AmmoParams з Common
AmmoParams DroneLink::getAmmoParams() {
    std::lock_guard<std::mutex> lock(m_stateMutex);

    AmmoParams p;
    if (m_hasAmmo) {
        std::strncpy(p.name, m_rawAmmo.name, 32);
        p.mass = m_rawAmmo.mass;
        p.drag = m_rawAmmo.drag;
        p.lift = m_rawAmmo.lift;
    } else {
        std::memset(p.name, 0, sizeof(p.name));
        p.mass = 0.0f;
        p.drag = 0.0f;
        p.lift = 0.0f;

    }
    return p;
}
//Повертаєммо структуру Target з Common
Target DroneLink::getTarget() {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    
    Target target;
    target.pos.x = m_rawTarget.x;
    target.pos.y = m_rawTarget.y;
    target.velocity.x = m_rawTelemetry.z;
    target.velocity.y = 0.0f;
    
    return target;
}
//Повертаєммо структуру DroneCommand з Common
DroneConfig DroneLink::getConfig() {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    DroneConfig c;
    c.hitRadius = m_hasAmmo ? m_rawAmmo.hitRadius : 5.0f;
    c.turnThreshold = m_hasConfig ? m_rawConfig.turnThreshold : 0.3f;
    return c;

}

// Метод для надсилання структури DroneCommand назад в чекер в бінарному форматі Control
void DroneLink::sendCommand(const DroneCommand& cmd) {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    dlink::Control c;
    c.turnRate = cmd.angelSpeed;

    //Автоматичне пригальмовування на крутих віражах за порогом turnThreshold
    float thresh = m_hasConfig ? m_rawConfig.turnThreshold :0.3f;
    c.accel = (std::abs(cmd.angelSpeed) > thresh) ? -0.3f : 1.0f;

    uint8_t out_buf[512];
    size_t m = dlink::encode(dlink::PKT_CONTROL, &c, sizeof(c), out_buf);
    if (m > 0) {
        write(m_uartFd, out_buf, m);
    }
}

void DroneLink::triggerDrop() {
    if (!m_dropped) {
        gpiod_line_set_value(m_dropLine, 1);
        usleep(80000);
        gpiod_line_set_value(m_dropLine, 0);
        m_dropped = true;
    }
}
    