#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <iomanip>

#define MPU6050_ADDR 0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_WHO_AM_I 0x75
#define REG_START_DATA 0x3B  // Початок блоку даних (Accel X H)

int main(int argc, char* argv[]) {
    const char* bus = (argc > 1) ? argv[1] : "/dev/i2c-1";
    
    // 1. Відкриття І2С-шини
    int fd = open(bus, O_RDWR);
    if (fd < 0) {
        std::cerr << "[ПОМИЛКА] Не вдалося відкрити I2C шину: " << bus << std::endl;
        return 1;
    }

    // 2. Вибір адреси пристрою
    if (ioctl(fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
        std::cerr << "[ПОМИЛКА] Пристрій з адресою 0x68 не знайдено (Немає ACK)!" << std::endl;
        close(fd);
        return 1;
    }

    // 3. Перевірка ID пристрою (WHO_AM_I)
    uint8_t reg_who = REG_WHO_AM_I;
    uint8_t id_val = 0;
    if (write(fd, &reg_who, 1) != 1 || read(fd, &id_val, 1) != 1) {
        std::cerr << "[ПОМИЛКА] Обрив зв'язку при спробі зчитати WHO_AM_I!" << std::endl;
        close(fd);
        return 1;
    }

    if (id_val != 0x68) {
        std::cerr << "[ПОМИЛКА] Невірний ID пристрою! Очікувалось 0x68, отримано: 0x" 
                  << std::hex << (int)id_val << std::dec << std::endl;
        close(fd);
        return 1;
    }

    // 4. Ініціалізація датчика
    uint8_t wake_cmd[] = {REG_PWR_MGMT_1, 0x00};
    if (write(fd, wake_cmd, 2) != 2) {
        std::cerr << "[ПОМИЛКА] Не вдалося записати в регістр живлення PWR_MGMT_1!" << std::endl;
        close(fd);
        return 1;
    }
    usleep(100000); // 100 мс на стабілізацію

    std::cout << "[УСПІХ] Датчик MPU-6050 ініціалізовано. Починаємо зчитування..." << std::endl;

    // 5. Цикл виведення даних (кілька разів на секунду)
    while (true) {
        uint8_t reg_start = REG_START_DATA;
        uint8_t buffer[14] = {0}; // Блок із 14 байт під усі сенсори

        // Двофазне читання блоку регістрів підряд
        if (write(fd, &reg_start, 1) != 1) {
            std::cerr << "\n[ПОМИЛКА] Обрив запису при запиті блоку даних!" << std::endl;
            usleep(500000);
            continue;
        }

        if (read(fd, buffer, 14) != 14) {
            std::cerr << "\n[ПОМИЛКА] Обрив читання! Отримано менше 14 байт." << std::endl;
            usleep(500000);
            continue;
        }

        // Збирання 16-бітних значень (Порядок: Старший байт перший - Big Endian)
        int16_t raw_accel_x = (buffer[0] << 8) | buffer[1];
        int16_t raw_accel_y = (buffer[2] << 8) | buffer[3];
        int16_t raw_accel_z = (buffer[4] << 8) | buffer[5];
         int16_t raw_temp    = (buffer[6] << 8) | buffer[7];
        
        int16_t raw_gyro_x  = (buffer[8] << 8) | buffer[9];
        int16_t raw_gyro_y  = (buffer[10] << 8) | buffer[11];
        int16_t raw_gyro_z  = (buffer[12] << 8) | buffer[13];

        // 6. Перерахунок у фізичні величини за даташитом
        // Шкали за замовчуванням: Accel = ±2g (16384 LSB/g), Gyro = ±250 °/s (131 LSB/°/s)
        double ax = (double)raw_accel_x / 16384.0;
        double ay = (double)raw_accel_y / 16384.0;
        double az = (double)raw_accel_z / 16384.0;

        // Формула температури з даташиту MPU-6050
        double temp_c = ((double)raw_temp / 340.0) + 36.53;

        double gx = (double)raw_gyro_x / 131.0;
        double gy = (double)raw_gyro_y / 131.0;
        double gz = (double)raw_gyro_z / 131.0;

        // Людиночитане виведення в один рядок
        std::cout << "\r" << std::fixed << std::setprecision(2)
                  << "ACCEL: [" << ax << ", " << ay << ", " << az << "] g | "
                  << "GYRO: [" << gx << ", " << gy << ", " << gz << "] °/s | "
                  << "TEMP: " << temp_c << " °C" << std::flush;

        usleep(200000); // Оновлення кожні 200 мс (~5 разів на секунду)
    }

    close(fd);
    return 0;
}