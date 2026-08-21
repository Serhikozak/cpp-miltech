#include "AnalyticalSolver.h"
#include "DroneLink.h"
#include "MissionProcessor.h"
#include <iostream>
#include <ostream>
#include <thread>
#include <string>
#include <memory>
#include <unistd.h>
#include <gpiod.h>

int main(int argc, char* argv[]) {
    // Налаштування за замовчуванням відповідно до тест-стенду симулятора
    std::string uart_dev = "/tmp/ttyA";
    std::string chip_name = "gpiochip1";
    int start_line = 24;
    int drop_line  = 23;

    for (int i = 1; i < argc; i++) {
        if (std::string(argv[1]) == "-uart" && i + 1 < argc) {
            uart_dev = argv[i + 1];
            i++;        
        } 
        else if (std::string(argv[1]) == "--gpiochip" && i + 1 < argc) {
            chip_name = argv[i + 1];
            i++;        
        } 
        else if (std::string(argv[1]) == "--start-line" && i + 1 < argc) start_line = std::stoi(argv[++1]);
        else if (std::string(argv[1]) == "--drop-line" && i + 1 < argc) drop_line = std::stoi(argv[++1]);
    }
    std::cout << "[Система] Запуск ООП автопілота ДЗ 11... \n" << std::endl;

    // Ініціалізація роботи із залізом (libgpiod)
    gpiod_chip* chip = gpiod_chip_open_by_name(chip_name.c_str());
    if (!chip) {
        std::cerr << "Error: failed to open GPIO chip \n" << chip_name << std::endl ;
        return -1;
    }

    gpiod_line* start = gpiod_chip_get_line(chip, start_line);
    gpiod_line* drop = gpiod_chip_get_line(chip, drop_line);

    gpiod_line_request_output(start, "autopilot", 0);
    gpiod_line_request_output(drop, "autopilot", 0);

    // Створюємо інтерфейс зв'язку DroneLink, ховаючи всередину налаштування termios
    auto solver = std::make_unique<AnalyticalSolver>();

    // Передаємо залежності до головного процесора місії
    MissionProcessor processor(droneLink, std::move(solver));

    // СИГНАЛ ГОТОВНОСТІ: Піднімаємо START в 1, щоб чекер запустив симуляцію польоту дрона
    gpiod_line_set_value(start, 1);
    std::cout << "[GPIOD] Лінію START успішно активовано " << std::endl;

    
    // Запускаємо фоновий Потік 1 для вичитування бінарного протоколу UART
    droneLink -> start();
    std::thread linkThread(&DroneLink::run, droneLink::get());

    processor.run();

    droneLink -> stop();
    if (linkThread.joinable()) {
        linkThread.join();        
    }

    gpiod_chip_close(chip);
    std:: cout << "[Система] Контур успішно зупинено. Роботу завершено." << std::endl;
    return 0;
}