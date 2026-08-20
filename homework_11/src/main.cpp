#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IBalisticSolver.h"
#include "FileConfigLoader.h"
#include "JsonTargetProvider.h"
#include "DronePhysics.h"
#include "MissionProcessor.h"
#include "fabrica.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    std::cout << "Початок роботи симулятора -----\n";

    // 1. Створюємо лоадер і завантажуємо конфігураційні файли в однопоточному режимі на старті
    IConfigLoader* loader = createLoader(LoaderType::FILE);
    if (!loader) {
        std::cerr << "Error: failed to create Config Loader\n";
        return 1;
    }

    loader->load();
    DroneConfig config = loader->getConfig();

    // 2. Створюємо інші компоненти системи через фабрику та конструктори
    // Для провайдера цілей передаємо шлях до файлу та крок часу з конфігурації
    ITargetProvider* provider = createProvider(ProviderType::JSON, "data/targets.json", config.arrayTimeStep);
    IBalisticSolver* solver = createSolver(SolverType::ANALYTICAL);
    DronePhysics* physics = new DronePhysics(config);
    
    // Створюємо головний процесор місії, передаючи йому всі залежності
    MissionProcessor* mission = new MissionProcessor(loader, provider, solver, physics);

    // Перевіряємо, чи всі компоненти успішно виділили пам'ять
    if (!provider || !solver || !physics || !mission) {
        std::cerr << "Error: failed to initialize simulation components\n";
        delete loader;
        delete provider;
        delete solver;
        delete physics;
        delete mission;
        return 1;
    }

    // 3. Створюємо та запускаємо 3 зовнішні потоки 
    // Передаємо адресу методу класу run та вказівник на відповідний об'єкт
    std::thread providerThread(&JsonTargetProvider::run, static_cast<JsonTargetProvider*>(provider), config.targetTimeStep, config.timeScale);
    std::thread physicsThread(&DronePhysics::run, physics);
    std::thread missionThread(&MissionProcessor::run, mission);

    std::cout << "Threads allocated. Waiting for threads initialization...\n";

    // 4. Очікування повної готовності потоків
    // Потоки виконують стартові налаштування і піднімають прапорець m_isReady = true
    while (provider && !static_cast<JsonTargetProvider*>(provider)->isThreadReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    while (!physics->isThreadReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    while (!mission->isThreadReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "All background threads are ready. Starting simulation...\n";

    // 5. Одночасна активація робочих циклів 
    // Прапорці m_keepRunning стають true, і потоки починають виконувати логіку в реальному часі
    if (provider) static_cast<JsonTargetProvider*>(provider)->start();
    physics->start();
    mission->start();

    std::cout << "Simulation loop is active. Main thread sleeping on mission join.\n";

    // 6. Очікування завершення головного потоку планувальника місії
    // Коли дрон влучить у радіус цілі або виконає місію, потік missionThread фінішує сам
    if (missionThread.joinable()) {
        missionThread.join();
    }

    std::cout << "Mission finished. Stopping background calculation threads...\n";

    // 7. Послідовна зупинка фонових потоків (Опускаємо атомарні прапорці)
    physics->stop();
    if (provider) static_cast<JsonTargetProvider*>(provider)->stop();

    std::cout << "Zipping threads together (join)...\n";

    // 8. Зшивання фонових потоків для безпечного завершення програми
    if (physicsThread.joinable()) {
        physicsThread.join();
    }
    if (providerThread.joinable()) {
        providerThread.join();
    }

    std::cout << "Cleaning dynamic memory to prevent leaks...\n";

    // 9. Безпечне очищення динамічної пам'яті (Memory Leak Protection)
    delete loader;
    delete provider;
    delete solver;
    delete physics;
    delete mission;

    std::cout << "Simulation successfully finished -----\n";
    return 0;
}