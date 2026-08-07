#include <iostream>
#include "fabrica.h"
#include "MissionProcessor.h"
#include "interfaces/IBalisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"

int main () {
    std::cout << "Start simulator ------" << std::endl;
    //Створюємо прості фабричні функції
    IConfigLoader* loader = createLoader(LoaderType::FILE);
    ITargetProvider* provider = createProvider(ProviderType::JSON, "data/targets.json");
    IBalisticSolver* solver = createSolver(SolverType::ANALYTICAL);

    //Перевірка на випадок якщо якийсь з компонентів не сторився
    if (!loader || !provider || !solver) {
        std::cerr << "Error: failed to create simulation" << std::endl;
        delete loader; delete provider; delete solver;
        return 1;
    }

    MissionProcessor processor(loader, provider, solver);

    //processor.init("config.json");
    processor.init();

    std::cout << "Simulation inicialized" << std::endl;

    //Головний цикл в симуляції
    while (processor.hasNext()) {
        processor.step();
    }
    std::cout << "Simulation finished" << std::endl;

    return 0;

}