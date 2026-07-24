#include "FileConfigLoader.h"
#include <iostream>

void FileConfigLoader::load() {
    //Тут буде логіка відкриття config.json та ammo.json
    std::cout << "Loading config files " << std::endl;
}

DroneConfig FileConfigLoader::getConfig() {
    DroneConfig config;
    //Тут заповнюємо структуру config даними з файлу
    return config;
}

AmmoParams FileConfigLoader::getAmmoParams() {
    AmmoParams params;
    //Тут заповнюємо структуру params даними з файлу
    return params;
}