#include "FileConfigLoader.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

void FileConfigLoader::load() {
    //Тут буде логіка відкриття config.json та ammo.json
    std::cout << "Loading config files " << std::endl;

    //=====================================
    //  Читання config.json
    //=====================================
        std::ifstream fin("data/config.json");
        if (!fin.is_open()) { std::cerr << "Cannot open config.json" << std::endl; return; }
        json j;
        fin >> j;
        fin.close();

        m_config.startPos.x    = j["drone"]["position"]["x"];
        m_config.startPos.y    = j["drone"]["position"]["y"];
        m_config.altitude      = j["drone"]["altitude"];
        m_config.initialDir    = j["drone"]["initialDirection"];
        m_config.attackSpeed   = j["drone"]["attackSpeed"];
        m_config.accelPath     = j["drone"]["accelerationPath"];
        m_config.angularSpeed  = j["drone"]["angularSpeed"];
        m_config.turnThreshold = j["drone"]["turnThreshold"];

        std::strncpy(m_config.ammoName, j["ammo"].get<std::string>().c_str(), 31);
        m_config.ammoName[31] = '\0';

        //m_config.simTimeStep   = j.value("timeStep", 0.05f);
        //m_config.hitRadius     = j.value("hitRadius", 2.0f);
        //m_config.arrayTimeStep = j.value("targetArrayTimeStep", 0.1f);

        m_config.simTimeStep   = j.value(json::json_pointer("/simulation/timeStep"), 0.05f);
        m_config.hitRadius     = j.value(json::json_pointer("/simulation/hitRadius"), 2.0f);
        m_config.arrayTimeStep = j.value(json::json_pointer("/simulation/targetArrayTimeStep"), 0.1f);

        // Безпечне зчитування нових параметрів із пункту 4 ТЗ:
        //m_config.targetTimeStep  = j.value("targetTimeStep", 0.05f);
        //m_config.physicsTimeStep = j.value("physicsTimeStep", 0.01f);
        //m_config.timeScale       = j.value("timeScale", 150.0f);

        m_config.targetTimeStep  = j.value(json::json_pointer("/simulation/targetTimeStep"), 0.05f);
        m_config.physicsTimeStep = j.value(json::json_pointer("/simulation/physicsTimeStep"), 0.01f);
        m_config.timeScale       = j.value(json::json_pointer("/simulation/timeScale"), 100.0f);
    //=====================================
    //  Читання ammo.json
    //=====================================
        std::ifstream fa("data/ammo.json");
        if (!fa.is_open()) { std::cerr << "Cannot open ammo.json" << std::endl; return ; }
        json ja;
        fa >> ja;
        fa.close();

        m_ammoCount = (int)ja.size();
        m_ammoParams = new AmmoParams[m_ammoCount];
        for (int i = 0; i < m_ammoCount; i++)
        {
            std::strncpy(m_ammoParams[i].name, ja[i]["name"].get<std::string>().c_str(), 31);
            m_ammoParams[i].name[31] = '\0';
            m_ammoParams[i].mass = ja[i]["mass"];
            m_ammoParams[i].drag = ja[i]["drag"];
            m_ammoParams[i].lift = ja[i]["lift"];
        }


}

DroneConfig FileConfigLoader::getConfig() {
    
    //Тут заповнюємо структуру config даними з файлу
    return m_config;
}

AmmoParams* FileConfigLoader::getAmmoParams() {
    
    //Тут заповнюємо структуру params даними з файлу
    return m_ammoParams;
}


FileConfigLoader::~FileConfigLoader() {
    if(m_ammoParams != nullptr) {
        delete[] m_ammoParams;
    }
}

int FileConfigLoader::getAmmoCount() const {
    return m_ammoCount;
}

