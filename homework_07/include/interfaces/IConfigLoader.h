#pragma once
#include "Common.hpp"
class IConfigLoader {
    public:
    
    //Метод для відкриття файлів config.json ammo.json
    virtual void load() = 0;

    //Повертає структуру з налаштуваннями дрона
    virtual DroneConfig getConfig() = 0;
    //Повертає структуру з параметрами боєприпасу
    virtual AmmoParams getAmmoParams() = 0;

    virtual ~IConfigLoader() = default;
};