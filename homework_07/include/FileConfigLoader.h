#pragma once
#include "interfaces/Common.hpp"
#include "interfaces/IConfigLoader.h"

class FileConfigLoader : public IConfigLoader {
    public:
        FileConfigLoader() = default;
        ~FileConfigLoader() override = default;

        void load() override;
        DroneConfig getConfig() override;
        AmmoParams getAmmoParams() override;
};