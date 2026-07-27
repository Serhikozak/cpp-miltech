#pragma once
#include "interfaces/Common.hpp"
#include "interfaces/IConfigLoader.h"
#include <string>

class FileConfigLoader : public IConfigLoader {
    public:
        FileConfigLoader() = default;
        ~FileConfigLoader() override;

        void load() override;
        DroneConfig getConfig() override;
        virtual AmmoParams* getAmmoParams() override;

        int getAmmoCount() const;
    private:
        DroneConfig m_config;
        AmmoParams* m_ammoParams = nullptr;
        int m_ammoCount = 0;
};