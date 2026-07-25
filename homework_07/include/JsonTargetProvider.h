#pragma once
#include "interfaces/ITargetProvider.h"
#include <nlohmann/json.hpp>
#include "interfaces/Common.hpp"

class JsonTargetProvider : public ITargetProvider {
    public:
        JsonTargetProvider(const char* param);
        ~JsonTargetProvider() override = default;

        int getTargetCount() override;
        Coord getTarget(int index) override;

        void updateTime(int stepIdx);
    private:
        int m_targetCount = 0;
        int m_timeStep = 0;
        int m_currentTimeIdx = 0; //Загальний шаг часу для всіх цілей
        nlohmann::json m_targetsJson; //Сюди зчитуєм масив цілей із файлу
};