#pragma once
#include "interfaces/ITargetProvider.h"
//#include <nlohmann/json.hpp>
#include "interfaces/Common.hpp"
#include <string>
#include <vector>

class JsonTargetProvider : public ITargetProvider {
    public:
        //Використовуємо  explicit та std::string замість char*
        explicit JsonTargetProvider(const std::string& param);
        ~JsonTargetProvider() override = default;
        //реалізація інтерфейсних методів
        int getTargetCount() override;
        Coord getTarget(int index) override;

        void updateTime(int stepIdx);
    private:
        //Замість m_targetCount та m_TargetsJson використовуємо чистий вектор STL
        std::vector<TargetTrajectory>m_targets;
        
        int m_timeStep = 0;
        int m_currentTimeIdx = 0; //Загальний шаг часу для всіх цілей
        //nlohmann::json m_targetsJson; //Сюди зчитуєм масив цілей із файлу
};