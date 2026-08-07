
#include "JsonTargetProvider.h"

#include <fstream>
#include <nlohmann/json.hpp>
#include "interfaces/Common.hpp"
# include <iostream>
#include <stdexcept>

using json = nlohmann::json;

JsonTargetProvider::JsonTargetProvider(const std::string& param) {
    std::ifstream target(param);
    if (!target.is_open()) {
        //Замість return викидаємо exeption
        throw std::runtime_error ("Cannot open targets.json " + param);
    }
    
    try {
        json targetdata;
        target >> targetdata;

         m_timeStep = targetdata.value("timeSteps", 0);
        //Парсинг масиву цілейза допомогою сучасногоциклу (Range-based for)
        if (targetdata.contains("targets") && targetdata["targets"].is_array()) {
            for (const auto& targetJson : targetdata["targets"]) {
                TargetTrajectory trajectory;

                //Заповнюємо внутрішній вектор positions координатами
                if (targetJson.contains("positions") && targetJson["positions"].is_array()) {
                    for (const auto& posJson : targetJson["positions"]) {
                        Coord c{.x = 0.0, .y = 0.0};
                        c.x = posJson.value("x", 0.0);
                        c.y = posJson.value("y", 0.0);

                        trajectory.positions.push_back(c);
                    }
                }
                //Зберігаємо готову траекторію у вектор цілей
                m_targets.push_back(trajectory);
            }
        }

    }

    catch (const json::parse_error& e) {
        throw std::runtime_error("JSON parse error in " + param + ": " + e.what());
    }



        
        //target.close();

    //Зчитуємо кількість цілей з файлу
    //m_targetCount = data.value("targetCount", 0);
   

    //if (data.contains("targets")) {
        //m_targetsJson = data["targets"];
    //}
}
    //Реалізація віртуального методу getTargetCount
    int JsonTargetProvider::getTargetCount()  {
        return static_cast<int>(m_targets.size());
    }

    void JsonTargetProvider::updateTime(int stepIdx) {
        if (m_timeStep > 0) {
            m_currentTimeIdx = stepIdx % m_timeStep;
        }
    }

    //Миттевий та безпечний пошук в памяті STL без парсингу JSON на льоту
    Coord JsonTargetProvider::getTarget(int index)  {
        
        if (index >= 0 && index < static_cast <int>(m_targets.size())) {
            const auto& positions = m_targets[index].positions;

            if (m_currentTimeIdx >= 0 && m_currentTimeIdx < static_cast<int>(positions.size())) {
                return positions[m_currentTimeIdx];
                
            }
        }
        return Coord{.x = 0.0, .y = 0.0};
    }


