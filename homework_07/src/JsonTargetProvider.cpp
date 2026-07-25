
#include "JsonTargetProvider.h"

#include <fstream>
#include <nlohmann/json.hpp>
#include "interfaces/Common.hpp"
# include <iostream>

using json = nlohmann::json;

JsonTargetProvider:: JsonTargetProvider(const char* param) {
    std::ifstream target(param);
    if (!target.is_open()) {
        std::cerr <<  "Cannot open target.json " << param << std::endl;
        return;

    } 

    json data;
    target >> data;
    target.close();

    //Зчитуємо кількість цілей з файлу
    m_targetCount = data.value("targetCount", 0);
    m_timeStep = data.value("timeSteps", 0);

    if (data.contains("targets")) {
        m_targetsJson = data["targets"];
    }
}
    //Реалізація віртуального методу getTargetCount
    int JsonTargetProvider::getTargetCount()  {
        return m_targetCount;
    }

    void JsonTargetProvider::updateTime(int stepIdx) {
        if (m_timeStep > 0) {
            m_currentTimeIdx = stepIdx % m_timeStep;
        }
    }

    Coord JsonTargetProvider::getTarget(int index)  {
        Coord t{0.0, 0.0};

        if (!m_targetsJson.empty() && index >= 0 && index < (int)m_targetsJson.size()) {
            auto& positions = m_targetsJson[index]["positions"];
            if (m_currentTimeIdx >= 0 && m_currentTimeIdx < (int)positions.size()) {
                t.x = positions[m_currentTimeIdx].value("x", 0.0);
                t.y = positions[m_currentTimeIdx].value("y", 0.0);
            }
        }
        return t;
    }


