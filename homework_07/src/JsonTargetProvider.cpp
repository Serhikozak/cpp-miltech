
#include "JsonTargetProvider.h"

#include <fstream>
#include <nlohmann/json.hpp>
#include "interfaces/Common.hpp"

using json = nlohmann::json;

JsonTargetProvider:: JsonTargetProvider(const char* param) {
    std::ifstream target(param);
    if (!target.is_open()) return;

    json data;
    target >> data;

    //Зчитуємо кількість цілей з файлу
    m_targetCount = data.value("targetCount", 0);
}
    //Реалізація віртуального методу getTargetCount
    int JsonTargetProvider::getTargetCount()  {
        return m_targetCount;
    }

    Coord JsonTargetProvider::getTarget(int index)  {
        Coord t;
        return t;
    }


