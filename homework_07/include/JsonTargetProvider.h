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
    private:
        int m_targetCount = 0;
        nlohmann::json m_jsonData;
};