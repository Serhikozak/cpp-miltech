#pragma once
#include <string>
#include <cstring>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

struct AmmoParams {
    char  name[32] = {0};
    float mass = 0.0f;
    float drag = 0.0f;
    float lift = 0.0f;
}; 

// JSON мапінг з безпечним копіюванням у char[]
inline void to_json(nlohmann::json& j, const AmmoParams& a) {
    j = json{{"name", std::string(a.name)}, {"mass", a.mass}, {"drag", a.drag}, {"lift", a.lift}};
}

inline void from_json(const nlohmann::json& j, AmmoParams& a) {
    std::string name_str = j.at("name").get<std::string>();
    std::strncpy(a.name, name_str.c_str(), sizeof(a.name) - 1);
    a.name[sizeof(a.name) - 1] = '\0'; 
    j.at("mass").get_to(a.mass);
    j.at("drag").get_to(a.drag);
    j.at("lift").get_to(a.lift);
}