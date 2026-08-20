cmake_minimum_required(VERSION 3.20)
project(homework_11)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(nlohmann_json 3.12.0 REQUIRED)
find_package(Threads REQUIRED)

set(PROJECT_SOURCES
    src/main.cpp
    src/AnalyticalSolver.cpp
    src/MissionProcessor.cpp
    src/DroneLink.cpp
)

add_executable(student ${PROJECT_SOURCES})

target_link_libraries(student PRIVATE 
    nlohmann_json::nlohmann_json
    Threads::Threads
    gpiod
)
target_include_directories(student PRIVATE
     ${CMAKE_CURRENT_SOURCE_DIR}/include
)



