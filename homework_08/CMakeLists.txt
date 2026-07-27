cmake_minimum_required(VERSION 3.20)
project(homework_08)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(nlohmann_json 3.12.0 REQUIRED)

set(PROJECT_SOURCES
    src/main.cpp
    src/Coord.cpp
    src/JsonTargetProvider.cpp
    src/FileConfigLoader.cpp
    src/AnalyticalSolver.cpp
    src/ComponentFactory.cpp
    
)

add_executable(homework_08 ${PROJECT_SOURCES})
target_link_libraries(homework_08 PRIVATE nlohmann_json::nlohmann_json)
target_include_directories(homework_08 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_custom_command(TARGET homework_08 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_CURRENT_SOURCE_DIR}/data"
    "$<TARGET_FILE_DIR:homework_08>/data")

