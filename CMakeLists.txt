cmake_minimum_required(VERSION 3.16.0)

project(raytracer
    LANGUAGES CXX
    VERSION 0.01)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_COLOR_DIAGNOSTICS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

# 1. Look for a SDL2 package, 2. look for the SDL2 component and 3. fail if none can be found
find_package(SDL2 REQUIRED CONFIG REQUIRED COMPONENTS SDL2)

# 1. Look for a SDL2 package, 2. Look for the SDL2maincomponent and 3. DO NOT fail when SDL2main is not available
find_package(SDL2 REQUIRED CONFIG COMPONENTS SDL2main)

find_package(nlohmann_json 3.2.0 REQUIRED)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/sdl_window.cpp
    src/raytracer.cpp
    src/math.cpp
    src/scene.cpp
    src/material.cpp
    src/camera.cpp
    src/scene_tree.cpp
    src/obj_loader.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE include)

if(TARGET SDL2::SDL2main)
    target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2main)
endif()
target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2 nlohmann_json::nlohmann_json)

enable_testing()
find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_math_tests
    tests/math_tests.cpp
    src/math.cpp
)
target_include_directories(${PROJECT_NAME}_math_tests PRIVATE include)
target_link_libraries(${PROJECT_NAME}_math_tests PRIVATE GTest::gtest GTest::gtest_main)

add_test(AllTests ${PROJECT_NAME}_math_tests)
