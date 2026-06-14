cmake_minimum_required(VERSION 4.3)

# Must be set before project() so CMake picks it up during compiler detection.
if(NOT DEFINED CMAKE_CXX_COMPILER)
    set(CMAKE_CXX_COMPILER clang++-23)
endif()

project(cal_viz CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)  # lets CMake 4.x tolerate old cmake_minimum_required in fetched deps

include(FetchContent)

FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        5.0.1
    GIT_SHALLOW    TRUE
)
FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        0.8.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(eigen yaml-cpp)

add_executable(cal_viz src/main.cpp)

target_include_directories(cal_viz PRIVATE src third_party)
target_link_libraries(cal_viz PRIVATE Eigen3::Eigen yaml-cpp::yaml-cpp)
