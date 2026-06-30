cmake_minimum_required(VERSION 3.5)
project(cspc_lidar)

# Set C++ version to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source code files
file(GLOB SOURCES ./*.cpp)

# Add executable file generation rules
add_executable(${PROJECT_NAME} ${SOURCES})

# Link pthread library
find_package(Threads REQUIRED)
target_link_libraries(cspc_lidar PRIVATE Threads::Threads)
