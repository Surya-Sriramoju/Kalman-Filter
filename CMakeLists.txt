cmake_minimum_required(VERSION 3.12)
project(KalmanFilter)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add source files
set(SOURCES
    main.cpp
    src/kalman_filter.cpp
    src/utils.cpp
)

# Add header files
set(HEADERS
    include/kalman_filter.hpp
    include/utils.hpp
)

# Add the executable target
add_executable(${PROJECT_NAME} ${SOURCES})

# Set include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    data
)

# Set additional compiler options
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

find_package(Eigen3 REQUIRED NO_MODULE)
# Set additional linker options if needed
target_link_libraries(${PROJECT_NAME} PRIVATE
    Eigen3::Eigen
)