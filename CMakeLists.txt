cmake_minimum_required(VERSION 3.0)
project(LineDetection LANGUAGES CXX)

add_executable(${PROJECT_NAME} src/main.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)

# OpenCV setup
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED) 

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
target_link_libraries(${PROJECT_NAME} ${WIRINGPI_LIBRARIES} -lcrypt -lrt -lpthread)
