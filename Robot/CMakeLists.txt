# CMakeList.txt: файл проекта CMake верхнего уровня; проведите глобальную конфигурацию
# и включите сюда подпроекты.
#
project(Robot)
cmake_minimum_required(VERSION 3.12)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../)
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB CPPS "*.cpp")
add_definitions(-Wall -O2)
add_executable(${PROJECT_NAME} ${CPPS})
target_link_libraries(${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
