cmake_minimum_required(VERSION 3.5)

project(spd_qt VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_executable(spd_qt "mainwindow.h" "mainwindow.cpp"  "main.cpp" "widget.cpp" "widget.h")

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(spdlog REQUIRED)
MESSAGE(STATUS "QT_VERSION_MAJOR: ${QT_VERSION_MAJOR}")

target_link_libraries(spd_qt PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(spd_qt PRIVATE spdlog::spdlog)

set_target_properties(spd_qt PROPERTIES
          CXX_STANDARD 17
          #CXX_EXTENSIONS OFF
          )