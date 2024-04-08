cmake_minimum_required(VERSION 3.5)

project(log_redirect_to_file VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_executable(log_redirect_file "mainwindow.h" "mainwindow.cpp" "mainwindow.ui" "main.cpp" )

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
MESSAGE(STATUS "QT_VERSION_MAJOR: ${QT_VERSION_MAJOR}")

target_link_libraries(log_redirect_file PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

set_target_properties(log_redirect_file PROPERTIES
          CXX_STANDARD 17
          #CXX_EXTENSIONS OFF
          )
