cmake_minimum_required(VERSION 3.5)

project(use_dll VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_executable(QPluginDemo
    "plugin.h" 
    "main.cpp"
)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
MESSAGE(STATUS "QT_VERSION_MAJOR: ${QT_VERSION_MAJOR}")

target_link_libraries(QPluginDemo PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

set_target_properties(QPluginDemo PROPERTIES
          CXX_STANDARD 17
          #CXX_EXTENSIONS OFF
          )

target_include_directories(QPluginDemo
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include/${project_name}>
)