cmake_minimum_required(VERSION 3.5)

project(plugin VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_library(PluginA
    # IPlugin.h
    # PluginManager.h
    # PluginManager.cpp
    myplugin1dll.h
    myplugin1dll.cpp
)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core)
MESSAGE(STATUS "QT_VERSION_MAJOR: ${QT_VERSION_MAJOR}")

target_link_libraries(PluginA PRIVATE Qt${QT_VERSION_MAJOR}::Core)

set_target_properties(PluginA PROPERTIES
          CXX_STANDARD 17
          #CXX_EXTENSIONS OFF
          )

target_include_directories(PluginA
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include/${project_name}>
)