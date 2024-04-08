cmake_minimum_required(VERSION 3.5)

project(VenderAdd VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_library(VenderAdd SHARED
    "Interface.h" 
    "Export.h"
    "Export.cpp"
    "VenderAdd.h"
)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
MESSAGE(STATUS "QT_VERSION_MAJOR: ${QT_VERSION_MAJOR}")

target_link_libraries(VenderAdd PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

set_target_properties(VenderAdd PROPERTIES
          CXX_STANDARD 17
          #CXX_EXTENSIONS OFF
          )

set_target_properties(${component}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../../use_dll/build
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../../use_dll/build
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../../use_dll/build
)

target_include_directories(VenderAdd
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include/${project_name}>
)