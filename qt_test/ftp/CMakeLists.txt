cmake_minimum_required(VERSION 3.5)

project(ftpdemo VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Network REQUIRED)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(ftpdemo
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET ftpdemo APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(ftpdemo SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(ftpdemo
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(ftpdemo PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(ftpdemo PRIVATE Qt${QT_VERSION_MAJOR}::Network)

set_target_properties(ftpdemo PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(ftpdemo)
endif()


get_filename_component(CMAKE_TOOLCHAIN_FILENAME ${CMAKE_TOOLCHAIN_FILE} NAME)
string(COMPARE EQUAL ${CMAKE_TOOLCHAIN_FILENAME} "vcpkg.cmake" USE_VCPKG)
if(USE_VCPKG)
set(TOOL_WINDEPLOYQT "windeployqt")
get_filename_component(CMAKE_TOOLCHAIN_DIR ${CMAKE_TOOLCHAIN_FILE} DIRECTORY)
add_custom_command(TARGET ftpdemo POST_BUILD
COMMAND ${CMAKE_TOOLCHAIN_DIR}/../../installed/x64-windows/tools/Qt6/bin/${TOOL_WINDEPLOYQT}$<$<CONFIG:Debug,RelWithDebInfo>:.debug.bat>
        $<TARGET_FILE:ftpdemo>
COMMENT "Running windeployqt... ${CMAKE_TOOLCHAIN_DIR}/../../packages/qtbase_x64-windows/tools/Qt6/bin/${TOOL_WINDEPLOYQT}$<$<CONFIG:Debug,RelWithDebInfo>:.debug.bat>"
)
endif()#todo make a function