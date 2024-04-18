# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(TreeViewDemo LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)

qt_standard_project_setup()

qt_add_executable(TreeViewDemo
    main.cpp
    mainwindow.cpp 
    mainwindow.h
    mainwindow.ui
    MyDataModel.h
    MyDataModel.cpp
    PointerVector.h
    TopData.h
    TopData.cpp
)

set_target_properties(TreeViewDemo PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE
)

target_link_libraries(TreeViewDemo PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

get_filename_component(CMAKE_TOOLCHAIN_FILENAME ${CMAKE_TOOLCHAIN_FILE} NAME)
string(COMPARE EQUAL ${CMAKE_TOOLCHAIN_FILENAME} "vcpkg.cmake" USE_VCPKG)
if(USE_VCPKG)
set(TOOL_WINDEPLOYQT "windeployqt")
get_filename_component(CMAKE_TOOLCHAIN_DIR ${CMAKE_TOOLCHAIN_FILE} DIRECTORY)
add_custom_command(TARGET TreeViewDemo POST_BUILD
COMMAND ${CMAKE_TOOLCHAIN_DIR}/../../installed/x64-windows/tools/Qt6/bin/${TOOL_WINDEPLOYQT}$<$<CONFIG:Debug,RelWithDebInfo>:.debug.bat>
        $<TARGET_FILE:TreeViewDemo>
COMMENT "Running windeployqt... ${CMAKE_TOOLCHAIN_DIR}/../../packages/qtbase_x64-windows/tools/Qt6/bin/${TOOL_WINDEPLOYQT}$<$<CONFIG:Debug,RelWithDebInfo>:.debug.bat>"
)
endif()#todo make a function

install(TARGETS TreeViewDemo
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET TreeViewDemo
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
