# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(mv_tree LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)

qt_standard_project_setup()

qt_add_executable(mv_tree
    main.cpp
    mainwindow.cpp mainwindow.h
)

set_target_properties(mv_tree PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE
)

target_link_libraries(mv_tree PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
)

get_filename_component(CMAKE_TOOLCHAIN_FILENAME ${CMAKE_TOOLCHAIN_FILE} NAME)
string(COMPARE EQUAL ${CMAKE_TOOLCHAIN_FILENAME} "vcpkg.cmake" USE_VCPKG)
if(USE_VCPKG)
set(TOOL_WINDEPLOYQT "windeployqt")
get_filename_component(CMAKE_TOOLCHAIN_DIR ${CMAKE_TOOLCHAIN_FILE} DIRECTORY)
add_custom_command(TARGET mv_tree POST_BUILD
COMMAND ${CMAKE_TOOLCHAIN_DIR}/../../installed/x64-windows/tools/Qt6/bin/${TOOL_WINDEPLOYQT}$<$<CONFIG:Debug,RelWithDebInfo>:.debug.bat>
        $<TARGET_FILE:mv_tree>
COMMENT "Running windeployqt... ${CMAKE_TOOLCHAIN_DIR}/../../packages/qtbase_x64-windows/tools/Qt6/bin/${TOOL_WINDEPLOYQT}$<$<CONFIG:Debug,RelWithDebInfo>:.debug.bat>"
)
endif()#todo make a function

install(TARGETS mv_tree
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET mv_tree
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
