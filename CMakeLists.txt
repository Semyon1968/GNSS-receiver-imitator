cmake_minimum_required(VERSION 3.5)

project(ImitatorGNSS VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt with required components
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Network PrintSupport)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Network PrintSupport)

# QCustomPlot sources (modified version with Antialiasing fixes)
set(QCP_SOURCES
    qcustomplot.cpp
    qcustomplot.h
)

set(PROJECT_SOURCES
    main.cpp
    gnsswindow.cpp
    gnsswindow.h
    gnsswindow.ui
    dialog.cpp
    dialog.h
    dialog.ui
    ubxparser.cpp
    ubxparser.h
    ${QCP_SOURCES}
)

# Check for duplicate source files
list(REMOVE_DUPLICATES PROJECT_SOURCES)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(ImitatorGNSS
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    if(ANDROID)
        add_library(ImitatorGNSS SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(ImitatorGNSS
            ${PROJECT_SOURCES}
            ubxdefs.h
        )
    endif()
endif()

# Link required Qt modules
target_link_libraries(ImitatorGNSS PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Network
    Qt${QT_VERSION_MAJOR}::PrintSupport
)

# Include directories
target_include_directories(ImitatorGNSS PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# macOS bundle settings
set(BUNDLE_ID_OPTION "")
if(APPLE AND ${QT_VERSION} VERSION_LESS 6.1.0)
    set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.ImitatorGNSS)
endif()

set_target_properties(ImitatorGNSS PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

# Installation
include(GNUInstallDirs)
install(TARGETS ImitatorGNSS
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Finalize for Qt6
if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(ImitatorGNSS)
endif()

# Add definitions to avoid multiple declarations
target_compile_definitions(ImitatorGNSS PRIVATE
    QT_DEPRECATED_WARNINGS
    QT_DISABLE_DEPRECATED_BEFORE=0x050F00  # Disable deprecation warnings before Qt 5.15
)
