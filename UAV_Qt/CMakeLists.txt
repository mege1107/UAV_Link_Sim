cmake_minimum_required(VERSION 3.16)
add_compile_options(/utf-8)

project(UAV_Qt VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets LinguistTools Concurrent Charts)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets LinguistTools Concurrent Charts)

set(TS_FILES UAV_Qt_zh_CN.ts)

set(PROJECT_SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    ${TS_FILES}
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(UAV_Qt
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}

        ../UAV_Link_Sim/channel.cpp
        ../UAV_Link_Sim/coding.cpp
        ../UAV_Link_Sim/frequency_hop.cpp
        ../UAV_Link_Sim/modulation.cpp
        ../UAV_Link_Sim/radio_awgn.cpp
        ../UAV_Link_Sim/radio_loopback.cpp
        ../UAV_Link_Sim/radio_usrp.cpp
        ../UAV_Link_Sim/receiver.cpp
        ../UAV_Link_Sim/sim_runner.cpp
        ../UAV_Link_Sim/spread_spectrum.cpp
        ../UAV_Link_Sim/sync.cpp
        ../UAV_Link_Sim/transmitter.cpp
        ../UAV_Link_Sim/usrp_driver.cpp
        ../UAV_Link_Sim/utils.cpp
    )

    target_include_directories(UAV_Qt PRIVATE
        ../UAV_Link_Sim
        D:/UHD/include
        D:/vcpkg/installed/x64-windows/include
    )

    target_link_directories(UAV_Qt PRIVATE
        D:/UHD/lib
    )

    target_link_libraries(UAV_Qt PRIVATE
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::Concurrent
        Qt${QT_VERSION_MAJOR}::Charts
        uhd
    )

    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})

else()
    if(ANDROID)
        add_library(UAV_Qt SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(UAV_Qt
            ${PROJECT_SOURCES}
        )
    endif()

    target_link_libraries(UAV_Qt PRIVATE
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::Concurrent
        Qt${QT_VERSION_MAJOR}::Charts
    )

    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

if(${QT_VERSION} VERSION_LESS 6.1.0)
    set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.UAV_Qt)
endif()

set_target_properties(UAV_Qt PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS UAV_Qt
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(UAV_Qt)
endif()