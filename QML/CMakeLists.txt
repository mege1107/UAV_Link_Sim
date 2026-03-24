cmake_minimum_required(VERSION 3.16)

project(QML VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS
    Quick
    QuickControls2
    QuickDialogs2
    Concurrent
    Charts
)

qt_standard_project_setup(REQUIRES 6.10)

qt_add_executable(appQML
    main.cpp
    backend.cpp
    backend.h

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

qt_add_qml_module(appQML
    URI App
    VERSION 1.0
    QML_FILES
        Main.qml
    RESOURCES
        images/groundstation_ui.png
)

target_include_directories(appQML PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../UAV_Link_Sim
    D:/UHD/include
    D:/vcpkg/installed/x64-windows/include
)

target_link_directories(appQML PRIVATE
    D:/UHD/lib
)

target_link_libraries(appQML
    PRIVATE
        Qt6::Quick
        Qt6::QuickControls2
        Qt6::QuickDialogs2
        Qt6::Concurrent
        Qt6::Charts
        uhd
)

set_target_properties(appQML PROPERTIES
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE FALSE
)

include(GNUInstallDirs)
install(TARGETS appQML
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
