# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(YARP REQUIRED)

# ======
# THRIFT
# ======

add_subdirectory(msgs)

# =======
# WRAPPER
# =======

yarp_prepare_plugin(WearDataToXsensFrameDevice
    TYPE yarp::dev::WearDataToXsensFrameDevice
    INCLUDE WearDataToXsensFrameDevice.h
    CATEGORY device
    DEFAULT ON)

yarp_add_plugin(WearDataToXsensFrameDevice
    include/WearDataToXsensFrameDevice.h
    src/WearDataToXsensFrameDevice.cpp)

target_link_libraries(WearDataToXsensFrameDevice
    YARP::YARP_dev)

target_include_directories(WearDataToXsensFrameDevice PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
