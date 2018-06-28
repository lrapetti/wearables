# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(YARP REQUIRED)

# =======
# WRAPPER
# =======

yarp_prepare_plugin(WearDataToXSensFrameWrapper
    TYPE yarp::dev::WearDataToXSensFrameWrapper
    INCLUDE include/WearDataToXSensFrameWrapper.h
    CATEGORY device)

yarp_add_plugin(WearDataToXSensFrameWrapper
    include/WearDataToXSensFrameWrapper.h
    src/WearDataToXSensFrameWrapper.cpp)

target_link_libraries(WearDataToXSensFrameWrapper
    YARP::YARP_dev)

target_include_directories(WearDataToXSensFrameWrapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
