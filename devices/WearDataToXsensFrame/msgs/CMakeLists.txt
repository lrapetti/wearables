# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

yarp_add_idl(XSENSFRAME_FILES thrift/XsensFrame.thrift)

add_library(XSensFrame ${XSENSFRAME_FILES} thrift/XsensFrame.thrift)
target_link_libraries(XSensFrame YARP::YARP_OS)

# Extract the include directory from the files names
foreach(file ${XSENSFRAME_FILES})
    STRING(REGEX MATCH ".+\\.h?h$" file ${file})
    if(file)
        get_filename_component(include_dir ${file} DIRECTORY)
        list(APPEND XSENSFRAME_INCLUDE_DIRS ${include_dir})
        list(REMOVE_DUPLICATES XSENSFRAME_INCLUDE_DIRS)
    endif()
endforeach()

foreach(dir ${XSENSFRAME_INCLUDE_DIRS})
    get_filename_component(parent_dir_name ${dir} NAME)
    if(${parent_dir_name} STREQUAL thrift)
        list(REMOVE_ITEM XSENSFRAME_INCLUDE_DIRS ${dir})
        get_filename_component(parent_dir_path ${dir} DIRECTORY)
        list(APPEND XSENSFRAME_INCLUDE_DIRS ${parent_dir_path})
    endif()
endforeach()

# Setup the include directories
target_include_directories(XSensFrame PUBLIC
    $<BUILD_INTERFACE:${XSENSFRAME_INCLUDE_DIRS}>)
