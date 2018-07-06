/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>

#include "WearDataToXsensFrameDevice.h"

using namespace yarp::dev;

const std::string DeviceName = "WearDataToXsensFrameDevice";
const std::string logPrefix = DeviceName + " : ";

// ======================
// DeviceDriver interface
// ======================

bool WearDataToXsensFrameDevice::open(yarp::os::Searchable& config)
{
    // Check ports in the config file
    if (!config.check("remotePort")) {
        yError() << logPrefix << "Remote port not found";
        return false;
    }
    if (!config.check("input_localPort")) {
        yError() << logPrefix << "Local input port not found";
        return false;
    }
    if (!config.check("output_localPort1")) {
        yError() << logPrefix << "Local output port1 not found";
        return false;
    }
    if (!config.check("output_localPort2")) {
        yError() << logPrefix << "Local output port2 not found";
        return false;
    }
    // Get the ports from the config file
    const std::string remotePort = config.find("remotePort").asString();
    const std::string localInputPort = config.find("input_localPort").asString();
    const std::string localOutputXsensSegmentsPort = config.find("output_localPort1").asString();
    const std::string localOutputXsensSensorsPort = config.find("output_localPort2").asString();

    // Open device input port
    if (!m_xsensDataPort.open("/" + DeviceName + localInputPort)) {
        yError() << logPrefix << "Unable to open the input port "
                 << ("/" + DeviceName + localInputPort).c_str();
        return false;
    }

    // Open device output ports
    if (!m_xsensSegmentsPortToHDE.open("/" + DeviceName + localOutputXsensSegmentsPort)) {
        yError() << logPrefix << "Unable to open the output port "
                 << ("/" + DeviceName + localOutputXsensSegmentsPort).c_str();
        return false;
    }
    if (!m_xsensSensorsPortToHDE.open("/" + DeviceName + localOutputXsensSensorsPort)) {
        yError() << logPrefix << "Unable to open the output port "
                 << ("/" + DeviceName + localOutputXsensSensorsPort).c_str();
        return false;
    }

    // Connect remote port with the device input port
    if (!yarp::os::Network::connect(remotePort,
                                    m_xsensDataPort.getName())) // to connect remote and local port
    {
        yError() << logPrefix << "Unable to connect port " << remotePort << " to "
                 << m_xsensDataPort.getName();
        return false;
    }
    return true;
}

bool WearDataToXsensFrameDevice::close()
{
    m_xsensDataPort.close();
    m_xsensSegmentsPortToHDE.close();
    m_xsensSensorsPortToHDE.close();
    return true;
}
