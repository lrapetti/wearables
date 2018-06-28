/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEARDATATOXSENSFRAMEWRAPPER_H
#define WEARDATATOXSENSFRAMEWRAPPER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>

namespace yarp {
    namespace dev {
        class WearDataToXSensFrameWrapper;
    }
} // namespace yarp

class yarp::dev::WearDataToXSensFrameWrapper
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IWrapper
    , public yarp::dev::IMultipleWrapper
{
private:
public:
    WearDataToXSensFrameWrapper() = default;
    ~WearDataToXSensFrameWrapper() override = default;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // IWrapper interface
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList&) override;
    bool detachAll() override;
};

#endif // WEARDATATOXSENSFRAMEWRAPPER_H
