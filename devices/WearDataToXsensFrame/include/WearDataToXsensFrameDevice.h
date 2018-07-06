/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WEAR_DATA_TO_XSENS_FRAME_DEVICE_H
#define WEAR_DATA_TO_XSENS_FRAME_DEVICE_H

#include <yarp/dev/DeviceDriver.h>

namespace yarp {
    namespace dev {
        class WearDataToXsensFrameDevice;
    }
} // namespace yarp

class yarp::dev::WearDataToXsensFrameDevice : public yarp::dev::DeviceDriver
{
private:
public:
    WearDataToXsensFrameDevice() = default;
    ~WearDataToXsensFrameDevice() override = default;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;
};

#endif // WEAR_DATA_TO_XSENS_FRAME_DEVICE_H
