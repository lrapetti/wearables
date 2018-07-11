/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WearDataToXsensFrameDevice.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>

#include <vector>

using namespace yarp::dev;

const std::string DeviceName = "WearDataToXsensFrameDevice";
const std::string logPrefix = DeviceName + " : ";

// ======================
// DeviceDriver interface
// ======================

bool WearDataToXsensFrameDevice::open(yarp::os::Searchable& config)
{
    // Default size for empty vectors
    // ==============================
    if (!config.check("linkVector_size")) {
        yError() << logPrefix << "Size for the empty link-related variable not defined";
        return false;
    }
    if (!config.check("sensorVector_size")) {
        yError() << logPrefix << "Size for the empty sensor-related variable not defined";
        return false;
    }

    // Get vector sizes from the config file
    m_linkVectorSize = static_cast<unsigned>(config.find("linkVector_size").asInt());
    m_sensorVectorSize = static_cast<unsigned>(config.find("sensorVector_size").asInt());

    // Ports
    // =====
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

// ===================
// TypedReaderCallback
// ===================

void WearDataToXsensFrameDevice::onRead(wearable::msg::WearData& inData)
{
    // inData contains fields as subdivided in the WearData.thrift

    // Links Quantities
    // ===================
    // From ~WearData.thrift~
    // --------------------------------------- //
    //    struct VirtualLinkKinSensorData {    //
    //      1: QuaternionWXYZ orientation,     //
    //      2: VectorXYZ position,             //
    //      3: VectorXYZ linearVelocity,       //
    //      4: VectorXYZ angularVelocity,      //
    //      5: VectorXYZ linearAcceleration,   //
    //      6: VectorXYZ angularAcceleration   //
    //    }                                    //
    // --------------------------------------- //
    //
    // to ~XsensFrame.thrift~
    // --------------------------------------- //
    //    struct XsensSegmentData {            //
    //        1:  Vector3 position;            //
    //        2:  Vector3 velocity;            //
    //        3:  Vector3 acceleration;        //
    //        4:  Quaternion orientation;      //
    //        5:  Vector3 angularVelocity;     //
    //        6:  Vector3 angularAcceleration; //
    //    }                                    //
    // --------------------------------------- //

    xsens::XsensSegmentsFrame xsSegmentFrame;

    // Check if the VirtualLinkKinSensor is empty. If yes, replicate the same structure of the
    // inData but with all the values equal to 0.0.
    std::vector<wearable::msg::VirtualLinkKinSensor> inputLinkQty;

    if (inData.virtualLinkKinSensors.empty()) {
        // create a dummy wds element
        // --------------------------
        wearable::msg::VirtualLinkKinSensor dummyElement_wds{
            {"", // initialize name (from info field)
             wearable::msg::SensorType::VIRTUAL_LINK_KIN_SENSOR, // initialize type (from info
                                                                 // field)
             wearable::msg::SensorStatus::ERROR}, // initialize status (from info field)
            {{0.0, {0.0, 0.0, 0.0}}, // initialize quaternion (from data field)
             {0.0, 0.0, 0.0}, // initialize position  (from data field)
             {0.0, 0.0, 0.0}, // initialize lineraVelocity  (from data field)
             {0.0, 0.0, 0.0}, // initialize angularVelocity  (from data field)
             {0.0, 0.0, 0.0}, // initialize linearAcceleration  (from data field)
             {0.0, 0.0, 0.0}}}; // initialize angularAcceleration  (from data field)

        // create a dummy wds struct of m_linkVectorSize wds elements
        // ------------------------------------------------------------------
        std::vector<wearable::msg::VirtualLinkKinSensor> dummy_wds;
        dummy_wds.resize(m_linkVectorSize, dummyElement_wds);

        inputLinkQty = std::move(dummy_wds);
    }
    else {
        inputLinkQty = std::move(inData.virtualLinkKinSensors);
    }

    xsSegmentFrame.segmentsData.reserve(inputLinkQty.size());
    for (const auto& wds : inputLinkQty) {
        xsens::XsensSegmentData xsSegmentData;

        // TODO: check for each single field the possibility of a null vector.

        // Quaternion
        xsSegmentData.orientation = {wds.data.orientation.w,
                                     {wds.data.orientation.imaginary.x,
                                      wds.data.orientation.imaginary.y,
                                      wds.data.orientation.imaginary.z}};
        // Position
        xsSegmentData.position = {wds.data.position.x, wds.data.position.y, wds.data.position.z};
        // Velocity
        xsSegmentData.velocity = {
            wds.data.linearVelocity.x, wds.data.linearVelocity.y, wds.data.linearVelocity.z};
        // Acceleration
        xsSegmentData.acceleration = {wds.data.linearAcceleration.x,
                                      wds.data.linearAcceleration.y,
                                      wds.data.linearAcceleration.z};
        // Angular Velocityn
        xsSegmentData.angularVelocity = {
            wds.data.angularVelocity.x, wds.data.angularVelocity.y, wds.data.angularVelocity.z};
        // Angular Acceleration
        xsSegmentData.angularAcceleration = {wds.data.angularAcceleration.x,
                                             wds.data.angularAcceleration.y,
                                             wds.data.angularAcceleration.z};

        xsSegmentFrame.segmentsData.push_back(xsSegmentData);
    }

    // Sensors Quantities
    // ===================
    // From ~WearData.thrift~
    // --------------------------------------- //
    //    struct FreeBodyAccelerationSensor {  //
    //       1: SensorInfo info,               //
    //       2: VectorXYZ data                 //
    //    }                                    //
    // ---------
    //    struct Magnetometer {                //
    //      1: SensorInfo info,                //
    //      2: VectorXYZ data                  //
    //    }                                    //
    // ---------
    //    struct OrientationSensor {           //
    //      1: SensorInfo info,                //
    //      2: QuaternionWXYZ data             //
    //    }                                    //
    // --------------------------------------- //

    // to ~XsensFrame.thrift~
    // --------------------------------------- //
    //    struct XsensSensorData {             //
    //        1:  Vector3 acceleration;        //
    //        2:  Vector3 angularVelocity;     //
    //        3:  Vector3 magnetometer;        //
    //        4:  Quaternion orientation;      //
    //    }                                    //
    // --------------------------------------- //
    // Note that the angularVelocity is not provided anymore as a sensor. We decided to provide this
    // field as a 0.0 field.

    xsens::XsensSensorsFrame xsSensorFrame;
    std::vector<wearable::msg::FreeBodyAccelerationSensor> inputFreeBodyAccQty;
    std::vector<wearable::msg::Magnetometer> inputMagnetometerQty;
    std::vector<wearable::msg::OrientationSensor> inputOrientationSensQty;

    //    unsigned long size_FreeBodyAccelerometer = inData.freeBodyAccelerationSensors.size();
    //    unsigned long size_Magnetometer = inData.magnetometers.size();
    //    unsigned long size_OrientationSensor = inData.orientationSensors.size();

    xsSensorFrame.sensorsData.reserve(inData.freeBodyAccelerationSensors.size());
    xsSensorFrame.sensorsData.reserve(inData.magnetometers.size());
    xsSensorFrame.sensorsData.reserve(inData.orientationSensors.size());

    // Check if the sensor structs are empty.
    // Assumption: if one is empty, all empty
    // TODO: check if each single sensor struct is empty.
    if (inData.freeBodyAccelerationSensors.empty()) {

        // create a dummy element for each sensor
        // --------------------------------------
        // freeBodyAccelerationSensors: initialization to zero
        wearable::msg::FreeBodyAccelerationSensor dummyElement_freeBodyAcc{
            {"", // initialize name (from info field)}
             wearable::msg::SensorType::FREE_BODY_ACCELERATION_SENSOR, // initialize type (from info
                                                                       // field)
             wearable::msg::SensorStatus::ERROR}, // initialize status (from info field)
            {0.0, 0.0, 0.0}};

        // magnetometers: initialization to zero
        wearable::msg::Magnetometer dummyElement_magneto{
            {"", // initialize name (from info field)}
             wearable::msg::SensorType::MAGNETOMETER, // initialize type (from info
                                                      // field)
             wearable::msg::SensorStatus::ERROR}, // initialize status (from info field)
            {0.0, 0.0, 0.0}};

        // orientationSensors: initialization to zero
        wearable::msg::OrientationSensor dummyElement_orientSens{
            {"", // initialize name (from info field)}
             wearable::msg::SensorType::ORIENTATION_SENSOR, // initialize type (from info
                                                            // field)
             wearable::msg::SensorStatus::ERROR}, // initialize status (from info field)
            {0.0, {0.0, 0.0, 0.0}}};
        ;
        // create a dummy struct of m_linkVectorSize elements for each sensor
        // ------------------------------------------------------------------
        std::vector<wearable::msg::FreeBodyAccelerationSensor> dummy_freeBodyAcc;
        dummy_freeBodyAcc.resize(m_sensorVectorSize, dummyElement_freeBodyAcc);

        std::vector<wearable::msg::Magnetometer> dummy_magneto;
        dummy_magneto.resize(m_sensorVectorSize, dummyElement_magneto);

        std::vector<wearable::msg::OrientationSensor> dummy_orientSens;
        dummy_orientSens.resize(m_sensorVectorSize, dummyElement_orientSens);

        inputFreeBodyAccQty = std::move(dummy_freeBodyAcc);
        inputMagnetometerQty = std::move(dummy_magneto);
        inputOrientationSensQty = std::move(dummy_orientSens);
    }
    else {
        inputFreeBodyAccQty = std::move(inData.freeBodyAccelerationSensors);
        inputMagnetometerQty = std::move(inData.magnetometers);
        inputOrientationSensQty = std::move(inData.orientationSensors);
    }

    // Assumption:
    // freeBodyAccelerationSensors.size() == inData.magnetometers.size() ==
    // inData.orientationSensors.size()
    for (unsigned long i = 0; i < inData.freeBodyAccelerationSensors.size(); ++i) {
        xsens::XsensSensorData xsSensorData;

        // TODO: check for each single field the possibility of a null vector.

        // Acceleration
        xsSensorData.acceleration = {inputFreeBodyAccQty.at(i).data.x,
                                     inputFreeBodyAccQty.at(i).data.y,
                                     inputFreeBodyAccQty.at(i).data.z};
        // Magnetometer
        xsSensorData.magnetometer = {inputMagnetometerQty.at(i).data.x,
                                     inputMagnetometerQty.at(i).data.y,
                                     inputMagnetometerQty.at(i).data.z};
        // Orientation
        xsSensorData.orientation = {inputOrientationSensQty.at(i).data.w,
                                    {inputOrientationSensQty.at(i).data.imaginary.x,
                                     inputOrientationSensQty.at(i).data.imaginary.y,
                                     inputOrientationSensQty.at(i).data.imaginary.z}};
        // Angular Velocity
        xsSensorData.angularVelocity = {0.0, 0.0, 0.0};

        xsSensorFrame.sensorsData.push_back(xsSensorData);
    }
}

// TODO: handle the status! From the ~WearData.thrift~, the STATUS exists per each sensor, in
// the ~XsensFrame.thrift~ there is a STATUS for the whole Xsens suit.  Define a proper mapping.
