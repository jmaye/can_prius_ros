/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file CANPriusNode.h
    \brief This file defines the CANPriusNode class which implements
           the CAN Prius node.
  */

#ifndef CAN_PRIUS_NODE_H
#define CAN_PRIUS_NODE_H

#include <string>
#include <memory>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

class FrontWheelsSpeed;
class RearWheelsSpeed;
class Steering1;
class CANConnection;

namespace diagnostic_updater {
  class HeaderlessTopicDiagnostic;
}

namespace prius {

  /** The class CANPriusNode implements the CAN Prius node.
      \brief CAN Prius node
    */
  class CANPriusNode {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructor
    CANPriusNode(const ros::NodeHandle& nh);
    /// Copy constructor
    CANPriusNode(const CANPriusNode& other) = delete;
    /// Copy assignment operator
    CANPriusNode& operator = (const CANPriusNode& other) = delete;
    /// Move constructor
    CANPriusNode(CANPriusNode&& other) = delete;
    /// Move assignment operator
    CANPriusNode& operator = (CANPriusNode&& other) = delete;
    /// Destructor
    virtual ~CANPriusNode();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Spin once
    void spin();
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Publishes the front wheel speed
    void publishFrontWheelsSpeed(const ros::Time& timestamp,
      const FrontWheelsSpeed& fws);
    /// Publishes the rear wheel speed
    void publishRearWheelsSpeed(const ros::Time& timestamp,
      const RearWheelsSpeed& rws);
    /// Publishes the steering1
    void publishSteering1(const ros::Time& timestamp, const Steering1& st);
    /// Diagnose the CAN connection
    void diagnoseCANConnection(diagnostic_updater::DiagnosticStatusWrapper&
      status);
    /// Retrieves parameters
    void getParameters();
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// ROS node handle
    ros::NodeHandle _nodeHandle;
    /// Front wheel speed publisher
    ros::Publisher _frontWheelsSpeedPublisher;
    /// Rear wheel speed publisher
    ros::Publisher _rearWheelsSpeedPublisher;
    /// Steering1 publisher
    ros::Publisher _steering1Publisher;
    /// Frame ID
    std::string _frameId;
    /// CAN device
    std::string _canDeviceStr;
    /// CAN connection
    std::shared_ptr<CANConnection> _canConnection;
    /// Retry timeout for CAN
    double _retryTimeout;
    /// Diagnostic updater
    diagnostic_updater::Updater _updater;
    /// Frequency diagnostic for front wheels speed
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> _fwsFreq;
    /// Front wheels speed minimum frequency
    double _fwsMinFreq;
    /// Front wheels speed maximum frequency
    double _fwsMaxFreq;
    /// Frequency diagnostic for rear wheels speed
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> _rwsFreq;
    /// Rear wheels speed minimum frequency
    double _rwsMinFreq;
    /// Rear wheels speed maximum frequency
    double _rwsMaxFreq;
    /// Frequency diagnostic for steering1
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> _st1Freq;
    /// Steering1 1minimum frequency
    double _st1MinFreq;
    /// Steering1 maximum frequency
    double _st1MaxFreq;
    /// Queue depth
    int _queueDepth;
    /// Front wheels speed packet counter
    long _fwsPacketCounter;
    /// Rear wheels speed packet counter
    long _rwsPacketCounter;
    /// Steering1 packet counter
    long _st1PacketCounter;
    /** @}
      */

  };

}

#endif // CAN_PRIUS_NODE_H
