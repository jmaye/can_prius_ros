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

#include "janeth/CANPriusNode.h"

#include <memory>

#include <boost/shared_ptr.hpp>

#include <libcan-prius/com/CANConnection.h>
#include <libcan-prius/sensor/PRIUSReader.h>
#include <libcan-prius/types/PRIUSMessage.h>
#include <libcan-prius/types/FrontWheelsSpeed.h>
#include <libcan-prius/types/RearWheelsSpeed.h>
#include <libcan-prius/types/Speed1.h>
#include <libcan-prius/types/Speed2.h>
#include <libcan-prius/types/Speed3.h>
#include <libcan-prius/types/Steering1.h>
#include <libcan-prius/types/Steering2.h>
#include <libcan-prius/types/Brakes.h>
#include <libcan-prius/types/Acceleration1.h>
#include <libcan-prius/types/Acceleration2.h>
#include <libcan-prius/exceptions/IOException.h>

#include "can_prius_ros/FrontWheelsSpeedMsg.h"
#include "can_prius_ros/RearWheelsSpeedMsg.h"
#include "can_prius_ros/Steering1Msg.h"

namespace janeth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  CANPriusNode::CANPriusNode(const ros::NodeHandle& nh) :
      _nodeHandle(nh) {
    _nodeHandle.param<std::string>("frame_id", _frameId,
      "vehicle_odometry_link");
    _nodeHandle.param<std::string>("can_device", _canDevice, "/dev/cpc_usb_0");
    const int queueDepth = 100;
    _frontWheelsSpeedPublisher =
      _nodeHandle.advertise<can_prius_ros::FrontWheelsSpeedMsg>(
      "front_wheels_speed", queueDepth);
    _rearWheelsSpeedPublisher =
      _nodeHandle.advertise<can_prius_ros::RearWheelsSpeedMsg>(
      "rear_wheels_speed", queueDepth);
    _steering1Publisher =
      _nodeHandle.advertise<can_prius_ros::Steering1Msg>(
      "steering1", queueDepth);
  }

  CANPriusNode::~CANPriusNode() {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void CANPriusNode::publishFrontWheelsSpeed(const ros::Time& timestamp,
      const FrontWheelsSpeed& fws) {
    boost::shared_ptr<can_prius_ros::FrontWheelsSpeedMsg> fwsMsg(
      new can_prius_ros::FrontWheelsSpeedMsg);
    fwsMsg->header.stamp = timestamp;
    fwsMsg->header.frame_id = _frameId;
    fwsMsg->Right = fws.mRight;
    fwsMsg->Left = fws.mLeft;
    _frontWheelsSpeedPublisher.publish(fwsMsg);
  }

  void CANPriusNode::publishRearWheelsSpeed(const ros::Time& timestamp,
      const RearWheelsSpeed& rws) {
    boost::shared_ptr<can_prius_ros::RearWheelsSpeedMsg> rwsMsg(
      new can_prius_ros::RearWheelsSpeedMsg);
    rwsMsg->header.stamp = timestamp;
    rwsMsg->header.frame_id = _frameId;
    rwsMsg->Right = rws.mRight;
    rwsMsg->Left = rws.mLeft;
    _rearWheelsSpeedPublisher.publish(rwsMsg);
  }

  void CANPriusNode::publishSteering1(const ros::Time& timestamp,
      const Steering1& st) {
    boost::shared_ptr<can_prius_ros::Steering1Msg> stMsg(
      new can_prius_ros::Steering1Msg);
    stMsg->header.stamp = timestamp;
    stMsg->header.frame_id = _frameId;
    stMsg->value = st.mValue;
    _steering1Publisher.publish(stMsg);
  }

  void CANPriusNode::spin() {
    CANConnection device(_canDevice);
    PRIUSReader reader(device);
    while (_nodeHandle.ok()) {
      try {
        std::shared_ptr<PRIUSMessage> message = reader.readMessage();
        const ros::Time timestamp = ros::Time::now();
        if (message->instanceOf<FrontWheelsSpeed>()) {
          const FrontWheelsSpeed& fws = message->typeCast<FrontWheelsSpeed>();
          publishFrontWheelsSpeed(timestamp, fws);
        }
        else if (message->instanceOf<RearWheelsSpeed>()) {
          const RearWheelsSpeed& rws = message->typeCast<RearWheelsSpeed>();
          publishRearWheelsSpeed(timestamp, rws);
        }
        else if (message->instanceOf<Speed1>()) {
          const Speed1& sp = message->typeCast<Speed1>();
        }
        else if (message->instanceOf<Speed2>()) {
          const Speed2& sp = message->typeCast<Speed2>();
        }
        else if (message->instanceOf<Speed3>()) {
          const Speed3& sp = message->typeCast<Speed3>();
        }
        else if (message->instanceOf<Steering1>()) {
          const Steering1& st = message->typeCast<Steering1>();
          publishSteering1(timestamp, st);
        }
        else if (message->instanceOf<Steering2>()) {
          const Steering2& st = message->typeCast<Steering2>();
        }
        else if (message->instanceOf<Brakes>()) {
          const Brakes& b = message->typeCast<Brakes>();
        }
        else if (message->instanceOf<Acceleration1>()) {
          const Acceleration1& a = message->typeCast<Acceleration1>();
        }
        else if (message->instanceOf<Acceleration2>()) {
          const Acceleration2& a = message->typeCast<Acceleration2>();
        }
      } 
      catch (const IOException& e) {
        ROS_WARN_STREAM("IO Exception: " << e.what()
          << ". Attempting to continue");
      }
      ros::spinOnce();
    }
  }

}
