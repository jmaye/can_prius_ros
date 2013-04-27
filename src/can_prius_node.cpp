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

/** \file can_prius_node.cpp
    \brief This file is the ROS node for CAN Prius.
  */

#include <ros/ros.h>

namespace canprius {

  /** The class CANPriusNode implements the CAN Prius node.
      \brief CAN PRIUS node
    */
  class CANPriusNode {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructor with node handle
    CANPriusNode(const ros::NodeHandle& nh);
    /// Destructor
    ~CANPriusNode();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Spin once
    void spin();
    /** @}
      */

  private:
    /** \name Private methods
      @{
      */
//    void publishVehicleNavigationSolution(const ros::Time & stamp, const VehicleNavigationSolution & vn);
//    void publishVehicleNavigationPerformance(const ros::Time & stamp, const VehicleNavigationPerformance & vn);
    /** @}
      */

    /** \name Private members
      @{
      */
    /// ROS node handle
    ros::NodeHandle _nodeHandle;
//    ros::Publisher mVehicleNavigationSolutionPublisher;
//    ros::Publisher mVehicleNavigationPerformancePublisher;
    /// Frame ID
    std::string _frameId;
    /** @}
      */

  };

  CANPriusNode::CANPriusNode(const ros::NodeHandle& nh) :
      _nodeHandle(nh) {
//    _nodeHandle.param<std::string>("ip_address", mIp, "129.132.39.171");
//    _nodeHandle.param<std::string>("frame_id", _frameId, "vehicle_base_link");
//    int queueDepth = 100;
//    mVehicleNavigationSolutionPublisher = mNodeHandle.advertise<applanix::VehicleNavigationSolutionMsg>("vehicle_navigation_solution",queueDepth);
//    mVehicleNavigationPerformancePublisher = mNodeHandle.advertise<applanix::VehicleNavigationPerformanceMsg>("vehicle_navigation_performance",queueDepth);
  }

  CANPriusNode::~CANPriusNode() {
  }

//  void CANPriusNode::publishVehicleNavigationPerformance(const ros::Time & stamp, const ::VehicleNavigationPerformance & vn) {
//    boost::shared_ptr<applanix::VehicleNavigationPerformanceMsg> navMsg(new applanix::VehicleNavigationPerformanceMsg);
//    navMsg->header.stamp = stamp;
//    navMsg->header.frame_id = mFrameId;

//    /// Time 1
//    navMsg->TimeDistance.Time1 = vn.mTimeDistance.mTime1;
//    /// Time 2 
//    navMsg->TimeDistance.Time2 = vn.mTimeDistance.mTime2;
//    /// Distance tag
//    navMsg->TimeDistance.DistanceTag = vn.mTimeDistance.mDistanceTag;
//    /// Time type
//    navMsg->TimeDistance.TimeType = vn.mTimeDistance.mTimeType;
//    /// Distance type
//    navMsg->TimeDistance.DistanceType = vn.mTimeDistance.mDistanceType;

//    
//    /// North position RMS error
//    navMsg->NorthPositionRMSError = vn.mNorthPositionRMSError;
//    /// East position RMS error
//    navMsg->EastPositionRMSError = vn.mEastPositionRMSError;
//    /// Down position RMS error
//    navMsg->DownPositionRMSError = vn.mDownPositionRMSError;
//    /// North velocity RMS error
//    navMsg->NorthVelocityRMSError = vn.mNorthVelocityRMSError;
//    /// East velocity RMS error
//    navMsg->EastVelocityRMSError = vn.mEastVelocityRMSError;
//    /// Down velocity RMS error
//    navMsg->DownVelocityRMSError = vn.mDownVelocityRMSError;
//    /// Roll RMS error
//    navMsg->RollRMSError = vn.mRollRMSError;
//    /// Pitch RMS error
//    navMsg->PitchRMSError = vn.mPitchRMSError;
//    /// Heading RMS error
//    navMsg->HeadingRMSError = vn.mHeadingRMSError;
//    /// Error ellipsoid semi major
//    navMsg->ErrorEllipsoidSemiMajor = vn.mErrorEllipsoidSemiMajor;
//    /// Error ellipsoid semi minor
//    navMsg->ErrorEllipsoidSemiMinor = vn.mErrorEllipsoidSemiMinor;
//    /// Error ellipsoid orientation
//    navMsg->ErrorEllipsoidOrientation = vn.mErrorEllipsoidOrientation;

//    mVehicleNavigationPerformancePublisher.publish(navMsg);

//  }

  void CANPriusNode::spin() {
//    TCPConnectionClient connection(mIp, mPort);
//    POSLVComTCP device(connection);
    while (ros::ok()) {
      try {
//        std::shared_ptr<Packet> packet = device.readPacket();
//        ros::Time stamp = ros::Time::now();
//        if (packet == NULL) 
//          {
//            ROS_DEBUG("Dropping message...");
//          }
//        else
//          {
//            const Group& group = packet->groupCast();
//            if (group.instanceOf<VehicleNavigationSolution>())
//              {
//                const VehicleNavigationSolution& vn =
//                  group.typeCast<VehicleNavigationSolution>();
//                publishVehicleNavigationSolution(stamp, vn);
//              }
//            else if (group.instanceOf<VehicleNavigationPerformance>())
//              {
//                const VehicleNavigationPerformance& vn =
//                  group.typeCast<VehicleNavigationPerformance>();
//                publishVehicleNavigationPerformance(stamp, vn);
//              }
//          }
      } 
      catch (const IOException& e) {
        ROS_WARN_STREAM("IO Exception: " << e.what() << ". Attempting to continue");
      }
      ros::spinOnce();
    }
  }

}

int main(int argc, char** argv) {
  ros::init(argc,argv,"can_prius");
  ros::NodeHandle nh("~");
  int returnValue = 0;
  try {
    canprius::CANPriusNode cn(nh);
    cn.spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    returnValue = -1;
  }
  catch (...) {
      ROS_ERROR_STREAM("Unknown Exception");
      returnValue = -2;
    }
  return returnValue;
}
