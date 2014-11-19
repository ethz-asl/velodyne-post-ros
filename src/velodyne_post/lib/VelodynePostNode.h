/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
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

/** \file VelodynePostNode.h
    \brief This file defines the VelodynePostNode class which implements the
           Velodyne post-processing node.
  */

#ifndef VELODYNE_POST_NODE_H
#define VELODYNE_POST_NODE_H

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <velodyne/BinarySnappyMsg.h>
#include <velodyne/DataPacketMsg.h>

class Calibration;
class DataPacket;

namespace velodyne {

  /** The class VelodynePostNode implements the Velodyne post-processing node.
      \brief Velodyne post-processing node
    */
  class VelodynePostNode {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructor
    VelodynePostNode(const ros::NodeHandle& nh);
    /// Copy constructor
    VelodynePostNode(const VelodynePostNode& other) = delete;
    /// Copy assignment operator
    VelodynePostNode& operator = (const VelodynePostNode& other) = delete;
    /// Move constructor
    VelodynePostNode(VelodynePostNode&& other) = delete;
    /// Move assignment operator
    VelodynePostNode& operator = (VelodynePostNode&& other) = delete;
    /// Destructor
    virtual ~VelodynePostNode();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Spin
    void spin();
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Velodyne callback for binary snappy
    void velodyneBinarySnappyCallback(const velodyne::BinarySnappyMsgConstPtr&
      msg);
    /// Velodyne callback for data packet
    void velodyneDataPacketCallback(const velodyne::DataPacketMsgConstPtr&
      msg);
    /// Retrieves parameters
    void getParameters();
    /// Update subscription (subscribe only when subscriber is around)
    void updateSubscription(const ros::TimerEvent& event);
    /// Publishes the currently stored data
    void publish();
    /// Inits the subscribers
    void initSubscribers();
    /// Shutdowns the subscribers
    void shutdownSubscribers();
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// ROS node handle
    ros::NodeHandle _nodeHandle;
    /// Velodyne binary snappy subscriber
    ros::Subscriber _velodyneBinarySnappySubscriber;
    /// Velodyne data packet subscriber
    ros::Subscriber _velodyneDataPacketSubscriber;
    /// Velodyne calibration
    std::shared_ptr<Calibration> _calibration;
    /// Calibration file name
    std::string _calibFileName;
    /// Device name
    std::string _deviceName;
    /// Min distance for conversions
    double _minDistance;
    /// Max distance for conversions
    double _maxDistance;
    /// Velodyne binary snappy topic name
    std::string _velodyneBinarySnappyTopicName;
    /// Velodyne data packet topic name
    std::string _velodyneDataPacketTopicName;
    /// Vector for queuing the data packets
    std::vector<DataPacket> _dataPackets;
    /// Queue size for receiving messages
    int _queueDepth;
    /// Number of data packets to accumulate before publishing
    int _numDataPackets;
    /// Point cloud publisher
    ros::Publisher _pointCloudPublisher;
    /// Point cloud topic name
    std::string _pointCloudTopicName;
    /// Frame ID
    std::string _frameId;
    /// Point cloud counter
    long _pointCloudCounter;
    /// Use binary snappy
    bool _useBinarySnappy;
    /// Transport hints
    ros::TransportHints _transportHints;
    /// Transport type (tcp or udp)
    std::string _transportType;
    /// Flag for active subscribtions
    bool _subscriptionIsActive;
    /// Update subscription callback timer
    ros::Timer _timer;
    /** @}
      */

  };

}

#endif // VELODYNE_POST_NODE_H
