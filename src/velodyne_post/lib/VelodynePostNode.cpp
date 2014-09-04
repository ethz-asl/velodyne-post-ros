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

#include "VelodynePostNode.h"

#include <sstream>
#include <iomanip>
#include <cstring>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <libsnappy/snappy.h>

#include <libvelodyne/sensor/DataPacket.h>
#include <libvelodyne/sensor/Calibration.h>
#include <libvelodyne/sensor/Converter.h>
#include <libvelodyne/data-structures/VdynePointCloud.h>
#include <libvelodyne/exceptions/IOException.h>

namespace velodyne {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  VelodynePostNode::VelodynePostNode(const ros::NodeHandle& nh) :
      _nodeHandle(nh),
      _pointCloudCounter(0) {
    getParameters();
    _velodyneSubscriber =
      _nodeHandle.subscribe(_velodyneCompressedDataTopicName,
      _queueDepth, &VelodynePostNode::velodyneCallback, this);
    _pointCloudPublisher = _nodeHandle.advertise<sensor_msgs::PointCloud2>(
      _pointCloudTopicName, _queueDepth);
    _dataPackets.reserve(_numDataPackets);
  }

  VelodynePostNode::~VelodynePostNode() {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void VelodynePostNode::velodyneCallback(const
      velodyne::BinarySnappyMsgConstPtr& msg) {
    std::string uncompressedData;
    snappy::Uncompress(
      reinterpret_cast<const char*>(msg->data.data()),
      msg->data.size(), &uncompressedData);
    _frameId = msg->header.frame_id;
    DataPacket dataPacket;
    std::istringstream binaryStream(uncompressedData);
    dataPacket.readBinary(binaryStream);
    _dataPackets.push_back(dataPacket);
    if (_dataPackets.size() == _numDataPackets) {
      publish();
      _dataPackets.clear();
    }
  }

  void VelodynePostNode::publish() {
    sensor_msgs::PointCloudPtr rosPointCloud(new sensor_msgs::PointCloud);
    rosPointCloud->header.stamp = ros;
    rosPointCloud->header.frame_id = _frameId;
    rosPointCloud->header.seq = _pointCloudCounter++;
//    const size_t numPoints = pointCloud.getSize();
//    rosCloud->points.reserve(numPoints);
//    rosCloud->channels.resize(1);
//    rosCloud->channels[0].name = "intensity";
//    rosCloud->channels[0].values.reserve(numPoints);
//    for (auto it = pointCloud.getPointBegin(); it != pointCloud.getPointEnd();
//        ++it) {
//      geometry_msgs::Point32 rosPoint;
//      rosPoint.x = it->mX;
//      rosPoint.y = it->mY;
//      rosPoint.z = it->mZ;
//      rosCloud->points.push_back(rosPoint);
//      rosCloud->channels[0].values.push_back(it->mIntensity);
//    }


    for (auto it = _dataPackets.cbegin(); it != _dataPackets.cend(); ++it) {
      VdynePointCloud pointCloud;
      Converter::toPointCloud(*it, *_calibration, pointCloud, _minDistance,
        _maxDistance);
    }
    sensor_msgs::PointCloud2Ptr rosPointCloud2(new sensor_msgs::PointCloud2);
    _pointCloudPublisher.publish(rosPointCloud2);
  }

//  void VelodyneNode::publishDataPacket(const ros::Time& timestamp,
//      const DataPacket& dp) {
//    VdynePointCloud pointCloud;
//    Converter::toPointCloud(dp, *_calibration, pointCloud, _minDistance,
//      _maxDistance);
//    sensor_msgs::PointCloudPtr rosCloud(new sensor_msgs::PointCloud);
//    rosCloud->header.stamp = timestamp;
//    rosCloud->header.frame_id = _frameId;
//    rosCloud->header.seq = _dataPacketCounter;
//    const size_t numPoints = pointCloud.getSize();
//    rosCloud->points.reserve(numPoints);
//    rosCloud->channels.resize(1);
//    rosCloud->channels[0].name = "intensity";
//    rosCloud->channels[0].values.reserve(numPoints);
//    for (auto it = pointCloud.getPointBegin(); it != pointCloud.getPointEnd();
//        ++it) {
//      geometry_msgs::Point32 rosPoint;
//      rosPoint.x = it->mX;
//      rosPoint.y = it->mY;
//      rosPoint.z = it->mZ;
//      rosCloud->points.push_back(rosPoint);
//      rosCloud->channels[0].values.push_back(it->mIntensity);
//    }
//    _pointCloudPublisher.publish(rosCloud);
//    velodyne::DataPacketMsgPtr dataPacketMsg(new velodyne::DataPacketMsg);
//    dataPacketMsg->header.stamp = timestamp;
//    dataPacketMsg->header.frame_id = _frameId;
//    dataPacketMsg->header.seq = _dataPacketCounter;
//    for (size_t i = 0; i < DataPacket::mDataChunkNbr; ++i) {
//      const DataPacket::DataChunk& dataChunk = dp.getDataChunk(i);
//      dataPacketMsg->dataChunks[i].headerInfo = dataChunk.mHeaderInfo;
//      dataPacketMsg->dataChunks[i].rotationalInfo = dataChunk.mRotationalInfo;
//      for (size_t j = 0; j < DataPacket::DataChunk::mLasersPerPacket; ++j) {
//        dataPacketMsg->dataChunks[i].laserData[j].distance =
//          dataChunk.mLaserData[j].mDistance;
//        dataPacketMsg->dataChunks[i].laserData[j].intensity =
//          dataChunk.mLaserData[j].mIntensity;
//      }
//    }
//    _dataPacketPublisher.publish(dataPacketMsg);
//    velodyne::BinarySnappyMsgPtr binarySnappyMsg(new velodyne::BinarySnappyMsg);
//    binarySnappyMsg->header.stamp = timestamp;
//    binarySnappyMsg->header.frame_id = _frameId;
//    binarySnappyMsg->header.seq = _dataPacketCounter++;
//    std::ostringstream binaryStream;
//    dp.writeBinary(binaryStream);
//    std::string binaryStreamSnappy;
//    snappy::Compress(binaryStream.str().data(),
//      binaryStream.str().size(), &binaryStreamSnappy);
//    binarySnappyMsg->data.resize(binaryStreamSnappy.size());
//    std::copy(binaryStreamSnappy.begin(), binaryStreamSnappy.end(),
//      binarySnappyMsg->data.begin());
//    _binarySnappyPublisher.publish(binarySnappyMsg);
//    _dpFreq->tick();
//  }

  void VelodynePostNode::spin() {
    std::ifstream calibFile(_calibFileName);
    _calibration.reset(new Calibration());
    try {
      calibFile >> *_calibration;
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM("IOException: " << e.what());
    }
    ros::spin();
  }

  void VelodynePostNode::getParameters() {
    _nodeHandle.param<double>("sensor/min_distance", _minDistance, 0.9);
    _nodeHandle.param<double>("sensor/max_distance", _maxDistance, 120);
    _nodeHandle.param<std::string>("sensor/device_name", _deviceName,
      "Velodyne HDL-32E");
    if (_deviceName == "Velodyne HDL-64E S2")
      _nodeHandle.param<std::string>("sensor/calibration_file", _calibFileName,
        "conf/calib-HDL-64E.dat");
    else if (_deviceName == "Velodyne HDL-32E")
      _nodeHandle.param<std::string>("sensor/calibration_file", _calibFileName,
        "conf/calib-HDL-32E.dat");
    else
      ROS_ERROR_STREAM("Unknown device: " << _deviceName);
    _nodeHandle.param<std::string>("ros/velodyne_compressed_data_topic_name",
      _velodyneCompressedDataTopicName, "/velodyne/binary_snappy");
    _nodeHandle.param<std::string>("ros/point_cloud_topic_name",
      _pointCloudTopicName, "point_cloud");
    _nodeHandle.param<int>("ros/queue_depth", _queueDepth, 100);
    if (_deviceName == "Velodyne HDL-64E S2")
      _nodeHandle.param<int>("ros/num_data_packets", _numDataPackets, 348);
    else if (_deviceName == "Velodyne HDL-32E")
      _nodeHandle.param<int>("ros/num_data_packets", _numDataPackets, 174);
  }

}
