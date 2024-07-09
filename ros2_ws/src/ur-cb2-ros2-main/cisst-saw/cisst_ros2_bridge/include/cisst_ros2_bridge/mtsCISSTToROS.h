/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-21

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsCISSTToROS_h
#define _mtsCISSTToROS_h

// cisst include
#include <cisstVector/vctDynamicVectorTypes.h>

#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsVector.h>
#include <cisstMultiTask/mtsTransformationTypes.h>
#include <cisstMultiTask/mtsIntervalStatistics.h>

#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianArrayGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmInputData.h>
#include <cisstParameterTypes/prmKeyValue.h>
#include <cisstParameterTypes/prmOperatingState.h>

// ros include
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_srvs/srv/trigger.hpp>

// non standard messages
#include <cisst_msgs/msg/double_vec.hpp>
#include <cisst_msgs/msg/interval_statistics.hpp>
#include <cisst_msgs/msg/bool_stamped.hpp>
#include <cisst_msgs/srv/convert_float64_array.hpp>

namespace mts_cisst_to_ros {

    // cases for automatic header conversions
    // -4- ROS has header and child_frame_id, cisst has valid, timestamp, reference frame and moving frame
    // -3- ROS has header, cisst has valid, timestamp and reference frame
    // -2- ROS has header, cisst has valid, timestamp,
    // -1- ROS has header, cisst nothing
    // -0- ROS has no header, cisst has nothing

    template <typename _cisstType, typename _rosType>
    void cisst_header_to_ros_header(const _cisstType & cisstData, _rosType & rosData,
                                    std::shared_ptr<rclcpp::Node> node,
                                    const std::string & debugInfo)
    {
        try {
            const double cisstDataTime = cisstData.Timestamp();
            if (cisstDataTime > 0.0) {
                const double age =
                    mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime()
                    - cisstDataTime;
                if (age > 0.0) {
                    rosData.header.stamp = node->get_clock()->now() - rclcpp::Duration::from_seconds(age);
                } else {
                    rosData.header.stamp = node->get_clock()->now();
                }
            } else {
                rosData.header.stamp = node->get_clock()->now();
            }
        } catch (std::exception & e) {
            CMN_LOG_RUN_ERROR << "mtsCISSTToROSHeader caught exception \""
                              << e.what()
                              << "\"while computing timestamp for \"" << debugInfo
                              << "\"" << std::endl;
        }
    }

    // mts_ros_to_ros_header_choice<N> is preferred to
    // mts_ros_to_ros_header_choice<N-1>, but overload resolution will
    // fallback to mts_ros_to_ros_header_choice<N-1> (then
    // mts_ros_to_ros_header_choice<N-2> etc.) since it's a base class
    template<std::size_t _n>
    class header_choice: public header_choice<_n-1> {};
    template<>
    class header_choice<0> {};

    template <typename _cisstType, typename _rosType>
    auto header_impl(header_choice<4>,
                     const _cisstType & cisstData,
                     _rosType & rosData,
                     std::shared_ptr<rclcpp::Node> node,
                     const std::string & debugInfo)
        -> decltype(cisstData.Valid(),
                    cisstData.Timestamp(),
                    cisstData.ReferenceFrame(),
                    cisstData.MovingFrame(),
                    rosData.header,
                    rosData.child_frame_id,
                    false)
    {
        if (!cisstData.Valid()) {
            return false;
        }
        cisst_header_to_ros_header<_cisstType, _rosType>(cisstData, rosData, node, debugInfo);
        // set reference frame name
        rosData.header.frame_id = cisstData.ReferenceFrame();
        rosData.child_frame_id = cisstData.MovingFrame();
        return true;
    }

    template <typename _cisstType, typename _rosType>
    auto header_impl(header_choice<3>,
                     const _cisstType & cisstData,
                     _rosType & rosData,
                     std::shared_ptr<rclcpp::Node> node,
                     const std::string & debugInfo)
        -> decltype(cisstData.Valid(),
                    cisstData.Timestamp(),
                    cisstData.ReferenceFrame(),
                    rosData.header,
                    false)
    {
        if (!cisstData.Valid()) {
            return false;
        }
        cisst_header_to_ros_header<_cisstType, _rosType>(cisstData, rosData, node, debugInfo);
        // set reference frame name
        rosData.header.frame_id = cisstData.ReferenceFrame();
        return true;
    }

    template <typename _cisstType, typename _rosType>
    auto header_impl(header_choice<2>,
                     const _cisstType & cisstData,
                     _rosType & rosData,
                     std::shared_ptr<rclcpp::Node> node,
                     const std::string & debugInfo)
        -> decltype(cisstData.Valid(),
                    cisstData.Timestamp(),
                    rosData.header,
                    false)
    {
        if (!cisstData.Valid()) {
            return false;
        }
        cisst_header_to_ros_header<_cisstType, _rosType>(cisstData, rosData, node, debugInfo);
        return true;
    }

    template <typename _cisstType, typename _rosType>
    auto header_impl(header_choice<1>,
                     const _cisstType &,
                     _rosType & rosData,
                     std::shared_ptr<rclcpp::Node> node,
                     const std::string &)
        -> decltype(rosData.header,
                    false)
    {
        rosData.header.stamp = node->get_clock()->now();
        return true;
    }

    // last case, nothing to do
    template <typename _cisstType, typename _rosType>
    bool header_impl(header_choice<0>,
                     const _cisstType &,
                     _rosType &,
                     std::shared_ptr<rclcpp::Node>,
                     const std::string &)
    {
        return true;
    }

    template <typename _cisstType, typename _rosType>
    bool header(const _cisstType & cisstData, _rosType & rosData,
                std::shared_ptr<rclcpp::Node> node,
                const std::string & debugInfo)
    {
        return header_impl<_cisstType, _rosType>(header_choice<4>(), cisstData, rosData, node, debugInfo);
    }
}


// helper functions
template <typename _cisstFrame, typename _rosPose>
void mtsCISSTToROSPose(const _cisstFrame & cisstFrame, _rosPose & rosPose)
{
    vctQuatRot3 quat(cisstFrame.Rotation(), VCT_NORMALIZE);
    rosPose.orientation.x = quat.X();
    rosPose.orientation.y = quat.Y();
    rosPose.orientation.z = quat.Z();
    rosPose.orientation.w = quat.W();
    rosPose.position.x = cisstFrame.Translation().X();
    rosPose.position.y = cisstFrame.Translation().Y();
    rosPose.position.z = cisstFrame.Translation().Z();
}

template <typename _cisstFrame, typename _rosTransform>
void mtsCISSTToROSTransform(const _cisstFrame & cisstFrame, _rosTransform & rosTransform)
{
    vctQuatRot3 quat(cisstFrame.Rotation(), VCT_NORMALIZE);
    rosTransform.rotation.x = quat.X();
    rosTransform.rotation.y = quat.Y();
    rosTransform.rotation.z = quat.Z();
    rosTransform.rotation.w = quat.W();
    rosTransform.translation.x = cisstFrame.Translation().X();
    rosTransform.translation.y = cisstFrame.Translation().Y();
    rosTransform.translation.z = cisstFrame.Translation().Z();
}

template <typename _cisstVector, typename _rosWrench>
void mtsCISSTToROSWrench(const _cisstVector & cisstVector, _rosWrench & rosWrench)
{
    rosWrench.force.x = cisstVector.Element(0);
    rosWrench.force.y = cisstVector.Element(1);
    rosWrench.force.z = cisstVector.Element(2);
    rosWrench.torque.x = cisstVector.Element(3);
    rosWrench.torque.y = cisstVector.Element(4);
    rosWrench.torque.z = cisstVector.Element(5);
}


// std_msgs
void mtsCISSTToROS(const double & cisstData, std_msgs::msg::Float32 & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const double & cisstData, std_msgs::msg::Float64 & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const int & cisstData, std_msgs::msg::Int32 & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const bool & cisstData, std_msgs::msg::Bool & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const bool & cisstData, cisst_msgs::msg::BoolStamped & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const bool & cisstData, sensor_msgs::msg::Joy & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const std::string & cisstData, std_msgs::msg::String & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsMessage & cisstData, std_msgs::msg::String & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmEventButton & cisstData, std_msgs::msg::Bool & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmEventButton & cisstData, cisst_msgs::msg::BoolStamped & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmEventButton & cisstData, sensor_msgs::msg::Joy & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctDoubleVec & cisstData, std_msgs::msg::Float64MultiArray & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctDoubleMat & cisstData, std_msgs::msg::Float64MultiArray & rosData,
                   const std::string & debugInfo);

// geometry_msgs
void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::msg::Transform & rosData
                   ,  const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::msg::TransformStamped & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::msg::Pose & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianGet & cisstData, geometry_msgs::msg::PoseStamped & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianArrayGet & cisstData, geometry_msgs::msg::PoseArray & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianSet & cisstData, geometry_msgs::msg::Pose & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianSet & cisstData, geometry_msgs::msg::PoseStamped & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::msg::Pose & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::msg::Pose & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::msg::Pose & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctFrm4x4 & cisstData, geometry_msgs::msg::Transform & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::msg::Transform & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionCartesianSet & cisstData, geometry_msgs::msg::TransformStamped & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsFrm4x4 & cisstData, geometry_msgs::msg::TransformStamped & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctFrm3 & cisstData, geometry_msgs::msg::Transform & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vct3 & cisstData, geometry_msgs::msg::Vector3 & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctMatRot3 & cisstData, geometry_msgs::msg::Quaternion & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctMatRot3 & cisstData, geometry_msgs::msg::QuaternionStamped & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vct6 & cisstData, geometry_msgs::msg::Wrench & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vct6 & cisstData, geometry_msgs::msg::WrenchStamped & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::msg::Wrench & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::msg::WrenchStamped & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsDoubleVec & cisstData, geometry_msgs::msg::Vector3Stamped & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::msg::Twist & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmVelocityCartesianGet & cisstData, geometry_msgs::msg::TwistStamped & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmForceCartesianGet & cisstData, geometry_msgs::msg::Wrench & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmForceCartesianGet & cisstData, geometry_msgs::msg::WrenchStamped & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmForceCartesianSet & cisstData, geometry_msgs::msg::Wrench & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmForceCartesianSet & cisstData, geometry_msgs::msg::WrenchStamped & rosData,
                   const std::string & debugInfo);

// sensor_msgs
void mtsCISSTToROS(const vctDoubleVec & cisstData, sensor_msgs::msg::JointState & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionJointGet & cisstData, sensor_msgs::msg::JointState & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmPositionJointSet & cisstData, sensor_msgs::msg::JointState & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmVelocityJointGet & cisstData, sensor_msgs::msg::JointState & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmForceTorqueJointSet & cisstData, sensor_msgs::msg::JointState & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmStateJoint & cisstData, sensor_msgs::msg::JointState & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctDoubleMat & cisstData, sensor_msgs::msg::PointCloud & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const std::vector<vct3> & cisstData, sensor_msgs::msg::PointCloud & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const prmInputData & cisstData, sensor_msgs::msg::Joy & rosData,
                   const std::string & debugInfo);

// diagnostic_msgs
void mtsCISSTToROS(const prmKeyValue & cisstData, diagnostic_msgs::msg::KeyValue & rosData,
                   const std::string & debugInfo);

// std_srvs
void mtsCISSTToROS(const bool & cisstData, std_srvs::srv::Trigger::Response & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const std::string & cisstData, std_srvs::srv::Trigger::Response & rosData,
                   const std::string & debugInfo);

// cisst_msgs
void mtsCISSTToROS(const prmPositionJointGet & cisstData, cisst_msgs::msg::DoubleVec & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctDoubleVec & cisstData, cisst_msgs::msg::DoubleVec & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const mtsIntervalStatistics & cisstData, cisst_msgs::msg::IntervalStatistics & rosData,
                   const std::string & debugInfo);
void mtsCISSTToROS(const vctDoubleVec & cisstData, cisst_msgs::srv::ConvertFloat64Array::Response & rosData,
                   const std::string & debugInfo);

#endif // _mtsCISSTToROS_h
