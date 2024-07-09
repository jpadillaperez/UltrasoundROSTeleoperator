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

#ifndef _mtsROSToCISST_h
#define _mtsROSToCISST_h

// cisst include
#include <cisstMultiTask/mtsManagerLocal.h>

#include <cisstVector/vctDynamicVectorTypes.h>

#include <cisstMultiTask/mtsVector.h>
#include <cisstMultiTask/mtsTransformationTypes.h>
#include <cisstMultiTask/mtsIntervalStatistics.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstParameterTypes/prmVelocityCartesianSet.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmInputData.h>
#include <cisstParameterTypes/prmKeyValue.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmOperatingState.h>

// ros include
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

// non standard messages
#include <cisst_msgs/msg/double_vec.hpp>
#include <cisst_msgs/msg/interval_statistics.hpp>
#include <cisst_msgs/srv/convert_float64_array.hpp>

namespace mts_ros_to_cisst {

    // cases for automatic header conversions
    // -4- ROS has header and child_frame_id, cisst has timestamp, valid, reference frame and moving frame
    // -3- ROS has header, cisst has timestamp, valid and reference frame
    // -2- ROS has header, cisst has timestamp, valid
    // -1- ROS has no header, cisst has timestamp, valid
    // -0- ROS has no header, cisst has nothing

    template <typename _rosType, typename _cisstType>
    void ros_header_to_cisst_header(const _rosType & rosData,
                                    _cisstType & cisstData,
                                    std::shared_ptr<rclcpp::Node> node)
    {
        const double cisstNow = mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime();
        // first check that header.stamp is not zero
        if ((rosData.header.stamp.sec == 0)
            && (rosData.header.stamp.nanosec == 0)) {
            cisstData.SetTimestamp(cisstNow);
        } else {
            const double ageInSeconds = (node->get_clock()->now() - rosData.header.stamp).seconds();
            if (ageInSeconds > 0.0) {
                cisstData.SetTimestamp(cisstNow - ageInSeconds);
            } else {
                cisstData.SetTimestamp(cisstNow);
            }
        }
        // always set as valid for now
        cisstData.SetValid(true);
    }

    // mts_ros_to_cisst_header_choice<N> is preferred to
    // mts_ros_to_cisst_header_choice<N-1>, but overload resolution will
    // fallback to mts_ros_to_cisst_header_choice<N-1> (then
    // mts_ros_to_cisst_header_choice<N-2> etc.) since it's a base class
    template<std::size_t _n>
    class header_choice: public header_choice<_n-1> {};
    template<>
    class header_choice<0> {};

    template <typename _rosType, typename _cisstType>
    auto header_impl(header_choice<4>,
                     const _rosType & rosData,
                     _cisstType & cisstData,
                     std::shared_ptr<rclcpp::Node> node)
        -> decltype(rosData.header,
                    rosData.child_frame_id,
                    cisstData.SetTimestamp(0.0),
                    cisstData.SetValid(true),
                    cisstData.SetReferenceFrame(""),
                    cisstData.SetMovingFrame(""),
                    void())
    {
        ros_header_to_cisst_header<_rosType, _cisstType>(rosData, cisstData, node);
        // set reference frame name
        cisstData.SetReferenceFrame(rosData.header.frame_id);
        cisstData.SetMovingFrame(rosData.child_frame_id);
    }

    template <typename _rosType, typename _cisstType>
    auto header_impl(header_choice<3>,
                     const _rosType & rosData,
                     _cisstType & cisstData,
                     std::shared_ptr<rclcpp::Node> node)
        -> decltype(rosData.header,
                    cisstData.SetTimestamp(0.0),
                    cisstData.SetValid(true),
                    cisstData.SetReferenceFrame(""),
                    void())
    {
        ros_header_to_cisst_header<_rosType, _cisstType>(rosData, cisstData, node);
        // set reference frame name
        cisstData.SetReferenceFrame(rosData.header.frame_id);
    }

    template <typename _rosType, typename _cisstType>
    auto header_impl(header_choice<2>,
                     const _rosType & rosData,
                     _cisstType & cisstData,
                     std::shared_ptr<rclcpp::Node> node)
        -> decltype(rosData.header,
                    cisstData.SetTimestamp(0.0),
                    cisstData.SetValid(true),
                    void())
    {
        ros_header_to_cisst_header<_rosType, _cisstType>(rosData, cisstData, node);
    }

    template <typename _rosType, typename _cisstType>
    auto header_impl(header_choice<1>,
                     const _rosType &,
                     _cisstType & cisstData,
                     std::shared_ptr<rclcpp::Node> node)
        -> decltype(cisstData.SetTimestamp(0.0),
                    cisstData.SetValid(true),
                    void())
    {
        const double cisstNow = mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime();
        cisstData.SetTimestamp(cisstNow);
        // always set as valid for now
        cisstData.SetValid(true);
    }

    // last case, nothing to do
    template <typename _rosType, typename _cisstType>
    void header_impl(header_choice<0>,
                     const _rosType &,
                     _cisstType &,
                     std::shared_ptr<rclcpp::Node>)
    {
    }

    template <typename _rosType, typename _cisstType>
    void header(const _rosType & rosData, _cisstType & cisstData, std::shared_ptr<rclcpp::Node> node)
    {
        header_impl<_rosType, _cisstType>(header_choice<4>(), rosData, cisstData, node);
    }
}


// helper functions
template <typename _cisstFrame>
void mtsROSTransformToCISST(const geometry_msgs::msg::Transform & rosTransform, _cisstFrame & cisstFrame)
{
    cisstFrame.Translation().X() = rosTransform.translation.x;
    cisstFrame.Translation().Y() = rosTransform.translation.y;
    cisstFrame.Translation().Z() = rosTransform.translation.z;
    vctQuatRot3 quat;
    quat.X() = rosTransform.rotation.x;
    quat.Y() = rosTransform.rotation.y;
    quat.Z() = rosTransform.rotation.z;
    quat.W() = rosTransform.rotation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstFrame.Rotation().Assign(rotation);
}

template <typename _cisstFrame>
void mtsROSPoseToCISST(const geometry_msgs::msg::Pose & rosPose, _cisstFrame & cisstFrame)
{
    cisstFrame.Translation().X() = rosPose.position.x;
    cisstFrame.Translation().Y() = rosPose.position.y;
    cisstFrame.Translation().Z() = rosPose.position.z;
    vctQuatRot3 quat;
    quat.X() = rosPose.orientation.x;
    quat.Y() = rosPose.orientation.y;
    quat.Z() = rosPose.orientation.z;
    quat.W() = rosPose.orientation.w;
    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    cisstFrame.Rotation().Assign(rotation);
}


// std_msgs
void mtsROSToCISST(const std_msgs::msg::Float32 & rosData, double & cisstData);
void mtsROSToCISST(const std_msgs::msg::Float64 & rosData, double & cisstData);
void mtsROSToCISST(const std_msgs::msg::Int32 & rosData, int & cisstData);
void mtsROSToCISST(const std_msgs::msg::Bool & rosData, bool & cisstData);
void mtsROSToCISST(const std_msgs::msg::String & rosData, std::string & cisstData);
void mtsROSToCISST(const std_msgs::msg::String & rosData, mtsMessage & cisstData);
void mtsROSToCISST(const std_msgs::msg::Float64MultiArray & rosData, vctDoubleVec & cisstData);
void mtsROSToCISST(const std_msgs::msg::Float64MultiArray & rosData, vctDoubleMat & cisstData);

// geometry_msgs
void mtsROSToCISST(const geometry_msgs::msg::Vector3 & rosData, vct3 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Quaternion & rosData, vctMatRot3 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, prmPositionCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, prmPositionCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::TransformStamped & rosData, prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::TransformStamped & rosData, prmPositionCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, vctFrm3 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, vctFrm3 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::PoseStamped & rosData, vctFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, vctFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Pose & rosData, mtsFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, prmPositionCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, vctFrm3 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, vctFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Transform & rosData, mtsFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::TransformStamped & rosData, mtsFrm4x4 & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Wrench & rosData, prmForceCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::WrenchStamped & rosData, prmForceCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Wrench & rosData, prmForceCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::WrenchStamped & rosData, prmForceCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Wrench & rosData, mtsDoubleVec & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::WrenchStamped & rosData, mtsDoubleVec & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Twist & rosData, prmVelocityCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::TwistStamped & rosData, prmVelocityCartesianGet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::Twist & rosData, prmVelocityCartesianSet & cisstData);
void mtsROSToCISST(const geometry_msgs::msg::TwistStamped & rosData, prmVelocityCartesianSet & cisstData);

// sensor_msgs
void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmPositionJointSet & cisstData);
void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmForceTorqueJointSet & cisstData);
void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmVelocityJointSet & cisstData);
void mtsROSToCISST(const sensor_msgs::msg::JointState & rosData, prmStateJoint & cisstData);
void mtsROSToCISST(const sensor_msgs::msg::Joy & rosData, prmEventButton & cisstData);
void mtsROSToCISST(const sensor_msgs::msg::Joy & rosData, prmInputData & cisstData);

// diagnostic_msgs
void mtsROSToCISST(const diagnostic_msgs::msg::KeyValue & rosData, prmKeyValue & cisstData);

// cisst_msgs
void mtsROSToCISST(const cisst_msgs::msg::DoubleVec & rosData, prmPositionJointSet & cisstData);
void mtsROSToCISST(const cisst_msgs::msg::DoubleVec & rosData, vctDoubleVec & cisstData);
void mtsROSToCISST(const cisst_msgs::msg::IntervalStatistics & rosData, mtsIntervalStatistics & cisstData);
void mtsROSToCISST(const cisst_msgs::srv::ConvertFloat64Array::Request & rosData, vctDoubleVec & cisstData);

#endif // _mtsROSToCISST_h
