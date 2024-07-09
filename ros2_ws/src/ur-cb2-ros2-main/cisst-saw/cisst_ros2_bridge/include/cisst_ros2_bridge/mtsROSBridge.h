/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Adnan Munawar
  Created on: 2013-05-21

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#ifndef _mtsROSBridge_h
#define _mtsROSBridge_h

// cisst include
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

// ros2 include
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// conversion methods
#include <cisst_ros2_bridge/mtsCISSTToROS.h>
#include <cisst_ros2_bridge/mtsROSToCISST.h>

// ----------------------------------------------------
// Publisher
// ----------------------------------------------------

class mtsROSPublisherBase
{
private:
    //! Disable default constructor
    mtsROSPublisherBase(void);

public:
    //! Function used to pull data from the cisst component
    mtsFunctionRead Function;

    mtsROSPublisherBase(const std::string & name,
                        std::shared_ptr<rclcpp::Node> node,
                        const bool latch):
        mName(name),
        mNode(node),
        mLatched(latch)
    {}

    virtual ~mtsROSPublisherBase() {};

    virtual bool Execute(void) = 0;

protected:
    std::string mName;
    std::shared_ptr<rclcpp::Node> mNode;
    bool mLatched;
};

template <typename _mtsType, typename _rosType>
class mtsROSPublisher: public mtsROSPublisherBase
{
public:
    mtsROSPublisher(const std::string & name,
                    std::shared_ptr<rclcpp::Node> node,
                    const uint32_t queueSize = 5,
                    const bool latch = false):
        mtsROSPublisherBase(name, node, latch)
    {
        rclcpp::QoS qos(queueSize);
        if (mLatched) {
            qos.transient_local();
        }
        mPublisher =
            node->create_publisher<_rosType>(name, qos);
    }
    virtual ~mtsROSPublisher() {
        // xxx mPublisher.shutdown();
    }

    bool Execute(void) {
        if ((mPublisher->get_subscription_count() == 0) && !mLatched ) {
            return true;
        }
        mtsExecutionResult result = Function(mCISSTData);
        if (result) {
            if (mts_cisst_to_ros::header(mCISSTData, mROSData, mNode, mName)) {
                mtsCISSTToROS(mCISSTData, mROSData, mName);
                mPublisher->publish(mROSData);
                return true;
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "mtsROSPublisher::Execute: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROSPublisher::Execute: " << result
                              << " for topic " << mPublisher->get_topic_name() << std::endl;
        }
        return false;
    }

protected:
    typedef rclcpp::Publisher<_rosType> PublisherType;
    typename PublisherType::SharedPtr mPublisher;
    _mtsType mCISSTData;
    _rosType mROSData;
};

class mtsROSEventVoidPublisher: public mtsROSPublisherBase
{
public:
    mtsROSEventVoidPublisher(const std::string & name,
                             std::shared_ptr<rclcpp::Node> node,
                             const uint32_t queueSize = 100,
                             const bool latch = true):
        mtsROSPublisherBase(name, node, latch)
    {
        rclcpp::QoS qos(queueSize);
        if (mLatched) {
            qos.transient_local();
        }
        mPublisher =
            node->create_publisher<std_msgs::msg::Empty>(name, qos);
    }
    virtual ~mtsROSEventVoidPublisher() {
        /// xxx mPublisher.shutdown();
    }
    bool Execute(void) {
        return true;
    }

    void EventHandler(void) {
        if ((mPublisher->get_subscription_count() == 0) && !mLatched ) {
            return;
        }
        mPublisher->publish(mEmptyMsg);
    }
private:
    typedef typename rclcpp::Publisher<std_msgs::msg::Empty> PublisherType;
    typename PublisherType::SharedPtr mPublisher;
    std_msgs::msg::Empty mEmptyMsg;
};


template <typename _mtsType, typename _rosType>
class mtsROSEventWritePublisher: public mtsROSPublisherBase
{
public:
    mtsROSEventWritePublisher(const std::string & name,
                              std::shared_ptr<rclcpp::Node> node,
                              const uint32_t queueSize = 100,
                              const bool latch = true):
        mtsROSPublisherBase(name, node, latch)
    {
        rclcpp::QoS qos(queueSize);
        if (mLatched) {
            qos.transient_local();
        }
        mPublisher =
            node->create_publisher<_rosType>(name, qos);
    }
    virtual ~mtsROSEventWritePublisher() {
        // xxx mPublisher.shutdown();
    }

    bool Execute(void) {
        return true;
    }

    void EventHandler(const _mtsType & CISSTData) {
        if ((mPublisher->get_subscription_count() == 0) && !mLatched ) {
            return;
        }
        if (mts_cisst_to_ros::header(CISSTData, mROSData, mNode, mName)) {
            mtsCISSTToROS(CISSTData, mROSData, mName);
            mPublisher->publish(mROSData);
        }
    }

protected:
    typedef typename rclcpp::Publisher<_rosType> PublisherType;
    typename PublisherType::SharedPtr mPublisher;
    _rosType mROSData;
};


class mtsROStf2Broadcaster: public mtsROSPublisherBase
{
public:
    mtsROStf2Broadcaster(const std::string name,
                         std::shared_ptr<rclcpp::Node> node):
        mtsROSPublisherBase(name, node, false /* not latched */),
        mBroadcaster(node)
    {}
    virtual ~mtsROStf2Broadcaster() {
    }

    bool Execute(void) {
        mtsExecutionResult result = Function(mCISSTData);
        if (result) {
            // first check if it's new
            if (mCISSTData.Timestamp() > mLastTimestamp) {
                mLastTimestamp = mCISSTData.Timestamp();
                // then convert and check if the data is valid
                if (mts_cisst_to_ros::header(mCISSTData, mROSData, mNode, mName)) {
                    mtsCISSTToROS(mCISSTData, mROSData, mName);
                    mBroadcaster.sendTransform(mROSData);
                    return true;
                }
            }
        } else if (result.Value() != mtsExecutionResult::FUNCTION_NOT_BOUND) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "mtsROStf2Broadcaster::Execute: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROStf2Broadcaster::Execute: " << result
                              << " for " << mName << std::endl;
            return false;
        }
        return false;
    }

protected:
    tf2_ros::TransformBroadcaster mBroadcaster;
    geometry_msgs::msg::TransformStamped mROSData;
    prmPositionCartesianGet mCISSTData;
    double mLastTimestamp = 0.0;
};


class mtsROSEventWriteLog: public mtsROSPublisherBase
{
public:
    enum LogLevel {ROS_LOG_DEBUG, ROS_LOG_INFO, ROS_LOG_WARN, ROS_LOG_ERROR, ROS_LOG_FATAL};

    mtsROSEventWriteLog(const LogLevel level,
                        std::shared_ptr<rclcpp::Node> node):
        mtsROSPublisherBase("log", node, false /* not latched */),
        mLevel(level)
    {}
    virtual ~mtsROSEventWriteLog() {}

    bool Execute(void) {
        return true;
    }

    void EventHandler(const mtsMessage & message) {
        switch (mLevel) {
        case ROS_LOG_DEBUG:
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                         "%fs, #%zu: %s", message.Timestamp, message.Counter, message.Message.c_str());
            break;
        case ROS_LOG_INFO:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "%fs, #%zu: %s", message.Timestamp, message.Counter, message.Message.c_str());
            break;
        case ROS_LOG_WARN:
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                        "%fs, #%zu: %s", message.Timestamp, message.Counter, message.Message.c_str());
            break;
        case ROS_LOG_ERROR:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "%fs, #%zu: %s", message.Timestamp, message.Counter, message.Message.c_str());
            break;
        case ROS_LOG_FATAL:
            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),
                         "%fs, #%zu: %s", message.Timestamp, message.Counter, message.Message.c_str());
            break;
        default:
            break;
        }
    }

protected:
    const LogLevel mLevel;
};


// ----------------------------------------------------
// Subscriber
// ----------------------------------------------------

template <typename _mtsType, typename _rosType>
class mtsROSSubscriberWrite
{
public:
    typedef mtsROSSubscriberWrite<_mtsType, _rosType> ThisType;
    mtsROSSubscriberWrite(const std::string & name,
                          std::shared_ptr<rclcpp::Node> node):
        mNode(node)
    {
        mSubscriber =
            node->create_subscription<_rosType>(name,
                                                1, std::bind(&ThisType::Callback,
                                                             this,
                                                             std::placeholders::_1));
    }
    virtual ~mtsROSSubscriberWrite() {
        // xxx mSubscriber.shutdown();
    }

    void Callback(const std::shared_ptr<_rosType> rosData) {
        mts_ros_to_cisst::header(rosData, mCISSTData, mNode);
        mtsROSToCISST(*rosData, mCISSTData);
        mtsExecutionResult result = Function(mCISSTData);
        if (!result) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "mtsROSSubscriberWrite:Callback: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROSSubscriberWrite:Callback: " << result
                              << " for topic " << mSubscriber->get_topic_name() << std::endl;
        }
    }

    mtsFunctionWrite Function;

protected:
    typedef rclcpp::Subscription<_rosType> SubscriberType;
    typename SubscriberType::SharedPtr mSubscriber;
    _mtsType mCISSTData;
    std::shared_ptr<rclcpp::Node> mNode;
};


class mtsROSSubscriberVoid
{
public:
    typedef mtsROSSubscriberVoid ThisType;
    mtsROSSubscriberVoid(const std::string & name,
                         std::shared_ptr<rclcpp::Node> node) {
        mSubscriber =
            node->create_subscription<std_msgs::msg::Empty>(name,
                                                            1, std::bind(&ThisType::Callback,
                                                                         this,
                                                                         std::placeholders::_1));
    }
    virtual ~mtsROSSubscriberVoid() {
        /// xxx mSubscriber.shutdown();
    }

    void Callback(const std::shared_ptr<std_msgs::msg::Empty> CMN_UNUSED(rosData)) {
        mtsExecutionResult result = Function();
        if (!result) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "mtsROSSubscriberVoid:Callback: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROSSubscriberVoid::Callback: " << result
                              << " for topic " << mSubscriber->get_topic_name() << std::endl;
        }
    }

    mtsFunctionVoid Function;

protected:
    typedef rclcpp::Subscription<std_msgs::msg::Empty> SubscriberType;
    typename SubscriberType::SharedPtr mSubscriber;
};


template <typename _mtsType, typename _rosType>
class mtsROSSubscriberStateTable
{
public:
    typedef mtsROSSubscriberStateTable<_mtsType, _rosType> ThisType;
    mtsROSSubscriberStateTable(const std::string & name,
                               std::shared_ptr<rclcpp::Node> node,
                               const size_t & tableSize):
        mStateTable(tableSize, name),
        mNode(node)
    {
        mStateTable.AddData(mCISSTData, name);
        mSubscriber =
            node->create_subscription<_rosType>(name,
                                                1, std::bind(&ThisType::Callback,
                                                             this,
                                                             std::placeholders::_1));
    }
    virtual ~mtsROSSubscriberStateTable() {
        // xxx mSubscriber.shutdown();
    }

    void Callback(const std::shared_ptr<_rosType> rosData) {
        mStateTable.Start();
        mts_ros_to_cisst::header(*rosData, mCISSTData, mNode);
        mtsROSToCISST(*rosData, mCISSTData);
        mStateTable.Advance();
    }

    mtsStateTable mStateTable;
    _mtsType mCISSTData;

protected:
    typedef rclcpp::Subscription<_rosType> SubscriberType;
    typename SubscriberType::SharedPtr mSubscriber;
    std::shared_ptr<rclcpp::Node> mNode;
};


template <typename _mtsType, typename _rosType>
class mtsROSCommandWritePublisher
{
public:
    mtsROSCommandWritePublisher(const std::string & name,
                                std::shared_ptr<rclcpp::Node> node,
                                const uint32_t queueSize = 100,
                                const bool latch = false):
        mName(name),
        mNode(node),
        mLatched(latch)
    {
        rclcpp::QoS qos(queueSize);
        if (mLatched) {
            qos.transient_local();
        }
        mPublisher =
            node->create_publisher<_rosType>(name, qos);
    }
    virtual ~mtsROSCommandWritePublisher() {
        // xxx mPublisher.shutdown();
    }

    void Command(const _mtsType & CISSTData) {
        if ((mPublisher->get_subscription_count() == 0) && !mLatched ) {
            return;
        }
        if (mts_cisst_to_ros::header(CISSTData, mROSData, mNode, mName)) {
            mtsCISSTToROS(CISSTData, mROSData, mName);
            mPublisher->publish(mROSData);
        }
    }

protected:
    typedef rclcpp::Publisher<_rosType> PublisherType;
    typename PublisherType::SharedPtr mPublisher;
    std::string mName;
    std::shared_ptr<rclcpp::Node> mNode;
    bool mLatched;
    _rosType mROSData;
};


class mtsROSCommandVoidPublisher
{
public:
    mtsROSCommandVoidPublisher(const std::string & name,
                               std::shared_ptr<rclcpp::Node> node,
                               const uint32_t queueSize = 100,
                               const bool latch = false):
        mLatched(latch)
    {
        rclcpp::QoS qos(queueSize);
        if (mLatched) {
            qos.transient_local();
        }
        mPublisher =
            node->create_publisher<std_msgs::msg::Empty>(name, qos);
    }
    virtual ~mtsROSCommandVoidPublisher() {
        // xxx mPublisher.shutdown();
    }

    void Command(void)
    {
        if ((mPublisher->get_subscription_count() == 0) && !mLatched ) {
            return;
        }
        mPublisher->publish(mEmptyMsg);
    }

protected:
    typedef rclcpp::Publisher<std_msgs::msg::Empty> PublisherType;
    bool mLatched;
    typename PublisherType::SharedPtr mPublisher;
    std_msgs::msg::Empty mEmptyMsg;
};


// ----------------------------------------------------
// Services
// ----------------------------------------------------
template <typename _mtsResponseType,
          typename _rosQueryType>
class mtsROSCommandReadService
{
public:
    typedef mtsROSCommandReadService<_mtsResponseType,
                                     _rosQueryType> ThisType;

    mtsFunctionRead Function;

    mtsROSCommandReadService(const std::string name,
                             std::shared_ptr<rclcpp::Node> node):
        mName(name),
        mNode(node)
    {
        mServiceServer =
            node->create_service<_rosQueryType>(name,
                                                std::bind(&ThisType::Callback,
                                                          this,
                                                          std::placeholders::_1,
                                                          std::placeholders::_2));
    }

    bool Callback(const std::shared_ptr<typename _rosQueryType::Request> CMN_UNUSED(request),
                  std::shared_ptr<typename _rosQueryType::Response> response) {
        mtsExecutionResult result = Function(mResponse);
        if (result) {
            if (mts_cisst_to_ros::header(mResponse, *response, mNode, mName)) {
                mtsCISSTToROS(mResponse, *response, mName);
                return true;
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "mtsROSCommandReadService::Callback: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROSCommandReadService::Callback: " << result
                              << " for service " << mName << std::endl;
        }
        return false;
    }

protected:
    _mtsResponseType mResponse;
    typedef rclcpp::Service<_rosQueryType> ServerServiceType;
    typename ServerServiceType::SharedPtr mServiceServer;
    std::string mName;
    std::shared_ptr<rclcpp::Node> mNode;
};


template <typename _mtsRequestType,
          typename _mtsResponseType,
          typename _rosQueryType>
class mtsROSCommandQualifiedReadService
{
public:
    typedef mtsROSCommandQualifiedReadService<_mtsRequestType,
                                              _mtsResponseType,
                                              _rosQueryType> ThisType;

    mtsFunctionQualifiedRead Function;

    mtsROSCommandQualifiedReadService(const std::string name,
                                      std::shared_ptr<rclcpp::Node> node):
        mName(name),
        mNode(node)
    {
        mServiceServer =
            node->create_service<_rosQueryType>(name,
                                                std::bind(&ThisType::Callback,
                                                          this,
                                                          std::placeholders::_1,
                                                          std::placeholders::_2));
    }

    bool Callback(const std::shared_ptr<typename _rosQueryType::Request> request,
                  std::shared_ptr<typename _rosQueryType::Response> response) {
        mts_ros_to_cisst::header(*request, mRequest, mNode);
        mtsROSToCISST(*request, mRequest);
        mtsExecutionResult result = Function(mRequest, mResponse);
        if (result) {
            if (mts_cisst_to_ros::header(mResponse, *response, mNode, mName)) {
                mtsCISSTToROS(mResponse, *response, mName);
                return true;
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "mtsROSCommandQualifiedReadService::Callback: mtsFunction call failed");
            CMN_LOG_RUN_ERROR << "mtsROSCommandReadService::Callback: " << result
                              << " for service " << mName << std::endl;
        }
        return false;
    }

protected:
    _mtsRequestType mRequest;
    _mtsResponseType mResponse;
    typedef rclcpp::Service<_rosQueryType> ServerServiceType;
    typename ServerServiceType::SharedPtr mServiceServer;
    std::string mName;
    std::shared_ptr<rclcpp::Node> mNode;
};


// ----------------------------------------------------
// Bridge
// ----------------------------------------------------

/*! Base component to convert cisst/SAW commands and event to/from ROS
  topics.  This component starts without any cisstMultiTask
  interface nor commands nor functions.  The user can configure this
  component by adding one of the following:

  - AddPublisherFromCommandRead
  [required interface][function read] -> [topic publish] (periodic)

  - AddPublisherFromEventVoid
  [required interface][event void] -> [topic publish]

  - AddPublisherFromEventWrite
  [required interface][event write] -> [topic publish]

  - Addtf2BroadcasterFromCommandRead
  [required interface][function read] -> [broadcast tf] (periodic)
  The function read must get prmPositionCartesianGet

  - AddSubscriberToCommandVoid
  [required interface][function void] <- [topic subscribe]

  - AddSubscriberToCommandWrite
  [required interface][function write] <- [topic subscribe]

  - AddPublisherFromCommandWrite
  [provided interface][command write] -> [topic publish]

  - AddPublisherFromCommandVoid
  [provided interface][command void] -> [topic publish]

  - AddSubscriberToCommandRead
  [provided interface][command read] <- [topic subscribe]

  - AddSubscriberToEventVoid
  [providedInterface][event void] <- [topic subscribe]

  - AddSubscriberToEventWrite
  [providedInterface][event write] <- [topic subscribe]

  For each of these methods (except the ones with Void commands or
  events), the user needs to provided the cisst/SAW and ROS types
  used to convert the data back and forth.  Since the conversion
  method has to be defined at compilation time, this is done using
  template specialization:

  mtsROSBridge bridge("publisher", 5.0 * cmn_ms);
  bridge.AddPublisherFromCommandRead
  <vctDoubleVec, cisst_msgs::vctDoubleVec>
  ("required",
  "GetValue1",
  "/sawROSExample/get_value_1");

  The first template parameter is the cisst type used for the
  command, the second parameter is the ROS type used to publish.  At
  compilation time, the compiler will look for one of the following
  overloaded method:
  - bool mtsCISSTToROS(const _cisstType & in, _rosType out, const std::string & debugInfo)
  - bool mtsROSToCISST(const _rosType & in, _cisstType out, const std::string & debugInfo)

  Some default conversion methods are provided in mtsROSToCISST.h
  and mtsCISSTToROS.h.

*/
class mtsROSBridge: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    /*!
      \brief Constructor

      \param componentName component name
      \param periodInSeconds thread period
      \param spin call spinOnce() in run() is set to true
      \param sig true to install default signal handler, if
      set to false, either install your own handler or
      rely on cisst cleanup()
    */
    CISST_DEPRECATED mtsROSBridge(const std::string & componentName,
                                  const double periodInSeconds,
                                  const bool spin = false,
                                  const bool sig = true,
                                  std::shared_ptr<rclcpp::Node> node = nullptr);

    mtsROSBridge(const mtsTaskPeriodicConstructorArg & arg);

    /*!  Constructor using an existing rclcpp::Node.  By default,
      spin is set to false (see also PerformsSpin) and this
      constructor doesn't redefine the signal handler.
    */
    mtsROSBridge(const std::string & componentName,
                 const double periodInSeconds,
                 std::shared_ptr<rclcpp::Node> node);

    ~mtsROSBridge();

    // taskPeriodic
    void Configure(const std::string & CMN_UNUSED(filename) = "");
    bool AddIntervalStatisticsInterface(const std::string & interfaceName = "IntervalStatistics");
    void AddIntervalStatisticsPublisher(const std::string & rosNamespace,
                                        const std::string & componentName,
                                        const std::string & interfaceName = "IntervalStatistics");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    /*! Request that the bridge calls ros::spinOnce in its Run method.
      This allows to piggy back on the existing thread/periodicity
      instead of using ros::spin in your main.  This can be used if
      you also have another library or toolkit that has its own event
      loop (i.e. Qt with QApplication.exec()).  If you use multiple
      mtsROSBridge, make sure there's only one bridge with spin turned
      on. */
    inline void PerformsSpin(const bool spin) {
        mSpin = spin;
    }

    // --------- Required interface

    // --------- Publisher ------------------

    /*! Add a read function to a cisstMultiTask required interface.
      When connected to an existing provided interface, this allows
      to read some data from an existing cisstMultiTask component
      and publish it to ROS.  This action is performed periodically,
      it is triggered by the periodicity defined when the bridge
      (this class) is constructed.

      \param interfaceRequiredName Name of the required interface to be created
      \param functionName Name of the read function added to the interface
      \param topicName Name of the topic used to publish
    */
    template <typename _mtsType, typename _rosType>
    bool AddPublisherFromCommandRead(const std::string & interfaceRequiredName,
                                     const std::string & functionName,
                                     const std::string & topicName,
                                     const uint32_t queueSize = 100,
                                     const bool latch = false);

    /*! Add an event handler (void) to a cisstMultiTask required
      interface.  When connected to an existing provided interface,
      this allows to handle events from an existing cisstMultiTask
      component and publish it to ROS using std::msgs::Empty.

      \param interfaceRequiredName Name of the required interface to be created
      \param eventName Name of the event to handle
      \param topicName Name of the topic used to publish
    */
    bool AddPublisherFromEventVoid(const std::string & interfaceRequiredName,
                                   const std::string & eventName,
                                   const std::string & topicName,
                                   const uint32_t queueSize = 100,
                                   const bool latch = true);

    /*! Add an event handler (write) to a cisstMultiTask required
      interface.  When connected to an existing provided interface,
      this allows to handle events from an existing cisstMultiTask
      component and publish it to ROS after converting from _mtsType
      to _rosType.

      \param interfaceRequiredName Name of the required interface to be created
      \param eventName Name of the event to handle
      \param topicName Name of the topic used to publish
    */
    template <typename _mtsType, typename _rosType>
    bool AddPublisherFromEventWrite(const std::string & interfaceRequiredName,
                                    const std::string & eventName,
                                    const std::string & topicName,
                                    const uint32_t queueSize = 100,
                                    const bool latch = true);

    // --------- Subscriber ------------------

    /*! Add a write function to a cisstMultiTask required interface.
      When connected to an existing provided interface, this allows
      to send commands to an existing cisstMultiTask component from
      a ROS subscriber after converting from _rosType to _mtsType.

      \param interfaceRequiredName Name of the required interface to be created
      \param functionName Name of the write function
      \param topicName Name of the topic this subscribes to
    */
    template <typename _mtsType, typename _rosType>
    bool AddSubscriberToCommandWrite(const std::string & interfaceRequiredName,
                                     const std::string & functionName,
                                     const std::string & topicName);

    /*! Add a void function to a cisstMultiTask required interface.
      When connected to an existing provided interface, this allows
      to send commands to an existing cisstMultiTask component from
      a ROS subscriber (receiving std_msgs::Empty).

      \param interfaceRequiredName Name of the required interface to be created
      \param functionName Name of the void function
      \param topicName Name of the topic this subscribes to
    */
    bool AddSubscriberToCommandVoid(const std::string & interfaceRequiredName,
                                    const std::string & functionName,
                                    const std::string & topicName);

    // -------- tf2 broadcasters
    bool Addtf2BroadcasterFromCommandRead(const std::string & interfaceRequiredName,
                                          const std::string & functionName);

    // --------- Events to ROS log

    /*! Add an event handler (write) to a cisstMultiTask required
      interface.  When connected to an existing provided interface,
      this allows to handle events with a std::string payload from
      an existing cisstMultiTask component and log the message using
      either ROS_DEBUG, ROS_INFO, ROS_WARN, ROS_ERROR or ROS_FATAL
      based on the level selected (see
      mtsROSEventWriteLog::LogLevel).

      \param interfaceRequiredName Name of the required interface to be created
      \param eventName Name of the event to handle
      \param level Level used to log in ROS
    */
    bool AddLogFromEventWrite(const std::string & interfaceRequiredName,
                              const std::string & eventName,
                              const mtsROSEventWriteLog::LogLevel & level);

    // --------- Provided interface

    // --------- Publisher ------------------

    /*! Add a command (write) to a cisstMultiTask provided interface.
      When connected to an existing required interface, this allows
      to execute write functions from an existing cisstMultiTask
      component and publish it to ROS after converting from _mtsType
      to _rosType.

      \param interfaceProvidedName Name of the provided interface to be created
      \param commandName Name of the write command added to the interface
      \param topicName Name of the topic used to publish
    */
    template <typename _mtsType, typename _rosType>
    bool AddPublisherFromCommandWrite(const std::string & interfaceProvidedName,
                                      const std::string & commandName,
                                      const std::string & topicName,
                                      const uint32_t queueSize = 100,
                                      const bool latch = false);

    /*! Add a command (void) to a cisstMultiTask provided interface.
      When connected to an existing required interface, this allows
      to execute void functions from an existing cisstMultiTask
      component and publish it to ROS (using std_msgs::Empty).

      \param interfaceProvidedName Name of the provided interface to be created
      \param commandName Name of the void command added to the interface
      \param topicName Name of the topic used to publish
    */
    bool AddPublisherFromCommandVoid(const std::string & interfaceProvidedName,
                                     const std::string & commandName,
                                     const std::string & topicName,
                                     const uint32_t queueSize = 100,
                                     const bool latch = false);

    // --------- Subscriber ------------------

    /*! Add a command read to a cisstMultiTask provided interface.
      When connected to an existing required interface, this allows
      to execute read functions from an existing cisstMultiTask
      component.  When the subscriber receives some data, it adds it
      to a local state table after converting from _rosType to
      _mtsType.  The data will be cached for the next call to the
      read command (see mtsStateTable).

      \param interfaceProvidedName Name of the provided interface to be created
      \param commandName Name of the void command added to the interface
      \param topicName Name of the topic this subscribes to
      \param tableSize Size of the state table used to cache the data
    */
    template <typename _mtsType, typename _rosType>
    bool AddSubscriberToCommandRead(const std::string & interfaceProvidedName,
                                    const std::string & commandName,
                                    const std::string & topicName,
                                    const size_t & tableSize = 500);

    /*! Add a void event to a cisstMultiTask provided interface.  When
      connected to an existing required interface, this allows to
      send void events to an existing cisstMultiTask component.
      When the subscriber receives some data (std_msgs::Empty), it
      triggers the event.

      \param interfaceProvidedName Name of the provided interface to be created
      \param eventName Name of the write event added to the interface
      \param topicName Name of the topic this subscribes to
    */
    bool AddSubscriberToEventVoid(const std::string & interfaceProvidedName,
                                  const std::string & eventName,
                                  const std::string & topicName);

    /*! Add a write event to a cisstMultiTask provided interface.
      When connected to an existing required interface, this allows
      to send write events to an existing cisstMultiTask component.
      When the subscriber receives some data, it triggers the event
      after converting the data from _rosType to _mtsType.

      \param interfaceProvidedName Name of the provided interface to be created
      \param eventName Name of the write event added to the interface
      \param topicName Name of the topic this subscribes to
    */
    template <typename _mtsType, typename _rosType>
    bool AddSubscriberToEventWrite(const std::string & interfaceProvidedName,
                                   const std::string & eventName,
                                   const std::string & topicName);

    template <typename _mtsResponseType, typename _rosQueryType>
    bool AddServiceFromCommandRead(const std::string & interfaceRequiredName,
                                   const std::string & functionName,
                                   const std::string & serviceName);
    template <typename _mtsRequestType, typename _mtsResponseType, typename _rosQueryType>
    bool AddServiceFromCommandQualifiedRead(const std::string & interfaceRequiredName,
                                            const std::string & functionName,
                                            const std::string & serviceName);

protected:
    //! list of publishers
    typedef std::list<mtsROSPublisherBase *> PublishersType;
    PublishersType Publishers;

    //! ros node
    std::shared_ptr<rclcpp::Node> mNodePointer;

    //! spin flag, if set call spinOnce() in run
    bool mSpin;

    //! signal flag, if set use default signal handler from ros nodehandle
    bool mSignal;
};

template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddPublisherFromCommandRead(const std::string & interfaceRequiredName,
                                               const std::string & functionName,
                                               const std::string & topicName,
                                               const uint32_t queueSize,
                                               const bool latch)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired
        = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }
    if (!interfaceRequired) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "mtsROSBridge::AddPublisherFromCommandRead: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromCommandRead: failed to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }
    mtsROSPublisherBase * newPublisher =
        new mtsROSPublisher<_mtsType, _rosType>(topicName, mNodePointer, queueSize, latch);
    if (!interfaceRequired->AddFunction(functionName, newPublisher->Function)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "mtsROSBridge::AddPublisherFromCommandRead: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromCommandRead: failed to add function \""
                                 << functionName << "\" to interface required \""
                                 << interfaceRequiredName << "\"" << std::endl;
        delete newPublisher;
        return false;
    }
    Publishers.push_back(newPublisher);
    return true;
}


template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddSubscriberToCommandWrite(const std::string & interfaceRequiredName,
                                               const std::string & functionName,
                                               const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }
    if (!interfaceRequired) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "mtsROSBridge::AddSubscriberToCommandWrite: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToCommandWrite: failed to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }
    mtsROSSubscriberWrite<_mtsType, _rosType> * newSubscriber
        = new mtsROSSubscriberWrite<_mtsType, _rosType>(topicName, mNodePointer);
    if (!interfaceRequired->AddFunction(functionName, newSubscriber->Function)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "mtsROSBridge::AddSubscriberToCommandWrite: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToCommandWrite: failed to add function \""
                                 << functionName << "\" to interface required \""
                                 << interfaceRequiredName << "\"" << std::endl;
        delete newSubscriber;
        return false;
    }
    return true;
}


template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddPublisherFromEventWrite(const std::string & interfaceRequiredName,
                                              const std::string & eventName,
                                              const std::string & topicName,
                                              const uint32_t queueSize,
                                              const bool latch)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }

    mtsROSEventWritePublisher<_mtsType, _rosType> * newPublisher
        = new mtsROSEventWritePublisher<_mtsType, _rosType>(topicName, mNodePointer, queueSize, latch);
    if (!interfaceRequired->AddEventHandlerWrite(&mtsROSEventWritePublisher<_mtsType, _rosType>::EventHandler,
                                                 newPublisher, eventName)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "mtsROSBridge::AddPublisherFromEventWrite: failed to add event handler to required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromEventWrite: failed to add event handler for \""
                                 << eventName << "\" to required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        delete newPublisher;
        return false;
    }
    Publishers.push_back(newPublisher);
    return true;
}


template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddPublisherFromCommandWrite(const std::string & interfaceProvidedName,
                                                const std::string & commandName,
                                                const std::string & topicName,
                                                const uint32_t queueSize,
                                                const bool latch)
{
    // check if the interface exists of try to create one
    mtsInterfaceProvided * interfaceProvided = this->GetInterfaceProvided(interfaceProvidedName);
    if (!interfaceProvided) {
        interfaceProvided = this->AddInterfaceProvided(interfaceProvidedName);
    }

    mtsROSCommandWritePublisher<_mtsType, _rosType> * newPublisher
        = new mtsROSCommandWritePublisher<_mtsType, _rosType>(topicName, mNodePointer, queueSize, latch);
    if (!interfaceProvided->AddCommandWrite(&mtsROSCommandWritePublisher<_mtsType, _rosType>::Command,
                                            newPublisher, commandName))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "mtsROSBridge::AddPublisherFromCommandWrite: failed to create provided interface.");
            CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromCommandWrite: failed to create provided interface \""
                                     << interfaceProvidedName << "\"" << std::endl;
            delete newPublisher;
            return false;
        }
    return true;
}


template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddSubscriberToCommandRead(const std::string & interfaceProvidedName,
                                              const std::string & commandName,
                                              const std::string & topicName,
                                              const size_t & tableSize)
{
    // check if the interface exists of try to create one
    mtsInterfaceProvided * interfaceProvided = this->GetInterfaceProvided(interfaceProvidedName);
    if (!interfaceProvided) {
        interfaceProvided = this->AddInterfaceProvided(interfaceProvidedName);
    }

    mtsROSSubscriberStateTable<_mtsType, _rosType> * newSubscriber
        = new mtsROSSubscriberStateTable<_mtsType, _rosType>(topicName, mNodePointer, tableSize);
    if (!interfaceProvided->AddCommandReadState(newSubscriber->mStateTable,
                                                newSubscriber->mCISSTData,
                                                commandName)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "mtsROSBridge::AddSubscriberToCommandRead: failed to add command read to provided interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToCommandRead: failed to add command read \""
                                 << commandName << "\" to provided interface \""
                                 << interfaceProvidedName << "\"" << std::endl;
        delete newSubscriber;
        return false;
    }
    return true;
}


template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddSubscriberToEventWrite(const std::string & interfaceProvidedName,
                                             const std::string & eventName,
                                             const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceProvided * interfaceProvided = this->GetInterfaceProvided(interfaceProvidedName);
    if (!interfaceProvided) {
        interfaceProvided = this->AddInterfaceProvided(interfaceProvidedName);
    }

    mtsROSSubscriberWrite<_mtsType, _rosType> * newSubscriber
        = new mtsROSSubscriberWrite<_mtsType, _rosType>(topicName, mNodePointer);
    if (!interfaceProvided->AddEventWrite(newSubscriber->Function,
                                          eventName, _mtsType())) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "mtsROSBridge::AddSubscriberToEventWrite: failed to add event to provided interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToEventWrite: failed to add event \""
                                 << eventName << "\" to provided interface \""
                                 << interfaceProvidedName << "\"" << std::endl;
        delete newSubscriber;
        return false;
    }
    return true;
}




template <typename _mtsResponseType, typename _rosQueryType>
bool mtsROSBridge::AddServiceFromCommandRead(const std::string & interfaceRequiredName,
                                             const std::string & functionName,
                                             const std::string & serviceName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }
    if (!interfaceRequired) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "mtsROSBridge::AddServiceFromCommandRead: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddServiceFromCommandRead: failed to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }

    typedef mtsROSCommandReadService<_mtsResponseType, _rosQueryType> serviceType;
    serviceType * newService
        = new serviceType(serviceName, mNodePointer);

    if (!interfaceRequired->AddFunction(functionName, newService->Function)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "mtsROSBridge::AddServiceFromCommandRead: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddServiceFromCommandRead: failed to create function \""
                                 << functionName << "\"" << std::endl;
        delete newService;
        return false;
    }
    return true;
}

template <typename _mtsRequestType, typename _mtsResponseType, typename _rosQueryType>
bool mtsROSBridge::AddServiceFromCommandQualifiedRead(const std::string & interfaceRequiredName,
                                                      const std::string & functionName,
                                                      const std::string & serviceName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }
    if (!interfaceRequired) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "mtsROSBridge::AddServiceFromCommandQualifiedRead: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddServiceFromCommandQualifiedRead: failed to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }

    typedef mtsROSCommandQualifiedReadService<_mtsRequestType, _mtsResponseType, _rosQueryType> serviceType;
    serviceType * newService
        = new serviceType(serviceName, mNodePointer);

    if (!interfaceRequired->AddFunction(functionName, newService->Function)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "mtsROSBridge::AddServiceFromCommandQualifiedRead: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddServiceFromCommandQualifiedRead: failed to create function \""
                                 << functionName << "\"" << std::endl;
        delete newService;
        return false;
    }
    return true;
}

CMN_DECLARE_SERVICES_INSTANTIATION(mtsROSBridge);

#endif // _mtsROSBridge_h
