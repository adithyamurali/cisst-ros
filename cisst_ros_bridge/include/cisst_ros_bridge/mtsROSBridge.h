/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsROSBridge.h 4367 2013-07-17 02:47:21Z zchen24 $

  Author(s):  Anton Deguet, Zihan Chen, Adnan Munawar
  Created on: 2013-05-21

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights
  Reserved.

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

// ros include
#include <ros/ros.h>
#include <std_msgs/Empty.h>

// conversion methods
#include "cisst_ros_bridge/mtsCISSTToROS.h"
#include "cisst_ros_bridge/mtsROSToCISST.h"


// ----------------------------------------------------
// Publisher
// ----------------------------------------------------

class mtsROSPublisherBase
{
public:
    //! Function used to pull data from the cisst component
    mtsFunctionRead Function;
    //! ROS publisher to publish the converted data
    ros::Publisher Publisher;

    virtual bool Execute(void) = 0;
};

template <typename _mtsType, typename _rosType>
class mtsROSPublisher: public mtsROSPublisherBase
{
public:
    mtsROSPublisher(const std::string & rosTopicName, ros::NodeHandle & node) {
        Publisher = node.advertise<_rosType>(rosTopicName, 5);
    }
    ~mtsROSPublisher() {
        //! \todo, how to remove the topic from the node?
    }

    bool Execute(void) {
        mtsExecutionResult result = Function(CISSTData);

        if (result) {
            mtsCISSTToROS(CISSTData, ROSData);
            Publisher.publish(ROSData);
            return true;
        }
    }

protected:
    _mtsType CISSTData;
    _rosType ROSData;
};

class mtsROSEventVoidPublisher: public mtsROSPublisherBase
{
public:
    mtsROSEventVoidPublisher(const std::string & rosTopicName, ros::NodeHandle & node)
    {
        Publisher = node.advertise<std_msgs::Empty>(rosTopicName, 5);
    }
    ~mtsROSEventVoidPublisher(){
        //! \todo remove the topic from the node
    }
    bool Execute(void){return true;}

    void EventHandler(){
        Publisher.publish(mEmptyMsg);
    }
private:
    std_msgs::Empty mEmptyMsg;
};

template <typename _mtsType, typename _rosType>
class mtsROSEventWritePublisher: public mtsROSPublisherBase
{
public:
    mtsROSEventWritePublisher(const std::string & rosTopicName, ros::NodeHandle & node){
      Publisher = node.advertise<_rosType>(rosTopicName, 5);
    }
    ~mtsROSEventWritePublisher(){}

    bool Execute(void){return true;}

    void EventHandler(const _mtsType& CISSTData)
    {
        mtsCISSTToROS(CISSTData, ROSData);
        Publisher.publish(ROSData);
    }

protected:
    _rosType ROSData;
};

// ----------------------------------------------------
// Publisher Stamped
// ----------------------------------------------------


class mtsROSPublisherBaseStamped
{
public:
    //! Function used to pull data from the cisst component
    mtsFunctionRead Function;
    //! ROS publisher to publish the converted data
    ros::Publisher Publisher;

    virtual bool Execute(const ros::Time & startTime) = 0;
};


class mtsROSPublisherStamped: public mtsROSPublisherBaseStamped
{
public:
    mtsROSPublisherStamped(const std::string & rosTopicName, ros::NodeHandle & node) {
        Publisher = node.advertise<geometry_msgs::PoseStamped>(rosTopicName, 5);
    }
    ~mtsROSPublisherStamped() {
        //! \todo, how to remove the topic from the node?
    }

    bool Execute(const ros::Time & startTime) {
        mtsExecutionResult result = Function(CISSTData);

        if (result) {
            mtsCISSTToROS(CISSTData, ROSData);
            ROSData.header.stamp = startTime;
            Publisher.publish(ROSData);
            return true;
        }
    }

protected:
    prmPositionCartesianGet CISSTData;
    geometry_msgs::PoseStamped ROSData;
};

// ----------------------------------------------------
// Subscriber
// ----------------------------------------------------

class mtsROSSubscriberBase
{
public:
    //! Function used to pull data from the cisst component
    mtsFunctionWrite Function;        
    //! ROS publisher to publish the converted data
    ros::Subscriber Subscriber;
};

template <typename _mtsType, typename _rosType>
class mtsROSSubscriber: public mtsROSSubscriberBase
{
public:
    typedef mtsROSSubscriber<_mtsType, _rosType> ThisType;
    mtsROSSubscriber(const std::string & rosTopicName, ros::NodeHandle & node) {
        Subscriber = node.subscribe(rosTopicName, 1, &ThisType::Callback, this);
    }
    ~mtsROSSubscriber() {
        // \todo, how to remove the subscriber from the node?
    }

    void Callback(const _rosType & rosData) {
        mtsROSToCISST(rosData, CISSTData);
        mtsExecutionResult result = Function(CISSTData);
        if (!result) {
            std::cerr << result << std::endl;
        }
    }

protected:
    _mtsType CISSTData;
    _rosType ROSData;
};


class mtsROSSubscriberVoid: public mtsROSSubscriberBase
{
public:
  mtsROSSubscriberVoid(const std::string & rosTopicName, ros::NodeHandle & node){
    Subscriber = node.subscribe(rosTopicName, 1, &mtsROSSubscriberVoid::Callback, this);
  }
  ~mtsROSSubscriberVoid(){}

  void Callback(const std_msgs::Empty & rosData) {
    std::cout << "empty msg received" << std::endl;

    mtsExecutionResult result = FunctionVoid();
    if (!result) {
      std::cerr << result << std::endl;
    }
  }

  //! Function used to trigger void command
  mtsFunctionVoid FunctionVoid;
};


// ----------------------------------------------------
// Bridge
// ----------------------------------------------------

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
    mtsROSBridge(const std::string & componentName,
                 double periodInSeconds,
                 bool spin = false,
                 bool sig = true);
    inline ~mtsROSBridge() {}

    // taskPeriodic
    void Configure(const std::string & CMN_UNUSED(filename) = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    // --------- Publisher ------------------
    template <typename _mtsType, typename _rosType>
    bool AddPublisherFromReadCommand(const std::string & interfaceRequiredName,
                                     const std::string & functionName,
                                     const std::string & topicName);

    bool AddPublisherFromReadCommandStamped(const std::string & interfaceRequiredName,
                                     const std::string & functionName,
                                     const std::string & topicName);

    bool AddPublisherFromEventVoid(const std::string & interfaceRequiredName,
                                   const std::string & eventName,
                                   const std::string & topicName);

    template <typename _mtsType, typename _rosType>
    bool AddPublisherFromEventWrite(const std::string & interfaceRequiredName,
                                    const std::string & eventName,
                                    const std::string & topicName);


    // --------- Subscriber ------------------
    template <typename _mtsType, typename _rosType>
    bool AddSubscriberToWriteCommand(const std::string & interfaceRequiredName,
                                     const std::string & functionName,
                                     const std::string & topicName);

    bool AddSubscriberToVoidCommand(const std::string & interfaceRequiredNamPublishersTypee,
                                    const std::string & functionName,
                                    const std::string & topicName);

protected:
    //! list of publishers
    typedef std::list<mtsROSPublisherBase*> PublishersType;
    PublishersType Publishers;

    //! list of publishers
    typedef std::list<mtsROSPublisherBaseStamped*> PublishersTypeStamped;
    PublishersTypeStamped PublishersStamped;

    //! list of subscribers
    typedef std::list<mtsROSSubscriberBase*> SubscribersType;
    SubscribersType Subscribers;

    //! ros node
    ros::NodeHandle * Node;

    //! spin flag, if set call spinOnce() in run
    bool mSpin;

    //! signal flag, if set use default signal handler from ros nodehandle
    bool mSignal;
};

template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddPublisherFromReadCommand(const std::string & interfaceRequiredName,
                                               const std::string & functionName,
                                               const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }
    if (!interfaceRequired) {
        ROS_ERROR("mtsROS::AddPublisherFromReadCommand: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromReadCommand: faild to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }
    mtsROSPublisherBase * newPublisher = new mtsROSPublisher<_mtsType, _rosType>(topicName, *(this->Node));
    if (!interfaceRequired->AddFunction(functionName, newPublisher->Function)) {
        ROS_ERROR("mtsROS::AddPublisherFromReadCommand: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromReadCommand: faild to create function \""
                                 << functionName << "\"" << std::endl;
        delete newPublisher;
        return false;
    }
    Publishers.push_back(newPublisher);
    return true;
}


template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddSubscriberToWriteCommand(const std::string & interfaceRequiredName,
                                               const std::string & functionName,
                                               const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }
    if (!interfaceRequired) {
        ROS_ERROR("mtsROS::AddSubscribeToWriteCommand: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscribeToWriteCommand: faild to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }
    mtsROSSubscriberBase * newSubscriber = new mtsROSSubscriber<_mtsType, _rosType>(topicName, *(this->Node));
    if (!interfaceRequired->AddFunction(functionName, newSubscriber->Function)) {
        ROS_ERROR("mtsROS::AddSubscriberToWriteCommand: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToWriteCommand: faild to create function \""
                                 << functionName << "\"" << std::endl;
        delete newSubscriber;
        return false;
    }
    Subscribers.push_back(newSubscriber);
    return true;
}


template <typename _mtsType, typename _rosType>
bool mtsROSBridge::AddPublisherFromEventWrite(const std::string &interfaceRequiredName,
                                              const std::string &eventName,
                                              const std::string &topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }

    mtsROSEventWritePublisher<_mtsType, _rosType>* newPublisher = new mtsROSEventWritePublisher<_mtsType, _rosType>(topicName, *(this->Node));
    if (!interfaceRequired->AddEventHandlerWrite(&mtsROSEventWritePublisher<_mtsType, _rosType>::EventHandler, newPublisher, eventName))
    {
        ROS_ERROR("mtsROS::mtsROSEventWritePublisher: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "mtsROSEventWritePublisher: faild to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        delete newPublisher;
        return false;
    }
    Publishers.push_back(newPublisher);
    return true;
}

// ----------------------------------------------------
// Method for Adding Publishers !!! Only for StampedPoses
// ----------------------------------------------------

bool mtsROSBridge::AddPublisherFromReadCommandStamped(const std::string & interfaceRequiredName,
                                               const std::string & functionName,
                                               const std::string & topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }
    if (!interfaceRequired) {
        ROS_ERROR("mtsROS::AddPublisherFromReadCommandStamped: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromReadCommandStamped: faild to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        return false;
    }
    mtsROSPublisherBaseStamped * newPublisher = new mtsROSPublisherStamped(topicName, *(this->Node));
    if (!interfaceRequired->AddFunction(functionName, newPublisher->Function)) {
        ROS_ERROR("mtsROS::AddPublisherFromReadCommandStamped: failed to create function.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromReadCommandStamped: faild to create function \""
                                 << functionName << "\"" << std::endl;
        delete newPublisher;
        return false;
    }
    PublishersStamped.push_back(newPublisher);
    return true;
}


CMN_DECLARE_SERVICES_INSTANTIATION(mtsROSBridge);

#endif // _mtsROSBridge_h
