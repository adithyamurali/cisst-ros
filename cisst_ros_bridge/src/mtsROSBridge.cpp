/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsROSBridge.cpp 4363 2013-07-16 20:32:30Z zchen24 $

  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-21

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <signal.h>   // ROS only supports Linux
#include "cisst_ros_bridge/mtsROSBridge.h"
#include <ros/ros.h>

CMN_IMPLEMENT_SERVICES(mtsROSBridge);


mtsROSBridge::mtsROSBridge(const std::string & componentName, double periodInSeconds, bool spin, bool sig):
    mtsTaskPeriodic(componentName, periodInSeconds),
  mSpin(spin), mSignal(sig)
{
    typedef char * char_pointer;
    char_pointer * argv = new char_pointer[1];
    argv[0]= new char[strlen("mtsROSBridge") + 1];
    strcpy(argv[0], "mtsROSBridge");
    int argc = 1;
    if (mSignal) ros::init(argc, argv, componentName);
    else ros::init(argc, argv, componentName, ros::init_options::NoSigintHandler);

    Node = new ros::NodeHandle;
}

void mtsROSBridge::Configure(const std::string & CMN_UNUSED(filename))
{
}

void mtsROSBridge::Startup(void)
{
}

void mtsROSBridge::Cleanup(void)
{
  if (!mSignal) ros::requestShutdown();
}

void mtsROSBridge::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    ros::Time startTime = ros::Time::now();

    const PublishersTypeStamped::iterator end = PublishersStamped.end();
    PublishersTypeStamped::iterator iter;
    for (iter = PublishersStamped.begin();
         iter != end;
         ++iter) {
        (*iter)->Execute(startTime);
    }

    const PublishersType::iterator end2 = Publishers.end();
    PublishersType::iterator iter2;
    for (iter2 = Publishers.begin();
         iter2 != end2;
         ++iter2) {
        (*iter2)->Execute();
    }

    if (mSpin) ros::spinOnce();
}

bool mtsROSBridge::AddPublisherFromEventVoid(const std::string &interfaceRequiredName,
                                             const std::string &eventName,
                                             const std::string &topicName)
{
    // check if the interface exists of try to create one
    mtsInterfaceRequired * interfaceRequired = this->GetInterfaceRequired(interfaceRequiredName);
    if (!interfaceRequired) {
        interfaceRequired = this->AddInterfaceRequired(interfaceRequiredName);
    }

    mtsROSEventVoidPublisher* newPublisher = new mtsROSEventVoidPublisher(topicName, *(this->Node));
    if (!interfaceRequired->AddEventHandlerVoid(&mtsROSEventVoidPublisher::EventHandler, newPublisher, eventName))
    {
        ROS_ERROR("mtsROS::AddPublisherFromEventVoid: failed to create required interface.");
        CMN_LOG_CLASS_INIT_ERROR << "AddPublisherFromEventVoid: faild to create required interface \""
                                 << interfaceRequiredName << "\"" << std::endl;
        delete newPublisher;
        return false;
    }
    Publishers.push_back(newPublisher);
    return true;
}

bool mtsROSBridge::AddSubscriberToVoidCommand(const std::string &interfaceRequiredName,
                                              const std::string &functionName,
                                              const std::string &topicName)
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

    mtsROSSubscriberVoid * newSubscriber = new mtsROSSubscriberVoid(topicName, *(this->Node));
    if (!interfaceRequired->AddFunction(functionName, newSubscriber->FunctionVoid)) {
      ROS_ERROR("mtsROS::AddSubscriberToVoidCommand: failed to create function.");
      CMN_LOG_CLASS_INIT_ERROR << "AddSubscriberToVoidCommand: failed to create function \""
                               << functionName << "\"" << std::endl;
      delete newSubscriber;
      return false;
    }
    Subscribers.push_back(newSubscriber);
    return true;
}






