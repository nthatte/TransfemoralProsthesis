/*
 * ControllerManager-component.h
 *
 *  Created on: Jul 26, 2012
 *      Author: Michael Anderson
 */

#ifndef OROCOS_ATRIASCONTROLLERMANAGER_COMPONENT_H
#define OROCOS_ATRIASCONTROLLERMANAGER_COMPONENT_H

#include <rtt/os/TimeService.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/scripting/ScriptingService.hpp>
#include <rtt/scripting/StateMachine.hpp>
#include <rtt/os/Mutex.hpp>
#include <rtt/os/MutexLock.hpp>
#include <iostream>
#include <map>

#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <ros/package.h>

#include <robot_variant_defs.h>
#include <robot_invariant_defs.h>

#include <atrias_shared/controller_metadata.h>
#include <atrias_shared/globals.h>
#include <atrias_msgs/gui_output.h>
#include <atrias_msgs/gui_input.h>
#include <atrias_msgs/rt_ops_status.h>

#define RT_OPS_WAIT_TIMEOUT_SECS 2.0
#define CONTROLLER_LEVEL_DELIMITER "__"

using namespace std;
using namespace RTT;
using namespace atrias_msgs;

namespace atrias {
namespace controllerManager {
class ControllerManager: public TaskContext {
private:
    InputPort<gui_output> guiDataIn; //The data coming in from the GUI
    OutputPort<gui_input> guiDataOut; //The data being sent to the GUI

    InputPort<rt_ops_status> rtOpsDataIn; //The data coming in from RT Ops
    OutputPort<uint8_t> rtOpsDataOut; //The data being sent to RT Ops

    gui_output guiOutput;
    gui_input guiInput;
    rt_ops_status rtOpsOutput;

    os::Mutex nameCacheMutex;

    ControllerManagerError lastError;
    ControllerManagerState state;

    pid_t rosbagPID;   // PID of the child process executing 'roslaunch atrias rosbag.launch'.
    bool controllerLoaded;

    std::map<string, uint16_t> controllerChildCounts;

    string currentControllerName;
    controllerMetadata::ControllerMetadata metadata;
    //boost::shared_ptr<scripting::ScriptingService> scriptingProvider;
    scripting::ScriptingService::shared_ptr scriptingProvider;

    bool loadController(string controllerName);
    void unloadController();
    bool runController(string path);
    bool loadStateMachine(string path);
    void updateGui(); //Send updated information to the GUI
    void throwEstop();
    void resetRtOps();
    bool eStopFlagged(boost::array<uint8_t, NUM_MEDULLAS_ON_ROBOT> statuses);
    bool waitForRtOpsState(RtOpsCommand state);

    // These functions are provided as operations to sub-controller start
    // scripts to help them assign unique-names to their child controllers
    string getUniqueName(string parentName, string childType);
    void resetControllerNames();

public:
    ControllerManager(string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};

}
}

#endif