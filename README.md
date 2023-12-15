ipa325_wsg50
=========================================

This repository contains **Schunk WSG50-110 ROS-Driver**.

This is a ros-driver for the [Schunk WSG50-110 Gripper](https://www.weiss-robotics.com/en/produkte/gripping-systems/performance-line-en/wsg-50-en/)

Note that there is also an other ROS driver now: http://wiki.ros.org/wsg50


## Acknowledgements
This project is a result of the LIAA project.
http://www.project-leanautomation.eu/

![LIAA](http://www.project-leanautomation.eu/fileadmin/img/LIAALogo/Logo_LIAA.png "LIAA")

![EC](http://www.project-leanautomation.eu/typo3temp/pics/b3ba71db31.jpg "EC")

LIAA received funding from the European Union’s Seventh Framework Programme for research, technological development and demonstration under grant agreement no. 608604.

Project runtime: 02.09.2013 – 31.08.2017.

## Prerequisites

On the WSG 50 Control panel (open WSG ip in browser, without https), go to `Settings/Command Interface` and ensure that "Use text based interface" is disabled.

## ROS-Node Documentation
Special Thanks to Mainauthor **Florian Röser**
Current maintainer **Lorenz Halt**

this section contains the API of the ROS-Node for the WSG50 Driver.

The ROS node for the WSG50 Gripper makes use of:
* ROS Messages (see http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
* ROS Services (see http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
* ROS actionlib (see http://wiki.ros.org/actionlib_tutorials/Tutorials)

### Actions
Actions includes all actions provided by the ros-driver for the schunk wsg50 gripper.

* WSG50HomingAction: Provide an homing action.
* Homing Action: This action has to be run before any other motion command is send.
 1. @goal: bool direction
    - positive direction
    - negative movement direction
 2. @result: int status_code (see possible [Status Codes](#status-codes) for detailed description)
 3. @feedback
    - float width (in mm)
    - float speed (in mm/s)
    - float force (in newton)

* WSG50PrePositionFingersAction: Pre-Position fingers with given widht and speed.
 1. @goal params: bool stopOnBlock: defines if the pre-position movement should stop if something is in the way
    - float width (in mm)
    - float speed (in mm/s)
 2. @result: int status_code (see possible [Status Codes](#status-codes) for detailed description)
 3. @feedback
    - float width (in mm)
    - float speed (in mm/s)
    - float force (in newton)
* WSG50GraspPartAction
* WSG50ReleasePartAction

### ROS Messages
includes the messages and topics sent by the ROS-Driver

#### System State Message

Advertized as ROS-topic: */system_state*

this message will be published about every 20 milliseconds.

### ROS Services
includes the services provided by the ROS-Driver

* setAcceleration.srv: This sets the acceleration for the gripper motions, which are the movements of the fingers.
* setSoftLimits.srv: This service will set the soft limits (minus and plus) for the gripper motions
* clearSoftLimits.srv: **Note:** Even if the soft limits are cleared, the ros-wrapper will return soft limits in the system-state message, but these soft limits will then be equal to the maximum and minimum width of the finger movements.
* setForceLimit.srv: The force-limit defines with which maximum force the fingers will try to move, or at which force-threshold the fingers will stop moving.
* stop.srv: this will stop any motion
* fastStop.srv: this will stop any motion and put the gripper in a state where it requires the message "Acknowledge Fast Stop" before it can accept any other command!
* ackFastStop.srv: This will release the fast-stop state so the gripper can accept new commands


### Grasping States
Please consider the Schunk documentation for the grasping state graph!

When you are programming the Gripper, then you should be aware of these states, e.g. it is not possible to send the "Grasping" command when the gripper is in the state "No Part Found"!

### Command Overview
Most Gripper Commands are already implemented into the WSG50Controller.cpp class, but still may need to be implemented as ROS-Services or ROS-Actions.

### Connection Management
| Gripper Command | Description | ROS-Service / Activity |
| --------------- |:-----------:| ----------------------:|
| LOOP            | Send message and loop back this message | (issued at startup to test connection)
| DISCONNECT      | announce disconnect -> stops every movement | (issued at shutdown of the driver)


### Motion Control
| Gripper Command | Description | ROS-Service / Activity |
| --------------- |:-----------:| ----------------------:|
| HOMING | homing movement, required at connection before sending other motion commands | Action: WSG50HomingAction
| PREPOSITION FINGERS | pre-position fingers moves the fingers to a defined opening width | Action: WSG50PrePositionFingersAction
| STOP | stop all motions | Service: stop.srv
| FASTSTOP | Issue fast-stop (this requires acknowledge command before any other motion command can be issued) | Service: fastStop.srv
| ACKNOWLEDGE FASTSTOP | Acknowledge fast-stop (make other commands possible again) | Service: ackFastStop.srv
| GRASP PART | grasp part by passing normal width and grasping speed | Action: WSG50GraspPartAction
| RELEASE PART | release previously grasped part (to issue this command, the grasping state must be either one of the three: "HOLDING", "PART LOST", "NO PART" | Action: WSG50ReleasePartAction

### Motion Configuration
| Gripper Command | Description | ROS-Service / Activity |
| --------------- |:-----------:| ----------------------:|
| SET ACCELERATION | set acceleration | Service: setAcceleration.srv
| GET ACCELERATION | get acceleration | (published in systState.msg)
| SET FORCE LIMIT | set force-limit in newtons | Service: setForceLimit.srv
| GET FORCE LIMIT | get force-limit | (published in systState.msg)
| SET SOFT LIMITS | set soft-limits for plus and minus direction || Service: setSoftLimits.srv
| GET SOFT LIMITS | get soft-limits | (published in systState.msg)
| CLEAR SOFT LIMITS | clear soft limits | Service: clearSoftLimits.srv
| TARE FORCE SENSORS | zeroes the connected force sensor used for the force control loop | -- (todo)

### System State Commands
Getter methods to read the different system states like temperature, force and speed. Most of these values are already published in a single ROS-Message ''systState'' with a defined frequency (by default every 20ms).

| Gripper Command | Description | ROS-Service / Activity |
| --------------- |:-----------:| ----------------------:|
| GET SYSTEM STATE | get the current system state | -- (todo)
| GET GRASPING STATE | get the current grasping state | (published in systState.msg)
| GET GRASPING STATISTICS | get current grasping statistics for the number of executed grasps | -- (todo)
| GET OPENING WIDTH | get current opening width | (published in systState.msg)
| GET SPEED | get the current finger speed | (published in systState.msg)
| GET FORCE | get the current grasping force | (published in systState.msg)
| GET TEMPERATURE | get the current device temperature | -- (todo)

### System Configuration
These commands include the general system configuration commands. 

| Gripper Command | Description | ROS-Service / Activity |
| --------------- |:-----------:| ----------------------:|
| GET SYSTEM INFO | get information about the connected device | -- (todo)
| SET DEVICE TAG | set the device tag | -- (todo)
| GET DEVICE TAG | get the device tag | -- (todo)
| GET SYSTEM LIMITS | get the gripper's physical limits for stroke, speed, acceleration and force | -- (todo)

### Finger Interface
(Source Weiss Robotics:) ''The WSG series of grippers provides a sensor port n each base jaw to which sensor fingers can be connected to. The following commands are used to access and control these fingers.''

| Gripper Command | Description | ROS-Service / Activity |
| --------------- |:-----------:| ----------------------:|
| GET FINGER INFORMATION | return information about the connected fingers | -- (todo)
| GET FINGER FLAGS | return the state flags for the selected fingers | -- (todo)
| FINGER POWER CONTROL | enables or disables the power supply for the selected finger | -- (todo)
| GET FINGER DATA | return the current finger data for predefined finger types | -- (todo)


## Status Codes

the returned status-codes are int-values. Below is the short description:

| Code | Name | Description |
| -----|:----:| -----------:|
| 0 | E_SUCCESS | No error
| 1 | E_NOT_AVAILABLE | Device, service or data is not avialable
| 2 | E_NO_SENSOR | No sensor connected
| 3 | E_NOT_INITIALIZED | The device is not initialized
| 4 | E_ALREADY_RUNNING | Service is already running
| 5 | E_FEATURE_NOT_SUPPORTED | The asked feature is not supported
| 6 | E_INCONSISTENT_DATA | One or more dependent parameters mismatch
| 7 | E_TIMEOUT | Timeout error
| 8 | E_READ_ERROR | Error while reading from a device
| 9 | E_WRITE_ERROR | Error while writing to a device
| 10 | E_INSUFFICIENT_RESOURCES | No memory available
| 11 | E_CHECKSUM_ERROR | Checksum error
| 12 | E_NO_PARAM_EXPECTED | No parameters expected
| 13 | E_NOT_ENOUGH_PARAMS | Not enough parameters
| 14 | E_CMD_UNKNOWN | Unknown command
| 15 | E_CMD_FORMAT_ERROR | Comand format error
| 16 | E_ACCESS_DENIED | Access denied
| 17 | E_ALREADY_OPEN | The interface is already open
| 18 | E_CMD_FAILED | Command failed
| 19 | E_CMD_ABORTED | Command aborted
| 20 | E_INVALID_HANDLE | Invalid handle
| 21 | E_NOT_FOUND | Device not found
| 22 | E_NOT_OPEN | Device not open
| 23 | E_IO_ERROR | I/O error
| 24 | E_INVALID_PARAMETER | Invalid parameter
| 25 | E_INDEX_OUT_OF_BOUNDS | Index out of bounds
| 26 | E_CMD_PENDING | Command execution needs more time
| 27 | E_OVERRUN | Data overrun
| 28 | E_RANGE_ERROR | Range error
| 29 | E_AXIS_BLOCKED | Axis is blocked
| 30 | E_FILE_EXISTS | File already exists


### Datatype definition:
 * typedef enum TStat
 * typedef struct SSTATE
