#include "wsg_simulator/WSG50Simulator.h"
#include <mutex>
#include <thread>


/*
 *  Constructor
 */
WSG50Simulator::WSG50Simulator(double max_width)
{
    this->_currentOpeningWidth = 0;
    this->_currentSpeed = 0;
    this->_currentGraspingState = 0;
    this->_MaxWidth = max_width;
    this->_MinWidth = MINWIDTH;
    this->_MaxSpeed = MAXSPEED;
    this->_MinSpeed = MINSPEED;
    this->_MaxAcceleration = MAXACCELERATION;
    this->_MinAcceleration = MINACCELERATION;
    this->_MaxForceLimit = MAXFORCELIMIT;
    this->_MinForceLimit = MINFORCELIMIT;
    this->_softLimitMinus = MINWIDTH;
    this->_softLimitPlus = this->_MaxWidth;
    this->_acceleration = DEFAULTACCELERATION; // mm/s²
    this->_speed = DEFAULTSPEED;               // mm/s
    this->_forceLimit = DEFAULTFORCELIMIT;     // N
    this->_currentForceLimit = DEFAULTFORCELIMIT;

    _ready = true;
}

/*
 *  Destructor
 */
WSG50Simulator::~WSG50Simulator(void)
{
}


// returns true if the gripper is currently moving
//
bool WSG50Simulator::ready(void)
{
    return _ready;
}

/**
 *  ####################################
 *  ###### MOTION CONTROL       ########
 *  ####################################
 *
 */

/*
 *  STOP
 *  this will stop any motion
 */
void WSG50Simulator::stop()
{
    ROS_WARN("WSG50Simulator: stop not implemented yet");
}

/*
 *  FAST STOP
 *  immediately stop any motion and prevent any further motion-related commands from beeing executed
 *  Requires FAST STOP Acknowledgement message to release lock
 */
void WSG50Simulator::fastStop()
{
    ROS_WARN("WSG50Simulator: fastStop not implemented yet");
}

/*
 *  acknowledge previouslz issued FAST STOP or severe error
 *  to release motion-lock
 */
void WSG50Simulator::ackFastStop()
{
    ROS_WARN("WSG50Simulator: ackFastStop not implemented yet");
}

bool WSG50Simulator::homing()
{
    return homing(0);
}

/*
 *  perform homing activity
 */
bool WSG50Simulator::homing(unsigned int direction)
{
    // move to max./ min width
    if (direction == 1)
    {
        return moveToPosition(this->_MaxWidth, 30);
    }
    else
    {
        return moveToPosition(0, 30);
    }
}

/*
 * Pre-Position Fingers before graping parts
 */
bool WSG50Simulator::prePositionFingers(bool stopOnBlock, float width, float speed)
{
    return moveToPosition(width, speed);
}

/*
 *  Grasp Part
 *  @params:
 *      float width: in mm
 *      float speed: in mm/s
 */
bool WSG50Simulator::grasp(float width, float speed)
{
    if (!_ready)
    {
        ROS_WARN("Gripper not ready!");
        return false;
    }
    // check max and min values
    //
    if (width < _MinWidth)
    {
        width = _MinWidth;
    }
    else if (width > _MaxWidth)
    {
        width = _MaxWidth;
    }

    if (speed < _MinSpeed)
    {
        speed = _MinSpeed;
    }
    else if (speed > _MaxSpeed)
    {
        speed = _MaxSpeed;
    }
    setGraspingState(1);
    bool success =  moveToPosition(width, speed);
    setGraspingState(4);
    return success;
}

bool WSG50Simulator::release(float openWidth, float speed)
{
    if (!_ready)
    {
        ROS_WARN("Gripper not ready!");
        return false;
    }
    // check max and min values
    //
    if (openWidth < _MinWidth)
    {
        openWidth = _MinWidth;
    }
    else if (openWidth > _MaxWidth)
    {
        openWidth = _MaxWidth;
    }

    if (speed < _MinSpeed)
    {
        speed = _MinSpeed;
    }
    else if (speed > _MaxSpeed)
    {
        speed = _MaxSpeed;
    }
    setGraspingState(5);
    bool success =  moveToPosition(openWidth, speed);
    setGraspingState(0);
    return true;
}

void WSG50Simulator::setGraspingState(int i)
{
    std::lock_guard<std::mutex> g(_currentGraspingStateMutex);
    _currentGraspingState = i;
}

/**
 *  ####################################
 *  ###### MOTION CONFIGURATION ########
 *  ####################################
 *
 */

void WSG50Simulator::setAcceleration(float acceleration)
{
    ROS_WARN("WSG50Simulator: setAcceleration not implemented yet");
}

void WSG50Simulator::setForceLimit(float forcelimit)
{
    // nothing to do here
}

/*
 *  Set operating sof limits (for grasping parts)
 *  @param:
 *      float minusLimit: prevents the fingers from moving into a certain minimum width
 *      float plusLimit: prevents the fingers from moving into a certain maximum width
 *  Measurement: mm
 */
void WSG50Simulator::setSoftLimits(float minusLimit, float plusLimit)
{
    ROS_WARN("WSG50Simulator: setSoftLimits not implemented yet");
}

void WSG50Simulator::clearSoftLimits()
{
    ROS_WARN("WSG50Simulator: clearSoftLimits not implemented yet");
}

/*
 *  Zeroes the connected force sensor used for the force control loop
 *
 *  WARNING: this command is only allowed if not in force control mode (i.e. grasping state must not be HOLDING)
 */
void WSG50Simulator::tareForceSensor()
{
    ROS_WARN("WSG50Simulator: tareForceSensor not implemented yet");
}

/**
 *  ####################################
 *  ###### GETTER METHODS       ########
 *  ####################################
 *
 */
float WSG50Simulator::getMaxWidth() { return this->_MaxWidth; }
float WSG50Simulator::getMinWidth() { return this->_MinWidth; }
float WSG50Simulator::getMaxSpeed() { return this->_MaxSpeed; }
float WSG50Simulator::getMinSpeed() { return this->_MinSpeed; }
float WSG50Simulator::getMaxAcceleration() { return this->_MaxAcceleration; }
float WSG50Simulator::getMinAcceleration() { return this->_MinAcceleration; }
float WSG50Simulator::getMaxForceLimit() { return this->_MaxForceLimit; }
float WSG50Simulator::getMinForceLimit() { return this->_MinForceLimit; }
bool WSG50Simulator::areSoftLimitsSet() { return this->_softLimitsSet; }

float WSG50Simulator::getWidth(void)
{
    std::lock_guard<std::mutex> g(_currentWidthMutex);
    return _currentOpeningWidth;
}

float WSG50Simulator::getSpeed(void)
{
    std::lock_guard<std::mutex> g(_currentSpeedMutex);
    return _currentSpeed;
}

float WSG50Simulator::getForce(void)
{
    return 0;
}


// not asynchroneous
//
void WSG50Simulator::getSoftLimits(float *softLimits)
{ 
    softLimits[0] = _softLimitMinus;
    softLimits[1] = _softLimitPlus;
    return;
}

// return last set force limit, without sending a message to the gripper
//
float WSG50Simulator::getForceLimitFromCache()
{
    return _forceLimit;
}

// not asynchroneous
//
float WSG50Simulator::getForceLimit()
{
    return _forceLimit;
}

// return last set acceleration
// while avoiding another message to the gripper
// this may not be as accurate as other messages
//
float WSG50Simulator::getAccelerationFromCache()
{
    return _acceleration;
}

// not asynchron.
float WSG50Simulator::getAcceleration()
{
    return _acceleration;
}


/*
 * @return:
 *  0 = Idle
 *  1 = Grasping
 *  2 = No Part Found
 *  3 = Part lost
 *  4 = Holding
 *  5 = Releasing
 *  6 = Positioning
 *  7 = Error
 *  every thing else = researved
 */
int WSG50Simulator::getGraspingState()
{
    std::lock_guard<std::mutex> g (_currentGraspingStateMutex);
    return _currentGraspingState;
}

bool WSG50Simulator::moveToPosition(float width, float speed)
{
    if (!_ready)
    {
        ROS_WARN("Gripper not ready!");
        return false;
    }
    //ROS_INFO_STREAM("WSG50Simulator: Move to width " << width << " from width " << getWidth());
    _ready = false;
    ros::Rate r(100);
    float missing_width = width - this->getWidth();
    float direction = missing_width > 0 ? 1.0 : -1.0;
    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> g(_currentWidthMutex);
            std::lock_guard<std::mutex> g2(_currentSpeedMutex);
            float missing_current = width - _currentOpeningWidth;
            float required_speed = std::abs(missing_current) / 0.01;
            if (speed <= required_speed)
            {
                _currentOpeningWidth = _currentOpeningWidth + direction * speed * 0.01;
                _currentSpeed = speed;
            }
            else
            {
                _currentOpeningWidth = width;
                _currentSpeed = 0;
                break;
            }
            //ROS_INFO_STREAM("WSG50Simulator: Current width =  " << _currentOpeningWidth);
        }

        r.sleep();
    }
    _ready = true;
    return true;
}
