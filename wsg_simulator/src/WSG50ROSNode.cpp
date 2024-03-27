#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <chrono>
#include <thread>

#include <sensor_msgs/JointState.h>

#include "WSG50ROSNode.h"
#include "WSG50Controller.h"
#include "WSG50Observer.h"
#include "WSG50Subject.h"
#include "WSG50RosObserver.h"

// Actions
//
#include <actionlib/server/simple_action_server.h>
#include <ipa325_wsg50/WSG50HomingAction.h>
#include <ipa325_wsg50/WSG50PrePositionFingersAction.h>
#include <ipa325_wsg50/WSG50GraspPartAction.h>
#include <ipa325_wsg50/WSG50ReleasePartAction.h>

// Messages
//
#include <ipa325_wsg50/systState.h>

// Services
//
#include <ipa325_wsg50/setAcceleration.h>
#include <ipa325_wsg50/setForceLimit.h>
#include <ipa325_wsg50/setSoftLimits.h>
#include <ipa325_wsg50/tareForceSensor.h>
#include <std_srvs/Empty.h>

#include <wsg_simulator/WSG50Simulator.h>

// Programm Control
//
#define DEBUG true
#define TIMEOUT 500           // in milliseconds
#define TIMEFORWAITINGLOOP 20 // in milliseconds
#define PUBLISHINGINTERVAL 20 // in Herz

// Gripper Default Values
//
#define MAXWIDTH 110.0 
#define FORCELIMIT 10    // in Newton
#define ACCELERATION 200 // in mm/s²

// Gripper finger joint name
//
#define DEFAULTJOINTNAME "finger_left"
#define DEFAULTOPENINGJOINTNAME "gripper_opening"

// local variables
//
bool g_active = true;
static WSG50Simulator *_controller = nullptr;

/*
 * handle SIGINT messages
 *
 */
void mySigintHandler(int)
{
    ROS_WARN("Received SIGINT!");
    g_active = false;
    if (DEBUG)
        ROS_INFO("trying to delete controller");
    delete _controller;
    if (DEBUG)
        ROS_INFO("Controller deleted!");
    _controller = nullptr;
}

bool ready(int timeout)
{
    int counter = 0;
    while (!_controller->ready())
    {
        if (counter > TIMEOUT)
        {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMEFORWAITINGLOOP));
        counter++;
    }
    return true;
}

/*
 *  HOMING-Action
 */
class WSG50HomingAction : public WSG50RosObserver
{
protected:
    ros::NodeHandle nh_;
    // node handle must be called first
    actionlib::SimpleActionServer<ipa325_wsg50::WSG50HomingAction> homingserver_;
    std::string action_name_;
    // create messages that are used to publish feedback/result
    ipa325_wsg50::WSG50HomingFeedback feedback_;
    ipa325_wsg50::WSG50HomingResult result_;

    unsigned int goal_;

public:
    // Constructor
    WSG50HomingAction(std::string name) : homingserver_(nh_, name, false),
                                          action_name_(name)
    {
        _name = 200000;

        ROS_WARN("initializing homing action.");
        // register goal and feedback callbacks
        homingserver_.registerGoalCallback(std::bind(&WSG50HomingAction::doHoming, this));

        // start server
        homingserver_.start();
    }

    // homing action
    void doHoming()
    {
        if (DEBUG)
            ROS_INFO("\n\n##########################\n##  Homing...\n##########################");
        goal_ = (unsigned int)homingserver_.acceptNewGoal()->direction;
        int movementDirection = 0;
        if (goal_ == 1)
            movementDirection = 1;
        else
            movementDirection = 2;

        bool success = _controller->homing(movementDirection);
        if (success)
        {
            this->homingserver_.setSucceeded(this->result_);
        }
        else
        {
            this->homingserver_.setAborted(this->result_);
        }
    }
};

/*
 *  Pre-Position Fingers Action
 */
class WSG50PrePositionFingersActionServer : public WSG50RosObserver
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ipa325_wsg50::WSG50PrePositionFingersAction> prepositionserver_;
    std::string action_name;
    // feedbak & result messages
    ipa325_wsg50::WSG50PrePositionFingersFeedback feedback_;
    ipa325_wsg50::WSG50PrePositionFingersResult result_;
    bool stopOnBlock_;
    float width_, speed_;

public:
    // constructor
    WSG50PrePositionFingersActionServer(std::string name) : prepositionserver_(nh_, name, false),
                                                            action_name(name)
    {
        // register callback
        prepositionserver_.registerGoalCallback(std::bind(&WSG50PrePositionFingersActionServer::doPrePositionFingers, this));
        // start server
        prepositionserver_.start();
    }

    void doPrePositionFingers()
    {
        if (DEBUG)
            ROS_INFO("\n\n##########################\n##  Preposition Fingers\n##########################");
        // accept new goal and get values
        //
        ipa325_wsg50::WSG50PrePositionFingersGoalConstPtr goal;
        goal = prepositionserver_.acceptNewGoal();
        stopOnBlock_ = goal->stopOnBlock;
        width_ = goal->width;
        speed_ = goal->speed;

        // debug information
        if (DEBUG)
            ROS_INFO("Preposition Fingers Action called with following params: stopOnBlock = %d, width = %f, speed = %f",
                     (int)stopOnBlock_, width_, speed_);

        // trigger action
        bool success = _controller->prePositionFingers(stopOnBlock_, width_, speed_);

        if (success)
        {
            this->prepositionserver_.setSucceeded(this->result_);
        }
        else
        {
            this->prepositionserver_.setAborted(this->result_);
        }
    }
};

/*
 * Grasping Part Action
 * (for more detailed comments have a look at preposition fingers class or homing class)
 */
class WSG50GraspPartActionServer : public WSG50RosObserver
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ipa325_wsg50::WSG50GraspPartAction> gpserver_;
    std::string action_name;
    ipa325_wsg50::WSG50GraspPartFeedback fb_;
    ipa325_wsg50::WSG50GraspPartResult res_;
    float width_, speed_;

public:
    WSG50GraspPartActionServer(std::string name) : gpserver_(nh_, name, false), action_name(name)
    {
        gpserver_.registerGoalCallback(std::bind(&WSG50GraspPartActionServer::doGrasp, this));
        gpserver_.start();
    }

    void doGrasp()
    {
        if (DEBUG)
            ROS_INFO("\n\n##########################\n##  Grasping...\n##########################");
        ipa325_wsg50::WSG50GraspPartGoalConstPtr goal = gpserver_.acceptNewGoal();
        width_ = goal->width;
        speed_ = goal->speed;
        if (DEBUG)
            ROS_INFO("Grasp Part Action called with following params: width = %f, speed = %f", width_, speed_);
        bool success = _controller->grasp(width_, speed_);
        if (success)
        {
            this->gpserver_.setSucceeded(res_);
        }
        else
        {
            this->gpserver_.setAborted(res_);
        }
    }
};

/*
 * Releasing Part Action
 * (for more detailed comments have a look at preposition fingers class or homing class)
 */
class WSG50ReleasePartActionServer : public WSG50RosObserver
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ipa325_wsg50::WSG50ReleasePartAction> rpserver_;
    std::string action_name;
    ipa325_wsg50::WSG50ReleasePartFeedback fb_;
    ipa325_wsg50::WSG50ReleasePartResult res_;
    float width_, speed_;

public:
    WSG50ReleasePartActionServer(std::string name) : rpserver_(nh_, name, false), action_name(name)
    {
        rpserver_.registerGoalCallback(std::bind(&WSG50ReleasePartActionServer::doRelease, this));
        rpserver_.start();
    }

    void doRelease()
    {
        if (DEBUG)
            ROS_INFO("\n\n##########################\n##  Release Part...\n##########################");
        ipa325_wsg50::WSG50ReleasePartGoalConstPtr goal = rpserver_.acceptNewGoal();
        width_ = goal->openwidth;
        speed_ = goal->speed;
        if (DEBUG)
            ROS_INFO("Release Part Action called with following params: width = %f, speed = %f", width_, speed_);

        bool success = _controller->release(width_, speed_);
        if (success)
        {
            this->rpserver_.setSucceeded(res_);
        }
        else
        {
            this->rpserver_.setAborted(res_);
        }
    }
};

/*
 *  runs a loop to publish gripper states in ROS
 */
void publishStates(const std::string &jointName, const std::string &openingJointName)
{
    float softLimits[2];

    // get instance of the node handle
    //
    ros::NodeHandle node;

    // define publishing interval (in Herz)
    //
    ros::Rate r(PUBLISHINGINTERVAL);

    // define publishers
    //
    ipa325_wsg50::systState systStateMsg;
    ros::Publisher statePublisher = node.advertise<ipa325_wsg50::systState>("system_state", 10);
    ros::Publisher jointStatePublisher = node.advertise<sensor_msgs::JointState>("joint_states", 1);

    // start publishing loop
    //
    while ( // ros::ok() &&
        g_active)
    {

        // check if ros is ok
        if (!ros::ok())
        {
            ROS_ERROR("ros is not ok!");
            break;
        }

        // get soft limits
        //
        if (_controller->areSoftLimitsSet())
        {
            _controller->getSoftLimits(softLimits);
        }
        else
        {
            softLimits[0] = 0;    // default values over the whole range
            softLimits[1] = 1000; // default values over the whole range
        }

        // fill message
        //
        systStateMsg.width = _controller->getWidth();
        systStateMsg.speed = _controller->getSpeed();
        systStateMsg.force = _controller->getForce();
        systStateMsg.forcelimit = _controller->getForceLimitFromCache();
        systStateMsg.acceleration = _controller->getAcceleration();
        systStateMsg.softlimit_minus = softLimits[0];
        systStateMsg.softlimit_plus = softLimits[1];
        systStateMsg.softlimitsset = _controller->areSoftLimitsSet();
        systStateMsg.grasp_state = _controller->getGraspingState();

        // publish message
        //
        statePublisher.publish(systStateMsg);

        // publish joint position
        //
        sensor_msgs::JointState jointStateMsg;
        jointStateMsg.header.stamp = ros::Time::now();
        jointStateMsg.name.push_back(jointName);
        // the gripper returns the total opening but we need the value of
        // a single finger. This can then be combined with a mimic joint for
        // the second finger
        jointStateMsg.position.push_back(systStateMsg.width / 1000.0 / 2.0);
        jointStateMsg.velocity.push_back(systStateMsg.speed / 1000.0);
        jointStateMsg.effort.push_back(systStateMsg.force / 1000.0);

        // alternatively you can also use the full gripper opening
        //  should not be published as joint state, since there is no joint with this state. Use systStateMsg if you need full opening.
        //  jointStateMsg.name.push_back(openingJointName);
        //  jointStateMsg.position.push_back(systStateMsg.width/1000.0);
        //  jointStateMsg.velocity.push_back(systStateMsg.speed/1000.0);
        //  jointStateMsg.effort.push_back(systStateMsg.force/1000.0);

        jointStatePublisher.publish(jointStateMsg);

        // spin once (for incomming messages)
        //
        ros::spinOnce();

        // wait
        //
        r.sleep();
    }

    if (!g_active)
    {
        if (DEBUG)
            ROS_WARN("Received SIGINT! shut down publisher now.");
    }
    else
    {
        ROS_WARN("Shutting down publisher without SIGINT!");
    }
}

// ************************************************************************
// ros service:
// this will set the acceleration of the schunk gripper
// there is no response code, since the values are published in the system states message
//
bool setAccelerationService(ipa325_wsg50::setAcceleration::Request &req,
                            ipa325_wsg50::setAcceleration::Response &res)
{
    ROS_INFO("Set acceleration = %f", req.acceleration);

    // set acceleration
    //
    _controller->setAcceleration(req.acceleration);

    // return
    return true;
}

// ************************************************************************
// ros service: SetSoftLimits
// this will set the minus and plus soft-limits of the schunk gripper
// there is no response code, since the values are published in the system states message
//
bool setSoftLimitsService(ipa325_wsg50::setSoftLimits::Request &req,
                          ipa325_wsg50::setSoftLimits::Response &resp)
{
    ROS_INFO("Set soft limits: minus = %f; plus = %f", req.limit_minus, req.limit_plus);

    // set soft limits
    //
    _controller->setSoftLimits((float)req.limit_minus, (float)req.limit_plus);

    // return
    return true;
}

// ************************************************************************
// ros service: Clear Soft Limits
// this will set the minus and plus soft-limits of the schunk gripper
// there is no response code, since the values are published in the system states message
//
bool clearSoftLimitsService(std_srvs::Empty::Request &req,
                            std_srvs::Empty::Response &resp)
{
    ROS_INFO("Clear Soft Limits service called");

    // clear soft limits
    //
    _controller->clearSoftLimits();

    return true;
}

// ************************************************************************
// ros service: set force limit (newton)
// this will set the force limit of the schunk gripper
// there is no response code, since the values are published in the system states message
//
bool setForceLimitService(ipa325_wsg50::setForceLimit::Request &req,
                          ipa325_wsg50::setForceLimit::Response &resp)
{
    ROS_INFO("Set Force Limits service called; ForceLimit = %f", req.force);

    // set force limits
    //
    _controller->setForceLimit(req.force);
    return true;
}

// ************************************************************************
// ros service: stop
// this will stop gripper movements
//
bool stopService(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &resp)
{
    ROS_INFO("STOP service called");

    // set force limits
    //
    _controller->stop();
    return true;
}

// ************************************************************************
// ros service: fast stop
// this will stop gripper movements immediately
// in this state the gripper will not accept any other movement orders until
// "acknowledgeFastStop" is called!
//
bool fastStopService(std_srvs::Empty::Request &req,
                     std_srvs::Empty::Response &resp)
{
    ROS_INFO("FASTSTOP service called");

    // set force limits
    //
    _controller->fastStop();
    return true;
}

// ************************************************************************
// ros service: acknowledge fast stop
// return gripper into idle state where new commands can be sent.
//
bool acknowledgeFastStopService(std_srvs::Empty::Request &req,
                                std_srvs::Empty::Response &resp)
{
    ROS_INFO("Acknowledge fast stop service called");

    // acknowledge fast stop
    //
    _controller->ackFastStop();
    return true;
}

/**
 *  Main method
 */
int main(int argc, char **argv)
{
    // initialize ROS
    //
    ros::init(argc, argv, "WSG50Gripper");
    ros::NodeHandle node;

    // overwrite default sigint handler
    //
    signal(SIGINT, mySigintHandler);
    g_active = true;

    // Initialize controller
    //
    std::string ip, port;
    std::string jointName, openingJointName;
    double max_width;
    ros::param::param<std::string>("~joint_name", jointName, DEFAULTJOINTNAME);
    ros::param::param<std::string>("~opening_joint_name", openingJointName, DEFAULTOPENINGJOINTNAME);
    ros::param::param<double>("~max_width", max_width, MAXWIDTH);
    try
    {
        _controller = new WSG50Simulator(max_width);
    }
    catch (int e)
    {
        ROS_ERROR("An exception occured while initializing the WSG50Simulator!\nError: %d\nAborting further processing", e);
        return 0;
    }

    // wait until controller is ready
    if (!ready(TIMEOUT))
    {
        ROS_ERROR("A timeout occured while waiting for the controller.");
        return 0;
    }

    // start publishing loop in separate thread
    //
    ROS_INFO("call publishStates() method");
    std::thread t(publishStates, std::ref(jointName), std::ref(openingJointName));

    // wait XX miliseconds, so the publishing-cycle can become active
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Subscribe to actions
    //
    ROS_INFO("subscribe to action-servers");
    WSG50HomingAction homing("WSG50Gripper_Homing");
    WSG50PrePositionFingersActionServer prepFingers("WSG50Gripper_PrePositionFingers");
    WSG50GraspPartActionServer gpserver("WSG50Gripper_GraspPartAction");
    WSG50ReleasePartActionServer rpserver("WSG50Gripper_ReleasePartAction");

    // subscribe to services
    //
    ROS_INFO("subscribe to services");
    ros::ServiceServer acc = node.advertiseService("SetAcceleration", setAccelerationService);
    ros::ServiceServer sls = node.advertiseService("SetSoftLimits", setSoftLimitsService);
    ros::ServiceServer csl = node.advertiseService("ClearSoftLimits", clearSoftLimitsService);
    ros::ServiceServer sfl = node.advertiseService("SetForceLimit", setForceLimitService);
    ros::ServiceServer s = node.advertiseService("Stop", stopService);
    ros::ServiceServer fs = node.advertiseService("FastStop", fastStopService);
    ros::ServiceServer afs = node.advertiseService("AcknowledgeFastStop", acknowledgeFastStopService);

    // make ros spin
    //
    //    ros::spin();
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // disconnect
    //
    ROS_INFO("waiting for publisher to finish");
    t.join();

    ROS_WARN("closing down ROS-Node");
    if (_controller != nullptr)
    {
        delete _controller;
    }

    // shutdown ROS
    //
    ros::shutdown();

    return 0;
}