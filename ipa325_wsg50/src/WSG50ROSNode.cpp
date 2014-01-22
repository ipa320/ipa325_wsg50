#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

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
#include <ipa325_wsg50/clearSoftLimits.h>
#include <ipa325_wsg50/tareForceSensor.h>

// Programm Control
//
#define DEBUG       true
#define TIMEOUT     500         // in milliseconds
#define TIMEFORWAITINGLOOP 20   // in milliseconds
#define PUBLISHINGINTERVAL 20   // in Herz

// Gripper Default Values
//
#define FORCELIMIT  10          // in Newton
#define ACCELERATION 200        // in mm/s²


// local variables
//
bool                        g_active = true;
static WSG50Controller * _controller = NULL;

/*
 * handle SIGINT messages
 *
 */
void mySigintHandler(int)
{
    ROS_WARN("Received SIGINT!");
    g_active = false;
    if(DEBUG) ROS_INFO("trying to delete controller");
    delete _controller;
    if(DEBUG) ROS_INFO("Controller deleted!");
    _controller = 0;
}

bool ready(int timeout)
{
    int counter = 0;
    while(!_controller->ready()) {
        if(counter > TIMEOUT) {
            return false;
        }
        boost::this_thread::sleep(boost::posix_time::millisec(TIMEFORWAITINGLOOP));
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
    WSG50HomingAction(std::string name) :
        homingserver_(nh_, name, false),
        action_name_(name)
    {
        _name = 200000;

        ROS_WARN("initializing homing action.");
        // register goal and feedback callbacks
        homingserver_.registerGoalCallback(boost::bind(&WSG50HomingAction::doHoming, this));


        // subscribe result/feedback callback methods at controller
        // todo
        _controller->Attach(this, 0x20);
        // _controller->Attach(this, 6); // for testing purpose

        // start server
        homingserver_.start();
    }

    // Destructor
    ~WSG50HomingAction(void)
    {
        _controller->Detach(this, 0x21);
    }

    // homing action
    void doHoming()
    {
        if(DEBUG) ROS_INFO("\n\n##########################\n##  Homing...\n##########################");
        goal_ = (unsigned int) homingserver_.acceptNewGoal()->direction;
        int movementDirection = 0;
        if(goal_ == 1)
            movementDirection = 1;
        else
            movementDirection = 2;
        _controller->homing(movementDirection);
    }


    /*
     *  override parent method
     */
    using WSG50RosObserver::update;
    void update(TRESPONSE *response)
    {

        // if pending or already running, send feedback
        //
        if(response->status_code == E_CMD_PENDING
                || response->status_code == E_ALREADY_RUNNING)
        {
            result_.status_code = response->status_code;
            this->homingserver_.isActive();
        }
        // if success-response
        else if(response->status_code == E_SUCCESS) {
            result_.status_code = response->status_code;
            this->homingserver_.setSucceeded(this->result_);
        }
        // if other response message
        else {
            result_.status_code = response->status_code;
            this->homingserver_.setAborted(this->result_);
        }

        return;
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
    WSG50PrePositionFingersActionServer(std::string name) :
        prepositionserver_(nh_, name, false),
        action_name(name)
    {
        // register callback
        prepositionserver_.registerGoalCallback(boost::bind(&WSG50PrePositionFingersActionServer::doPrePositionFingers, this));
        // register observer
        _controller->Attach(this, 0x21);
        // start server
        prepositionserver_.start();
    }

    ~WSG50PrePositionFingersActionServer(void) {_controller->Detach(this, 0x21);}

    void doPrePositionFingers()
    {
        if(DEBUG) ROS_INFO("\n\n##########################\n##  Prepositoin Fingers\n##########################");
        // accept new goal and get values
        //
        ipa325_wsg50::WSG50PrePositionFingersGoalConstPtr goal;
        goal = prepositionserver_.acceptNewGoal();
        stopOnBlock_ = goal->stopOnBlock;
        width_ = goal->width;
        speed_ = goal->speed;

        // debug information
        if(DEBUG) ROS_INFO("Preposition Fingers Action called with following params: stopOnBlock = %d, width = %f, speed = %f",
                           (int) stopOnBlock_, width_, speed_);

        // subscribe for width updates
        //
        _controller->Attach(this, 0x43);

        // trigger action
        _controller->prePositionFingers(stopOnBlock_, width_, speed_);
    }

    void update(TRESPONSE *response)
    {
        if(response->id == 0x21) {
            if(DEBUG) ROS_INFO("node: send feedback / result message");
            if(response->status_code == E_ACCESS_DENIED ||
                    response->status_code == E_NOT_INITIALIZED ||
                    response->status_code == E_RANGE_ERROR ||
                    response->status_code == E_CMD_FORMAT_ERROR ||
                    response->status_code == E_INSUFFICIENT_RESOURCES ||
                    response->status_code == E_AXIS_BLOCKED ||
                    response->status_code == E_TIMEOUT ||
                    response->status_code == E_CMD_ABORTED)
            {
                result_.status_code = response->status_code;
                this->prepositionserver_.setAborted(result_);
                // detach observer from width updates
                _controller->Detach(this, 0x43);
            } else if(response->status_code == E_SUCCESS)
            {
                result_.status_code = response->status_code;
                this->prepositionserver_.setSucceeded(result_);
                // detach observer from width updates
                _controller->Detach(this, 0x43);
            } else if(response->status_code == E_CMD_PENDING)
            {
                // ??
            }
        } else if(response->id == 0x43) {
            feedback_.width = _controller->getWidth();
            feedback_.force = _controller->getForce();
            feedback_.speed = _controller->getSpeed();

            this->prepositionserver_.publishFeedback(feedback_);
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
    WSG50GraspPartActionServer(std::string name) : gpserver_(nh_, name, false), action_name(name) {
        gpserver_.registerGoalCallback(boost::bind(&WSG50GraspPartActionServer::doGrasp, this));
        _controller->Attach(this,0x25);
        gpserver_.start();
    }

    ~WSG50GraspPartActionServer() {_controller->Detach(this, 0x25);}

    void doGrasp()
    {
        if(DEBUG) ROS_INFO("\n\n##########################\n##  Grasping...\n##########################");
        ipa325_wsg50::WSG50GraspPartGoalConstPtr goal = gpserver_.acceptNewGoal();
        width_ = goal->width;
        speed_ = goal->speed;
        if(DEBUG) ROS_INFO("Grasp Part Action called with following params: width = %f, speed = %f", width_, speed_);
        _controller->Attach(this, 0x43);
        _controller->grasp(width_, speed_);
    }

    void update(TRESPONSE *response)
    {
        if(response->id==0x25) {        // send result response
            res_.status_code=response->status_code;
            if(response->status_code==E_ACCESS_DENIED ||
                    response->status_code==E_ALREADY_RUNNING ||
                    response->status_code==E_CMD_FORMAT_ERROR ||
                    response->status_code==E_RANGE_ERROR ||
                    response->status_code==E_CMD_ABORTED ||
                    response->status_code==E_CMD_FAILED ||
                    response->status_code==E_TIMEOUT) {
                gpserver_.setAborted(res_);
            } else if(response->status_code==E_SUCCESS) {
                gpserver_.setSucceeded(res_);
            }
            _controller->Detach(this, 0x43);
        } else if(response->id==0x43) { // send feedback response
            fb_.force=_controller->getForce();
            fb_.speed=_controller->getSpeed();
            fb_.width=_controller->getWidth();
            gpserver_.publishFeedback(fb_);
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
    WSG50ReleasePartActionServer(std::string name) : rpserver_(nh_, name, false), action_name(name) {
        rpserver_.registerGoalCallback(boost::bind(&WSG50ReleasePartActionServer::doRelease, this));
        _controller->Attach(this,0x26);
        rpserver_.start();
    }

    ~WSG50ReleasePartActionServer() {_controller->Detach(this, 0x26);}

    void doRelease()
    {
        if(DEBUG) ROS_INFO("\n\n##########################\n##  Release Part...\n##########################");
        ipa325_wsg50::WSG50ReleasePartGoalConstPtr goal = rpserver_.acceptNewGoal();
        width_ = goal->openwidth;
        speed_ = goal->speed;
        if(DEBUG) ROS_INFO("Release Part Action called with following params: width = %f, speed = %f", width_, speed_);
        _controller->Attach(this, 0x43);
        _controller->release(width_, speed_);
    }

    void update(TRESPONSE *response)
    {
        if(response->id==0x26) {        // send result response
            res_.status_code=response->status_code;
            if(response->status_code==E_ACCESS_DENIED ||
                    response->status_code==E_ALREADY_RUNNING ||
                    response->status_code==E_CMD_FORMAT_ERROR ||
                    response->status_code==E_RANGE_ERROR ||
                    response->status_code==E_CMD_ABORTED ||
                    response->status_code==E_TIMEOUT) {
                rpserver_.setAborted(res_);
            } else if(response->status_code==E_SUCCESS) {
                rpserver_.setSucceeded(res_);
            }
            _controller->Detach(this, 0x43);
        } else if(response->id==0x43) { // send feedback response
            fb_.force=_controller->getForce();
            fb_.speed=_controller->getSpeed();
            fb_.width=_controller->getWidth();
            rpserver_.publishFeedback(fb_);
        }
    }
};



/*
 *  runs a loop to publish gripper states in ROS
 */
void publishStates()
{
    float softLimits[2];

    // get instance of the node handle
    //
    ros::NodeHandle node;

    // define publishing interval (in Herz)
    //
    ros::Rate r(PUBLISHINGINTERVAL);

    // subscribe to regular updates
    //
    if(DEBUG) ROS_INFO("subscribing to autoupdates for width, speed, force and grasping state.");
    int updatePeriodInMs = PUBLISHINGINTERVAL;
    _controller->getOpeningWidthUpdates(false, true, updatePeriodInMs);
    boost::this_thread::sleep(boost::posix_time::millisec((TIMEFORWAITINGLOOP*5)));
    _controller->getSpeedUpdates(false, true, updatePeriodInMs);
    boost::this_thread::sleep(boost::posix_time::millisec((TIMEFORWAITINGLOOP*5)));
    _controller->getForceUpdates(false, true, updatePeriodInMs);
    boost::this_thread::sleep(boost::posix_time::millisec((TIMEFORWAITINGLOOP*5)));
    _controller->getGraspingStateUpdates(false, true, updatePeriodInMs);
    boost::this_thread::sleep(boost::posix_time::millisec((TIMEFORWAITINGLOOP*5)));

    ROS_INFO("publishStates(): subscribed to updates of 'widht', 'speed', 'force', 'grasping state'");

    // define publishers
    //
    ipa325_wsg50::systState systStateMsg;
    ros::Publisher  statePublisher = node.advertise<ipa325_wsg50::systState>("system_state", 10);

    // start publishing loop
    //
    while(//ros::ok() &&
          g_active
          ) {

        // check if ros is ok
        if(!ros::ok()) {
            ROS_ERROR("ros is not ok!");
            break;
        }

        // get soft limits
        //
        ROS_INFO("get soft limits");
        if(_controller->areSoftLimitsSet()) {
            _controller->getSoftLimits(softLimits);
        } else {
            softLimits[0] = 10000;
            softLimits[1] = 10000;
        }
        ROS_INFO("Soft Limits = %f; %f", softLimits[0], softLimits[1]);

        // fill message
        //
        systStateMsg.width = _controller->getWidth();
        systStateMsg.speed = _controller->getSpeed();
        systStateMsg.force = _controller->getForce();
        systStateMsg.acceleration = _controller->getAcceleration();
        systStateMsg.softlimit_minus = softLimits[0];
        systStateMsg.softlimit_plus = softLimits[1];
        systStateMsg.softlimitsset = _controller->areSoftLimitsSet();
        systStateMsg.grasp_state = _controller->getGraspingState();

        // publish message
        //
        statePublisher.publish(systStateMsg);

        // spin once (for incomming messages)
        //
        ros::spinOnce();

        // wait
        //
        r.sleep();
    }

    if(!g_active) {
        if(DEBUG) ROS_WARN("Received SIGINT! shut down publisher now.");
    } else {
        ROS_WARN("Shutting down publisher without SIGINT!");
    }
}

// ************************************************************************
// ros service:
// this will set the acceleration of the schunk gripper
// there is no response code, since the values are published in the system states message
//
bool setAcceleration(ipa325_wsg50::setAcceleration::Request &req,
                    ipa325_wsg50::setAcceleration::Response &res) {

    ROS_INFO("Set acceleration = %f", req.acceleration);

    // set acceleration
    //
    _controller->setAcceleration(req.acceleration);

    // return
    return true;
}



/**
 *  Main method
 */
int main(int argc, char** argv)
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
    try {
        _controller = new WSG50Controller();
    } catch (int e) {
        ROS_ERROR("An exception occured while initializing the WSG50Controller!\nError: %d\nAborting further processing", e);
        return 0;
    }

    // wait until controller is ready
    if(!ready(TIMEOUT)) {ROS_ERROR("A timeout occured while waiting for the controller."); return 0;}

    // set default values
    //
    _controller->setAcceleration(ACCELERATION);
    if(!ready(TIMEOUT)) {ROS_ERROR("A timeout occured while waiting for the controller."); return 0;}

    _controller->setForceLimit(FORCELIMIT);
    if(!ready(TIMEOUT)) {ROS_ERROR("A timeout occured while waiting for the controller."); return 0;}

    // start publishing loop in separate thread
    //
    ROS_INFO("call publishStates() method");
    boost::thread t(publishStates);

    // wait XX miliseconds, so the publishing-cycle can become active
    boost::this_thread::sleep(boost::posix_time::millisec(500));

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
    ros::ServiceServer acc  = node.advertiseService("SetAcceleration", setAcceleration);

    // make ros spin
    //
//    ros::spin();

    // disconnect
    //
    ROS_INFO("waiting for publisher to finish");
    t.join();

    ROS_WARN("closing down ROS-Node");
    if(_controller != 0) {
        delete _controller;
    }

    // shutdown ROS
    //
    ros::shutdown();

    return 0;
}

