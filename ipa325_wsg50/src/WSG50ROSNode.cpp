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

// Services
//

// Programm Control
//
#define DEBUG       true
#define TIMEOUT     500         // in milliseconds
#define TIMEFORWAITINGLOOP 20   // in milliseconds
#define PUBLISHINGINTERVAL 10   // in Herz

// Gripper Default Values
//
#define FORCELIMIT  10          // in Newton
#define ACCELERATION 200        // in mm/s²


// local variables
//
bool            g_active = true;
static WSG50Controller * _controller = NULL;

/*
 * handle SIGINT messages
 *
 */
void mySigintHandler(int)
{
    ROS_WARN("Received SIGINT!");
    g_active = false;
    delete _controller;
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
        if(DEBUG) ROS_WARN("doHoming() called!!");
        goal_ = (unsigned int) homingserver_.acceptNewGoal()->direction;
        _controller->homing(goal_);
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
    // get instance of the node handle
    //
    ros::NodeHandle node;

    // define publishing interval (in Herz)
    //
    ros::Rate r(PUBLISHINGINTERVAL);

    // set update cycle
    //
    short updatePeriodInMs = 20;
//    _controller->getOpeningWidthUpdates(false, true, updatePeriodInMs);
//    if(!ready(TIMEOUT)) ROS_ERROR("timout occured while setting update rate for opening width");
//    _controller->getSpeedUpdates(false, true, updatePeriodInMs);
//    if(!ready(TIMEOUT)) ROS_ERROR("timeout occured while setting update rate for speed");
//    _controller->getForceUpdates(false, true, updatePeriodInMs);
//    if(!ready(TIMEOUT)) ROS_ERROR("timeout occured while setting update rate for force");


    // start publishing loop
    //
    while(ros::ok() && g_active) {

        // TODO


        // spin once (for incomming messages)
        //
        ros::spinOnce();

        // wait
        //
        r.sleep();
    }
}

/**
 *  Main method
 */
int main(int argc, char** argv)
{
    // Declare variables
    //


    // initialize ROS
    //
    ros::init(argc, argv, "WSG50Gripper");
    ros::NodeHandle node;

    // overwrite default sigint handler
    //
    signal(SIGINT, mySigintHandler);

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


    // TODO:
    // Subscribe to actions
    //
    ROS_INFO("subscribe to action-servers");
    //WSG50HomingAction homing(ros::this_node::getName());
    WSG50HomingAction homing("WSG50Gripper_Homing");
    WSG50PrePositionFingersActionServer prepFingers("WSG50Gripper_PrePositionFingers");
    WSG50GraspPartActionServer gpserver("WSG50Gripper_GraspPartAction");
    WSG50ReleasePartActionServer rpserver("WSG50Gripper_ReleasePartAction");


    // spin and send messages
    //
    //ros::spin(); // probably not necessary, since this is called in the publishStates method

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

