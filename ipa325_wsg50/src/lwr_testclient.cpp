
#include <ros/ros.h>
#include <tf/tf.h>


#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <signal.h>


// includes for WSG50Gripper tests
//
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <ipa325_wsg50/WSG50HomingAction.h>
#include <ipa325_wsg50/WSG50PrePositionFingersAction.h>
#include <ipa325_wsg50/WSG50GraspPartAction.h>
#include <ipa325_wsg50/WSG50ReleasePartAction.h>

// include services
//


// contents
//

boost::mutex    g_mutex;


#define DEBUG true

#ifndef PI
    #define PI			3.1415926535897932384626433832795
#endif

#ifndef RAD
    #define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
    #define DEG(A)	((A) * 180.0 / PI )
#endif


bool    g_active = true;
bool    action_active = false;


void mySigintHandler(int)
{
    ROS_WARN("Received SIGINT!");

    g_active = false;
}


// goal callback
// is called, once the goal is completed
//
void prepFingersDoneCB(const actionlib::SimpleClientGoalState &state_,
                       const ipa325_wsg50::WSG50PrePositionFingersResultConstPtr &result_)
{
    if(DEBUG) ROS_INFO("Preposition Fingers DONE!");

    action_active = false;

    // ...
}

// active callback
// is called, once the activity has started execution
//
void prepFingersActiveCB()
{
    if(DEBUG) ROS_INFO("Preposition Fingers ACTIVE now!");

    // ...
}

// feedback callback
// called every time feedback for this goal is available
//
void prepFingersFeedbackCB(const ipa325_wsg50::WSG50PrePositionFingersFeedbackConstPtr &feedback_)
{
    if(DEBUG) {
        ROS_INFO("Feedback: width = %f; speed = %f; force = %f;", feedback_->width, feedback_->speed, feedback_->force);
    }

    // ...
}

/*
 * ROS Node
 * Tester for WSG50 Gripper
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "schunk_testclient");

    ros::NodeHandle node;


    // Override the default ros sigint handler //
    //
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);
    g_active = true;

    // define loop-rate:
    ros::Rate r(1); // 10 Hz

    ROS_INFO("Starting testclient");


   /* **************************************
    * test functions
    * *************************************/


#if 1
    // test homing
    //
    actionlib::SimpleActionClient<ipa325_wsg50::WSG50HomingAction> homingclient("WSG50Gripper_Homing", true);
    // the actionlib does the magic, if you provide the name of the action-server.

    ROS_INFO("Waiting for action server to start.");
    if(!homingclient.waitForServer(ros::Duration(20.0)))  // wait for infinite time
    {
        ROS_ERROR("Run into timout while waiting for server");
        return 1;
    }

    // send goal
    //
    ROS_INFO("Actionserver started, send homing");
    ipa325_wsg50::WSG50HomingGoal goal;
    goal.direction = 1; // 1 = positive movement direction; 0=negative movement direction
    homingclient.sendGoal(goal);

    // wait for feedback and result
    //

    bool finished_before_timeout = homingclient.waitForResult(ros::Duration(30.0));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = homingclient.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else {
        ROS_INFO("Homing-Action did not finish before the time out.");
        return 1;
    }
#endif

#if 1
    // test preposition fingers
    //
    ROS_INFO("Wait for x seconds");
    boost::this_thread::sleep(boost::posix_time::millisec(2000));
    ROS_INFO("Testclient: test preposition fingers");

    // connect to action server
    actionlib::SimpleActionClient<ipa325_wsg50::WSG50PrePositionFingersAction>
            prepositionclient_("WSG50Gripper_PrePositionFingers", true);
    if(!prepositionclient_.waitForServer(ros::Duration(20.0))) {
        ROS_ERROR("Run into timeout while waiting for the preposition action server.");
        return 1;
    }

    // send action goal
    ipa325_wsg50::WSG50PrePositionFingersGoal goal2;
    goal2.stopOnBlock = true;
    goal2.width = 55.0;
    goal2.speed = 50.0;
    prepositionclient_.sendGoal(goal2, &prepFingersDoneCB, &prepFingersActiveCB, &prepFingersFeedbackCB);

    // wait until action has completed
    action_active = true;
    while(action_active && g_active) {ros::spinOnce();}

#endif

#if 1
    // test grasp part
    //
    boost::this_thread::sleep(boost::posix_time::millisec(500));
    ROS_INFO("Testclient: Grasping");
    // connect
    actionlib::SimpleActionClient<ipa325_wsg50::WSG50GraspPartAction> gpclient_("WSG50Gripper_GraspPartAction", true);
    if(!gpclient_.waitForServer(ros::Duration(20.0))) {
        ROS_ERROR("Run into timeout while waiting for the grasp-part action server.");
        return 1;
    }
    // send action goal
    ipa325_wsg50::WSG50GraspPartGoal g;
    g.width = 25.0;
    g.speed = 30.0;
    gpclient_.sendGoal(g);
    if(gpclient_.waitForResult(ros::Duration(30)))
        ROS_INFO("Grasping successfull!");
    else {
        ROS_ERROR("Run into timeout while grasping!");
        return 1;
    }
#endif

#if 1
    // test release part
    //
    boost::this_thread::sleep(boost::posix_time::millisec(500));
    ROS_INFO("Testclient: releasing");
    // connect
    actionlib::SimpleActionClient<ipa325_wsg50::WSG50ReleasePartAction> rpclient_("WSG50Gripper_ReleasePartAction", true);
    if(!rpclient_.waitForServer(ros::Duration(20.0))) {
        ROS_ERROR("Run into timeout while waiting for the release-part action server.");
        return 1;
    }
    // send action goal
    ipa325_wsg50::WSG50ReleasePartGoal gr;
    gr.openwidth = 73.33;
    gr.speed = 70.0;
    rpclient_.sendGoal(gr);
    if(rpclient_.waitForResult(ros::Duration(30)))
        ROS_INFO("releasing part successfull!");
    else {
        ROS_ERROR("Run into timeout while releasing part!");
        return 1;
    }
#endif

    // exit
    return 0;
}
