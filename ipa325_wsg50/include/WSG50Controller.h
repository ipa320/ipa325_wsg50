#pragma once
/**
 * @brief The WSG50Controller class
 *
 * this class is an Ethernet-Controller for the Schunk WSG-50-Gripper
 * In a first version it will offer following functions
 *
 * - read:
 *  - width
 *  - speed
 *  - forcelimit
 *  - acceleration
 *
 * - command:
 *  - width
 *  - speed
 *  - forcelimit
 *  - acceleration
 *  - move
 *  - stop
 *  - homing (reset gripper to home position)
 */

#include "WSG50Communicator.h"
#include "WSG50Observer.h"
#include "WSG50ROSNode.h"
#include "WSG50Subject.h"
#include "WSG50RosSubject.h"
#include "WSG50RosObserver.h"

#include <mutex>
#include <queue>
#include <string>

/*!
 * Manufacturer's default network config
 */
#define DEFAULTIP   "192.168.1.20"
#define DEFAULTPORT "1000"

/*!
 * Commands
 */
// Connection Manager
static short _LOOP              = 0x06;     // Loop-Back
static short _DISCONNECT        = 0x07;     // announce Disconnect -> stops every movement

// Motion Control
static short _HOMING            = 0x20;     // homing movement
                                            // params: 0: default value; 1: homing in positive movement dir; 2: homing in negative movement dir.
static short _PREPFINGERS       = 0x21;     // Pre-position Fingers: move fingers to defined opening width
static short _STOP              = 0x22;     // Stop
static short _FASTSTOP          = 0x23;     // Issue Fast-Stop
static short _ACKFASTSTOP       = 0x24;     // Acknowledgement FastStop
static short _GRASPPART         = 0x25;     // Grasp part by passing normal width and grasping-speed
static short _RELEASEPART       = 0x26;     // Release a previously grasped part

// Motion Configuration
static short _SETACC            = 0x30;     // Set Acceleration
static short _GETACC            = 0x31;     // Get Acceleration
static short _SETFORCELIMIT     = 0x32;     // Set force-limit (float) in Newtons
static short _GETFORCELIMIT     = 0x33;     // Get force-limit
static short _SETSOFTLIMIT      = 0x34;     // Set soft limits for both the minus and plus direction (float minus, float plus)
static short _GETSOFTLIMIT      = 0x35;     // Get soft limits
static short _CLRSOFTLIMIT      = 0x36;     // Clear any set soft limits
static short _TAREFORCESENSOR   = 0x38;     // zeroes the connected force sensor used for the force control loop

// system state commands
static short _GETSYSTSTATE      = 0x40;     // Get the current system state
static short _GETGRASPSTATE     = 0x41;     // Get the current grasping state
                                            // idle = 0; grasping = 1; no part found = 2; part lost = 3; holding = 4
                                            // releasing = 5; positioning = 6; error = 7
static short _GETGRASPSTATS     = 0x42;     // get current grasping statistics for the number of executed grasps
static short _GETWIDTH          = 0x43;     // Get opening width
static short _GETSPEED          = 0x44;     // Get the current finger speed
static short _GETFORCE          = 0x45;     // Get the current grasping force
static short _GETTEMP           = 0x46;     // Get the current device temperature

// system configuration
static short _GETSYSTINFO       = 0x50;     // Get information about the connected device
static short _SETDEVTAG         = 0x51;     // Set the device tag (string)
static short _GETDEVTAG         = 0x52;     // Get the device tag
static short _GETSYSTLIMITS     = 0x53;     // Get the gripper's physical limits for stroke, speed, acceleration and force

// finger interface
static short _GETFINGERINFO     = 0x60;     // Return information about the connected fingers
static short _GETFINGERFLAGS    = 0x61;     // Return the state flags for the selected finger (params: int fingerindex)
static short _FINGERPOWERCTRL   = 0x62;     // enables or disables the power supply for the selected finter (params: int fingerindex, enum ON/OFF)
static short _GETFINGERDATA     = 0x63;     // return the current finger data for predefined finger types (params: int fingerindex)




class WSG50Controller : public WSG50Observer, WSG50RosSubject
{


public:

    // Connection Manager
    //
    WSG50Controller(void);
    WSG50Controller(std::string ip, std::string port, double max_width);
    virtual ~WSG50Controller(void);


    /**
     *  ####################################
     *  ###### MOTION CONTROL       ########
     *  ####################################
     *
     */
    bool    ready(void);        // ready for next command
    /*
     *homing:
     *@params: unsigned int direction
     *      0: default
     *      1: positive
     *      2: negative
     */
    void    homing(void);       // using default value 0
    void    homing(unsigned int direction);
    void    prePositionFingers(bool stopOnBlock, float width, float speed); // position finger prior to a grap movement
    void    stop(void);         // stop
    void    fastStop(void);     // stop immediately and prevent any further motion-related commands
    void    ackFastStop(void);  // if fast-stop was issued, this will bring the Gripper back to normal mode
    void    grasp(float width,  // grasp a part
                  float speed); // width: nominal width of the part; speed: grasping speed in mm/s
    void    release(float openWidth,    // the opening width
                    float speed);       // the opening-speed in mm/s

    /**
     *  ####################################
     *  ###### MOTION CONFIGURATION ########
     *  ####################################
     *
     */
    void    setAcceleration(float acceleration); // set the acceleration
    void    setForceLimit(float forcelimit);    // set the grasping-force-limit
    void    setSoftLimits(float minusLimit,     // set the soft-limits
                          float plusLimit);     // minus: negative motion direction; plus: positive motion direction
    void    clearSoftLimits(void);      // clear all prior set soft limits
    bool    areSoftLimitsSet(void);
    void    tareForceSensor(void);      // zeroes the connected force sensor


    /**
     *  ####################################
     *  ###### GETTER METHODS       ########
     *  ####################################
     *
     */

    float   getMaxWidth();
    float   getMinWidth();
    float   getMaxSpeed();
    float   getMinSpeed();
    float   getMaxAcceleration();
    float   getMinAcceleration();
    float   getMaxForceLimit();
    float   getMinForceLimit();

    // Gripper Stats
    //
    float   getWidth(void);
    float   getSpeed(void);
    float   getForce(void);
    int     getTemperature(void);

    float   getAcceleration(void);
    float   getAccelerationFromCache(void);
    float   getForceLimit(void);
    float   getForceLimitFromCache(void);
    void    getSoftLimits(float * softLimits); // expects a float array with at least the size of 2
    void    loop();
    bool    isCommunicationOk(void);
    std::string getSystemInformation(void);
    std::string getSystemTag(void);

    /**
     *  ####################################
     *  ###### SYSTEM STATE COMMANDS #######
     *  ####################################
     */
    SSTATE  getSystemState(bool updateOnChangeOnly,
                           bool enableAutoUpdate,
                           short updatePeriodInMillisec);
    int     getGraspingState(void);
    void    getGraspingStateUpdates(bool updateOnChangeOnly,
                             bool enableAutoUpdate,
                             short updatePeriodInMillisec);
    void    getOpeningWidthUpdates(bool updateOnChangeOnly, // 1 = yes; 0 = update always
                                   bool automaticUpdatesEnabled, // 1 = yes; 0 = no
                                   short updatePeriodInMs);
    void    getSpeedUpdates(bool updateOnChangeOnly, // 1 = yes; 0 = update always
                            bool automaticUpdatesEnabled, // 1 = yes; 0 = no
                            short updatePeriodInMs);
    void    getForceUpdates(bool updateOnChangeOnly, // 1 = yes; 0 = update always
                            bool automaticUpdatesEnabled, // 1 = yes; 0 = no
                            short updatePeriodInMs);


    /**
     *  #####################################
     *  ###### OVERLOADING OBSERVER METHODS #
     *  #####################################
     */
    using WSG50Observer::update;
    void update(TRESPONSE * resp);

    /**
     *  #####################################
     *  ###### OVERLOADING SUBJECT METHODS ##
     *  #####################################
     */
    void Attach(WSG50RosObserver * observer_, unsigned int msgId_);
    void Detach(WSG50RosObserver * observer_, unsigned int msgId_);

private:

    std::string _IP;
    std::string _PORT;

    // variables
    //
    int         _currentGraspingState;

    float       _MaxWidth,      // mm
                _MinWidth,
                _MaxSpeed,
                _MinSpeed,
                _MaxForceLimit,
                _MinForceLimit,
                _MaxAcceleration,
                _MinAcceleration,
                _acceleration,      // mm/s²
                _speed,             // mm/s
                _softLimitMinus,    //
                _softLimitPlus,     //
                _forceLimit,        // N

                // current system data
                _currentOpeningWidth,
                _currentSpeed,
                _currentForce,
                _currentForceLimit,
                _currentTemperature;

    TMESSAGE    _msg;
    TRESPONSE   _resp;
    std::queue<TRESPONSE> _responseQueue;
    SSTATE      _systemState;

    WSG50Communicator   *_wsgComm;
    WSG50Observer       *_observer;

    bool        _checkingCommunication,
                _communicationOK,
                _ready,
                _systStatesReadyForCommand,
                _widthAutoUpdate,
                _speedAutoUpdate,
                _forceAutoUpdate,
                _graspingStateAutoUpdates,
                _softLimitsSet;
    unsigned char * _LoopTestData;
    unsigned char * _dat;
    int         _LoopTestDataLength;

    // threading
    //
    std::mutex      _msgMutex,
                    _currentForceMutex,
                    _currentSpeedMutex,
                    _currentWidthMutex,
                    _currentGraspingStateMutex,
                    _responseMutex;

    // run State-Machine
    //
    void setupConnection();


    // Update handler
    //
    void updateHandler();
    void notifyObserver(unsigned int msgId, TRESPONSE * resp);


};
