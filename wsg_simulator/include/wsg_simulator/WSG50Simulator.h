#include <WSG50Controller.h>

// #ifndef MAXWIDTH
// #define MAXWIDTH 110.0 -> use parameter instead to allow using wsg50-210 as well
// #endif

#ifndef MINWIDTH
#define MINWIDTH 0.0
#endif

#ifndef MAXSPEED
#define MAXSPEED 420.0
#endif

#ifndef MINSPEED
#define MINSPEED 5.0
#endif

#ifndef MAXFORCELIMIT
#define MAXFORCELIMIT 80.0
#endif

#ifndef MINFORCELIMIT
#define MINFORCELIMIT 5.0
#endif

#ifndef MAXACCELERATION
#define MAXACCELERATION 5000.0
#endif

#ifndef MINACCELERATION
#define MINACCELERATION 100.0
#endif

#ifndef DEFAULTACCELERATION
#define DEFAULTACCELERATION 500.0
#endif

#ifndef DEFAULTSPEED
#define DEFAULTSPEED 50.0
#endif

#ifndef DEFAULTFORCELIMIT
#define DEFAULTFORCELIMIT 10.0
#endif

class WSG50Simulator
{

public:
    // Connection Manager
    //
    WSG50Simulator(double max_width);
    virtual ~WSG50Simulator(void);

    /**
     *  ####################################
     *  ###### MOTION CONTROL       ########
     *  ####################################
     *
     */
    bool ready(void); // ready for next command
    /*
     *homing:
     *@params: unsigned int direction
     *      0: default
     *      1: positive
     *      2: negative
     */
    bool homing(void); // using default value 0
    bool homing(unsigned int direction);
    bool prePositionFingers(bool stopOnBlock, float width, float speed); // position finger prior to a grap movement
    void stop(void);                                                     // stop
    void fastStop(void);                                                 // stop immediately and prevent any further motion-related commands
    void ackFastStop(void);                                              // if fast-stop was issued, this will bring the Gripper back to normal mode
    bool grasp(float width,                                              // grasp a part
               float speed);                                             // width: nominal width of the part; speed: grasping speed in mm/s
    bool release(float openWidth,                                        // the opening width
                 float speed);

    /** grasping state
        * 0 = Idle
        * 1 = Grasping
        * 2 = No Part Found
        * 3 = Part lost
        * 4 = Holding
        * 5 = Releasing
        * 6 = Positioning
        * 7 = Error void
     */
    void setGraspingState(int i);

    /**
     *  ####################################
     *  ###### MOTION CONFIGURATION ########
     *  ####################################
     *
     */
    void setAcceleration(float acceleration); // set the acceleration
    void setForceLimit(float forcelimit);     // set the grasping-force-limit
    void setSoftLimits(float minusLimit,      // set the soft-limits
                       float plusLimit);      // minus: negative motion direction; plus: positive motion direction
    void clearSoftLimits(void);               // clear all prior set soft limits
    bool areSoftLimitsSet(void);
    void tareForceSensor(void); // zeroes the connected force sensor

    /**
     *  ####################################
     *  ###### GETTER METHODS       ########
     *  ####################################
     *
     */

    float getMaxWidth();
    float getMinWidth();
    float getMaxSpeed();
    float getMinSpeed();
    float getMaxAcceleration();
    float getMinAcceleration();
    float getMaxForceLimit();
    float getMinForceLimit();

    // Gripper Stats
    //
    float getWidth(void);
    float getSpeed(void);
    float getForce(void);
    int getTemperature(void);

    float getAcceleration(void);
    float getAccelerationFromCache(void);
    float getForceLimit(void);
    float getForceLimitFromCache(void);
    void getSoftLimits(float *softLimits); // expects a float array with at least the size of 2

    int getGraspingState(void);

    bool moveToPosition(float width, float speed);

private:
    int _currentGraspingState;

    float _MaxWidth, // mm
        _MinWidth,
        _MaxSpeed,
        _MinSpeed,
        _MaxForceLimit,
        _MinForceLimit,
        _MaxAcceleration,
        _MinAcceleration,
        _acceleration,   // mm/s²
        _speed,          // mm/s
        _softLimitMinus, //
        _softLimitPlus,  //
        _forceLimit,     // N

        // current system data
        _currentOpeningWidth,
        _currentSpeed,
        _currentForce,
        _currentForceLimit,
        _currentTemperature;


    bool _ready,
        _softLimitsSet;

    // threading
    //
    std::mutex _currentSpeedMutex,
        _currentWidthMutex,
        _currentGraspingStateMutex;
};