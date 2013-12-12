#include "testerMain.h"

#include <iostream>
#include <time.h>
#include "MovementLib.h"
#include <TypeIRML.h>
#include <exception>

#include <boost/thread/thread.hpp>


#ifndef PI
#define PI			3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / PI )
#endif


//using Eigen::Matrix4d;


//*******************************************************************************************
// main()

int main(int argc, char *argv[])
{
	int		i=0
			,	ResultValue
			;
	
    std::cout << "########################################\n"
                 << "              Welcome\n########################################\n"
                 << std::endl;

    FRIBasicDriverCPP * fd = NULL;

	// connect
    try {
        fd = new FRIBasicDriverCPP();
    }
    catch (int e)
    {
        printf("An Exception occured while connecting to the FRIDriver: %d\n", e);
        return 0;
    }
    printf("\n\n");


	// ******************************************************************************************
	// Calling test methods to receive and print the available robot informations
	// remove comments to run
	
	//testGetterCompleteInformation(&fd);
	//testGetterMethods(&fd);

	// set new velocity and acceleration:
//	fd->setMaxAcceleration(90.0);
//	fd->setMaxVelocity(90.0);


#if 1
    // print system status
    //
    printf("Reached Position with Cartesian Velocity Control\n%s\n",fd->GetCompleteRobotStateAndInformation());

    // Check for communication quality and if Robot is reachable
    //
    ResultValue = fd->IsCommunicationOk();
    if(!ResultValue) {
        printf("ERROR: bad communication! FRIMode is turned OFF!");
        return 0;
    } else {
        printf("Robot is turned on and reachable.");
    }
#endif

    MovementLib mv = MovementLib(fd);

#if 0
    // get cart-position and print as e6pos
    //

    float measured_position[12];
    e6pos pose;
    fd->GetMeasuredCartPose(measured_position);
    pose = mv.calcPoseFromFloatArray(measured_position, 12);
    printf("Measured Current Position:\n");
    mv.printE6POS(pose);
    printf("*****************************\n");
#endif


#if 0
	// ******************************************************************************************
    // Move to some position using joint ptp movement
    // STARTING POSITION!!
    //

    printf("Moving to pos2\n");

//	double pos2[] = {90.0, 0.0, 0.0, -90.0, 0.0, 90.0, 0.0};
// some other pose, close to pose2 of the cartesian trajectory tests
//    double pos2[] = {90.0, 35.0, 0.0, -90.0, -0.0, 55.0, 45.0, 0.0};
    double pos2[] = {-78.812, 39.331, 5.999, -84.519, -1.021, 51.064, 32.190};
    fd->mvJoint(pos2);
    fd->Stop();
    printf("Reached Position with Cartesian Velocity Control\n%s\n",fd->GetCompleteRobotStateAndInformation());

#endif


#if 0
    // ******************************************************************************************
    // Move to some position
    // Control Scheme 10 = JOINT_POSITION_CONTROL
    //
    fd->StartInControlScheme(10);

    if(!fd->IsMachineOK())
    {
        printf("Error: machine is not OK!");
    }
    else
    {
        float jointValuesRad[NUMBER_OF_JOINTS];
        fd->getMeasuredJointPositions(jointValuesRad);

        for(int i=0; i<100; ++i)
        {
            jointValuesRad[0] -= 0.015;
            fd->cmdVelJoint(jointValuesRad);
        }
        printf(".\n");
    }
#endif


	
#if 0
    // ******************************************************************************************
    // test very simple Cartesian Velocity Control
    //
    printf("testerMain::\tperform movement with cartesian impedance control\n");
    if(!testCmdVelCart(fd)) {
        printf("testerMain::\tCould not finish movement. Stopping execution.\n");
        fd->Stop();
        return 0;
    }
	
    //printf("Reached Position with Cartesian Velocity Control\n%s\n",fd->GetCompleteRobotStateAndInformation());
#endif

#if 0
    // ******************************************************************************************
    // Move to Test-Position 2 (cartesian velocity control)
    //
    MovementLib mv = MovementLib(fd);

    e6pos p;
    p.x = 0.0;
    p.y = -0.439;
    p.z = 0.311;
    p.a = 91.000;
    p.b = -1.000;
    p.c = 91.000;

    ResultValue = mv.movePTPCart(p, 5);
    if(!ResultValue) {
        printf("\tTestFindHole::runTest01() \tERROR: could not finish movement!\n");
        return false;
    }
    mv.~MovementLib();

#endif

    while(true) {

#if 1
        // ******************************************************************************************
        // move to starting position using cartesian position
        //
        printf("\ttesterMain::\tMoving to starting position");
        e6pos startPose;
        startPose.x = -0.084326;
        startPose.y =  0.601255;
        startPose.z =  0.304220;
        startPose.a = -60.178525;
        startPose.b =   6.514957;
        startPose.c =  88.356525;
        ResultValue = mv.movePTPCart(startPose, 5);
        if(!ResultValue) {
            printf("testerMain::\tCould not finish movement. Stopping execution.\n");
    //        fd->Stop();
            return 0;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
#endif


#if 1
        // ******************************************************************************************
        // tester
        // find a hole using a zig-zag path and test pressure
        //
        printf("testerMain::\ttest ''FindHole functions''\n");
        ResultValue = findHoleTester(fd);
        if(!ResultValue) {
            printf("testerMain::\tCould not finish movement. Stopping execution.\n");
    //        fd->Stop();
            return 0;
        }
        printf("testerMain::\tpausing....\n");
        boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
#endif


#if 1
        // move upwards to get out of critical positions
        //
        e6pos pose2;
        pose2.x =   0.143531;
        pose2.y =   0.657165;
        pose2.z =   0.143250;
        pose2.a = -60.549;
        pose2.b =   0.89;
        pose2.c =  88.0;

    //    MovementLib mv = MovementLib(fd);

        e6pos estimatedCenter = pose2;
        e6pos startPos = estimatedCenter;
        startPos.z = startPos.z + 0.1;
        ResultValue = mv.movePTPCart(startPos, 3);
        if(!ResultValue) {
            printf("\tTestFindHole::runTest01() \tERROR: could not finish movement!\n");
            return false;
        }
        printf("\tTestFindHole::runTest01() \tmoved out of tool-zone.\n");
#endif

    }

#if 0
	// ******************************************************************************************
	// Move back to home position
    //
	// set new velocity and acceleration:
    fd->setMaxAcceleration(50.0);
    fd->setMaxVelocity(50.0);
	
	double homePosition[] = {100.0, 45, 0, -90, 0.0, -45, 0.0};
    printf("\nMoving to home position\n");
    fd->mvJoint(homePosition);
    fd->Stop();
#endif


#if 0
    // ******************************************************************************************
    // loop X seconds and print forces and torques
    //
    printf("testerMain::\tPrint Forces and Torques: \n");
    float forceAndTorqeValues[6];

    for(int i=0;i<turns;i++) {

        getForces(forcesAndTorques);
        printf("\r%d:\t%f\t%f\t%f", i, forcesAndTorques[0], forcesAndTorques[1], forcesAndTorques[2]);
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
#endif


    //printf("%s\n",fd->GetCompleteRobotStateAndInformation());

    mv.~MovementLib();

    fd->Stop();

    fd->~FRIBasicDriverCPP();

    return 0;
}



/*============================================================================================
 * Tester Methods
 *============================================================================================
 */


bool findHoleTester(FRIBasicDriverCPP *_fd)
{
    MovementLib mv = MovementLib(_fd);

    // estimated hole-position
    //
    e6pos pose;
    pose.x =   0.146531;
    pose.y =   0.657165;
    pose.z =   0.143250;
    pose.a = -60.549;
    pose.b =   0.0;
    pose.c =  90.0;

    double errorTolerance   = 1.0,  // cm
           speed            = 1.5,    // cm/s
           angle            = 85;   // degree

    bool ret = mv.findHole(pose, errorTolerance, speed, angle);

    mv.~MovementLib();

    return ret;
}



// tester method for cartesian impedance control
// moving 20cm in one direction
bool testCmdVelCart(FRIBasicDriverCPP *_fd)
{
    // declare variables
    int     i = 0,
            counter = 0;
    bool    endPositionReached  = false,
            resultValue         = true,
            negDist             = false;
    float   MeasuredCartPose[NUMBER_OF_FRAME_ELEMENTS],
            StartPose[NUMBER_OF_FRAME_ELEMENTS],
            EndPose[NUMBER_OF_FRAME_ELEMENTS],
            CommandedPose[NUMBER_OF_FRAME_ELEMENTS],
            tolerance           = 0.01,
            dist                = 0.0,
            xVal                = 0.0;


    printf("NumberOfFrameElements: %d\n", NUMBER_OF_FRAME_ELEMENTS);

    // check and restart robot in proper control scheme
    //
    _fd->StartInControlScheme(20); // 20 == Cartesian Impedance Control

    // get current cartesian position
    //
    _fd->GetMeasuredCartPose(StartPose);
    printFloatArray(StartPose, 12);

    // copy starting-values to end-pose
    //
    for(i=0;i<NUMBER_OF_FRAME_ELEMENTS;i++)
    {
        EndPose[i] = StartPose[i];
        CommandedPose[i] = StartPose[i];
    }

    // set new end-position (only x-value will be changed)
    //
    EndPose[3] = EndPose[3] + 0.2;
    xVal = StartPose[3];

    // loop
    //
    counter = 0;
    while(!endPositionReached)
    {
        // check if machine is ok
        if(!_fd->IsMachineOK())
        {
            printf("\ttestMain::testCmdVelCart() \tMachine is not ready!\n");
            return false;
        }

        // DEBUG: stop after X loops
//        if(counter >= 50) return true;

        // check if final position is reached
        //
        _fd->GetMeasuredCartPose(MeasuredCartPose);

        // range of 0.002 m (2 milimeter)
        tolerance = 0.002;
        dist = EndPose[3] - MeasuredCartPose[3];
        negDist = false;
        if(dist < 0.0) {
            dist*(-1.0);
            negDist = true;
        }
        if((counter%10) == 0) {
            printf("\ttesterMain::testCmdVelCart() \tremaining distance: %f\n", dist);
        }
        if(dist < tolerance)
        {
            printf("\ttesterMain::testCmdVelCart() \tEnd-position reached.\n");
            endPositionReached = true;
            break;
        }

        // add 0.5 milimeter in one direction (e.g. x) until start-position + 20cm is reached
        //
        if(negDist){
            CommandedPose[3] = xVal - (((float) counter / 2.0) / 1000.0);
        } else {
            CommandedPose[3] = xVal + (((float) counter / 2.0) / 1000.0);
        }

        // move
        resultValue = _fd->cmdVelCart(CommandedPose);
        if(!resultValue) {
            printf("\ttesterMain::testCmdVelCart() \tSome unknown error occured, stopped movement.\n");
            return false;
        }

        // increase counter
        counter++;
    }
    return true;
}



// make a Velocity Joint Movement to a set position
void testCmdVelJoint(FRIBasicDriverCPP *fd, double *velPos)
{
	// for velocity control, YOU have to make sure, that the robot is in the correct control scheme and to start the Robot.
    fd->StartInControlScheme(10);

	if(!fd->IsMachineOK())
	{
        printf("Error: machine is not OK!");
	}


	// ********************************************************
	// It depends on you, how you calculate the single steps.
	// in this Tester-Method I'm using the TypeIRML Library for the calculations. Any other calculation will do fine too.

	bool						MachineOK					=	true	;
	int							i							=	0		
							,	ResultValue					=	0		;
	double						CycleTime					=	0.002	;
	float						JointValuesInRad[NUMBER_OF_JOINTS]		;

    TypeIRML					*RML						=	NULL	;
	TypeIRMLInputParameters		*IP							=	NULL	;
	TypeIRMLOutputParameters	*OP							=	NULL	;
	RML					=	new TypeIRML(		NUMBER_OF_JOINTS
											,	CycleTime			);
	IP					=	new TypeIRMLInputParameters(NUMBER_OF_JOINTS);
	OP					=	new TypeIRMLOutputParameters(NUMBER_OF_JOINTS);
	

	memset(JointValuesInRad, 0x0, NUMBER_OF_JOINTS * sizeof(float));


	// print target position
    printf("Target Joint Values: \n");
    for(i=0;i<NUMBER_OF_JOINTS;i++) {printf("     %f", velPos[i]);}
    printf("\n");

	// check if robot is in correct control scheme and OK
	if(fd->getCurrentControlScheme() != 10 || !fd->IsMachineOK()) 
	{
        printf("Robot is not ready for movement.\n");
		delete	RML;
		delete	IP;
		delete	OP;

		return;
	}

	// get current position
	fd->getMeasuredJointPositions(JointValuesInRad);

	// print current position
    printf("Current Joint Positions: \n");
    for(i=0;i<NUMBER_OF_JOINTS;i++) {printf("     %f", JointValuesInRad[i]);}
    printf("\n");

	// set current position as commanded position.

	// setting parameter for TypeIRML calculations
	for ( i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		IP->CurrentPosition->VecData		[i] =	(double)DEG(JointValuesInRad[i]);
		IP->TargetPosition->VecData			[i] =	(double)velPos[i];
		IP->MaxVelocity->VecData			[i] =	(double)50.0;		
		IP->MaxAcceleration->VecData		[i] =	(double)50.0;
		IP->SelectionVector->VecData		[i] =	true;
	}

	ResultValue = TypeIRML::RML_WORKING;

	// starting loop
    printf("Start movement Loop");

	while (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED)
	{
		ResultValue = RML->GetNextMotionState_Position(*IP, OP);

		// check RML-Status
		if((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
		{
			printf("MoveToHome(): ERROR during trajectory generation (%d).", ResultValue);
		}

		for(i=0;i<NUMBER_OF_JOINTS;i++)
		{
			JointValuesInRad[i] = RAD((double)(OP->NewPosition->VecData[i]));
		}

		fd->cmdVelJoint(JointValuesInRad);

		*IP->CurrentPosition = *OP->NewPosition;
		*IP->CurrentVelocity = *OP->NewVelocity;
	}

    printf("New position reached.\n");

	if(!fd->IsMachineOK()) 
	{
		printf("Moving: ERROR, machine is not ready.");
		
		delete	RML;
		delete	IP;
		delete	OP;
		
		return;
	}

	fd->Stop();

	delete RML;
	delete IP;
	delete OP;

	return;
}


void testGetterMethods(FRIBasicDriverCPP *fd)
{
	bool					MachineOK;
	float					JointStiffnessValues[LBR_MNJ]
						,	JointTorques[LBR_MNJ]
						,	JointValuesInRad[NUMBER_OF_JOINTS]
						,	MeasuredPose[NUMBER_OF_FRAME_ELEMENTS]
						,	EstimatedCartForcesAndTorques[NUMBER_OF_JOINTS]
						,	CycleTime = 0;

	// get Current Control Scheme:
    printf("\nCurrent Control Scheme: %d\n\n", fd->getCurrentControlScheme());
    usleep(500);

	// get Measured Joint Positions
	fd->getMeasuredJointPositions(JointValuesInRad);
    printf("\nMeasured Joint Positions (in Radiant): \n");
	printFloatArray(JointValuesInRad, NUMBER_OF_JOINTS);

    usleep(500);

	// getMeasured Joint Torques
	fd->GetMeasuredJointTorques(JointTorques);
    printf("\nMeasured Joint Torques (in Radiant): \n");
	printFloatArray(JointTorques, NUMBER_OF_JOINTS);
    printf("==========================================\n");

    usleep(500);

	// get Measured Cartesian Pose
	fd->GetMeasuredCartPose(MeasuredPose);
    printf("\nMeasured Cartesian Pose:\n");
	printFloatArray(MeasuredPose, NUMBER_OF_FRAME_ELEMENTS);
    printf("==========================================\n");

    usleep(500);

	// get Drive Tempereatures
	fd->GetDriveTemperatures(JointStiffnessValues);
    printf("\nMeasured Drive Temperatures:\n");
	printFloatArray(JointStiffnessValues, NUMBER_OF_JOINTS);
    printf("==========================================\n");

    usleep(500);

	// get Estimated External Cart Forces and Torques
	fd->GetEstimatedExternalCartForcesAndTorques(EstimatedCartForcesAndTorques);
    printf("\nEstimated Cartesian Forces And Torques (in N):\n");
	printFloatArray(EstimatedCartForcesAndTorques, NUMBER_OF_JOINTS);
    printf("==========================================\n");

    usleep(500);

	// Get Estimated External Joint Torques
	fd->GetEstimatedExternalJointTorques(JointTorques);
    printf("\nGet Estimated External Joint Torques: \n");
	printFloatArray(JointTorques, NUMBER_OF_JOINTS);
    printf("==========================================\n");

    usleep(500);

	// Get FRI Cycle Time
	CycleTime = fd->GetFRICycleTime();
    printf("\nRobot Cycle Time: %f\n", CycleTime);
    printf("==========================================\n");

    usleep(500);

	// is Machine OK
	MachineOK = fd->IsMachineOK();
    printf("\nCheck if Machine is OK: \n");
	if(MachineOK) {
        printf("Yes, Machine is OK.\n");
	} else {
        printf("NO, Machine is NOT OK!\n");
	}
    printf("==========================================\n");

	return;
}

void testGetterCompleteInformation(FRIBasicDriverCPP *fd) 
{
    printf("Complete Robot and State information:\n%s\n\n", fd->GetCompleteRobotStateAndInformation());
}

void printFloatArray(float *ar, int size)
{
	for(int i=0; i<size; i++) {
        printf("%f    ", (float) ar[i]);
	}
    printf("\n");
}


void printDoubleArray(double *ar, int size)
{
	for(int i=0; i<size; i++) {
        printf("%f    ", (double)ar[i]);
	}
    printf("\n");
}
