/****************************************************************************
	Description:	Implements the CDrive control class.
	Classes:		CDrive
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include <frc/Timer.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/geometry/Translation2d.h>
#include "Drive.h"
#include "IOMap.h"

using namespace ctre;
using namespace std;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
	Description:	CDrive Constructor.
	Arguments:		Joystick* pDriveController
	Derived From:	Nothing
****************************************************************************/
CDrive::CDrive(Joystick* pDriveController)
{
	// Initialize member variables.
	m_dBeta					= m_dDefaultBeta;
	m_dZeta					= m_dDefaultZeta;

	// Create object pointers.
	m_pDriveController 		= pDriveController;
	m_pLeftMotor1			= new CFalconMotion(1);
	m_pLeftMotor2			= new WPI_TalonFX(2);
	m_pRightMotor1			= new CFalconMotion(3);
	m_pRightMotor2			= new WPI_TalonFX(4);
	m_pRobotDrive			= new DifferentialDrive(*m_pLeftMotor1->GetMotorPointer(), *m_pRightMotor1->GetMotorPointer());
	m_pRamseteController	= new RamseteController(m_dBeta, m_dZeta);
	m_pGyro 				= new AHRS(SPI::Port::kMXP);
}

/****************************************************************************
	Description:	CDrive Destructor.
	Arguments:		None
	Derived From:	Nothing
****************************************************************************/
CDrive::~CDrive()
{
	delete m_pLeftMotor1;
	delete m_pLeftMotor2;
	delete m_pRightMotor1;
	delete m_pRightMotor2;
	delete m_pRobotDrive;
	delete m_pRamseteController;
	delete m_pGyro;

	m_pLeftMotor1			= nullptr;
	m_pLeftMotor2			= nullptr;
	m_pRightMotor1			= nullptr;
	m_pRightMotor2			= nullptr;
	m_pRobotDrive			= nullptr;
	m_pRamseteController 	= nullptr;
	m_pGyro					= nullptr;
}

/****************************************************************************
	Description:	Initialize drive parameters.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::Init()
{
	// Make motor 2 follow motor 1 on both sides.
	m_pLeftMotor2->Follow(*m_pLeftMotor1->GetMotorPointer());
	m_pRightMotor2->Follow(*m_pRightMotor1->GetMotorPointer());

	// Set max acceleration to .65 seconds.
	m_pLeftMotor1->SetOpenLoopRampRate(.65);
	m_pRightMotor1->SetOpenLoopRampRate(.65);

	// Clear persistant motor controller faults.
	m_pLeftMotor1->ClearStickyFaults();
	m_pRightMotor1->ClearStickyFaults();
}

/****************************************************************************
	Description:	Main method that calls functionality, to be used in a loop.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::Tick()
{
	// Set variables to joysticks.
	double XAxis = m_pDriveController->GetRawAxis(4);
	double YAxis = -m_pDriveController->GetRawAxis(2);

	// Check if joystick is in deadzone.
	if (fabs(XAxis) < 0.1)
	{
		XAxis = 0;
	}
	if (fabs(YAxis) < 0.1)
	{
		YAxis = 0;
	}

	// Drive the robot.
	m_pRobotDrive->ArcadeDrive(YAxis, XAxis, false);
}

/****************************************************************************
	Description:	MotionProfiling method that generates a new trajectory.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::GenerateTragectory()
{
	// Create start and end poses for robot.
	const Pose2d m_pStartPoint
	{
		1.5_ft,					// X starting position on field in feet.
		24.0_ft,				// Y starting position on field in feet.
		Rotation2d(0_deg)		// Starting rotation on field in degrees.
	};
	const Pose2d m_pEndPoint
	{
		10.0_ft,				// X ending position on field in feet.
		24.0_ft,				// Y ending position on field in feet.
		Rotation2d(0_deg)		// Ending rotation on field in degrees.
	};

	// Create interior waypoints for trajectory. (The current waypoints make the robot follow a 2 curve snake path.)
	vector<Translation2d> m_pInteriorWaypoints
	{
		Translation2d
		{
			4.0_ft,				// X of point 2 on field in feet.
			23.0_ft				// Y of point 2 on field in feet.
		},
		Translation2d
		{
			6.0_ft,				// X of point 3 on field in feet.
			25.0_ft				// Y of point 3 on field in feet.
		},
		Translation2d
		{
			8.0_ft,				// X of point 4 on field in feet.
			24.0_ft				// Y of point 4 on field in feet.
		}
	};

	// Configure trajectory properties.
	TrajectoryConfig m_pConfig
	{
		12_fps,					// Robot max velocity I think. (Or whatever max velocity or acceleration you wish to put.)
		12_fps_sq				// Robot max acceleration I think.
	};

	// Generate the trajectory.
	m_Trajectory = TrajectoryGenerator::GenerateTrajectory(m_pStartPoint, m_pInteriorWaypoints, m_pEndPoint, m_pConfig);
}

/****************************************************************************
	Description:	MotionProfiling method that follows a pre-generated trajectory.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::FollowTragectory()
{
	
}