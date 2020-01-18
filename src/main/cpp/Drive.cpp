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
using namespace units;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
	Description:	CDrive Constructor.
	Arguments:		Joystick* pDriveController
	Derived From:	Nothing
****************************************************************************/
CDrive::CDrive(Joystick* pDriveController)
{
	// Initialize member variables.
	m_bMotionProfile		= false;
	m_dBeta					= m_dDefaultBeta;
	m_dZeta					= m_dDefaultZeta;
	m_dDrivebaseWidth		= m_dDefaultDrivebaseWidth;

	// Create object pointers.
	m_pDriveController 		= pDriveController;
	m_pLeftMotor1			= new CFalconMotion(8);
	m_pLeftMotor2			= new WPI_TalonFX(9);
	m_pRightMotor1			= new CFalconMotion(10);
	m_pRightMotor2			= new WPI_TalonFX(11);
	m_pRobotDrive			= new DifferentialDrive(*m_pLeftMotor1->GetMotorPointer(), *m_pRightMotor1->GetMotorPointer());
	m_pKinematics			= new DifferentialDriveKinematics(inch_t(m_dDrivebaseWidth));
	m_pRamseteController	= new RamseteController(m_dBeta, m_dZeta);
	m_pGyro 				= new AHRS(SPI::Port::kMXP);
	m_pTimer				= new Timer();
	m_pOdometry 			= new DifferentialDriveOdometry(Rotation2d(degree_t(m_pGyro->GetYaw())), m_StartPoint);
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

	// Reset motor encoders.
	m_pLeftMotor1->ResetEncoderPosition();
	m_pRightMotor1->ResetEncoderPosition();

	// Reset robot field odometry.
	ResetOdometry();

	// Start the timer.
	m_pTimer->Start();

	// Reset gyro.
	m_pGyro->Reset();
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
	double YAxis = m_pDriveController->GetRawAxis(1);

	// Check if joystick is in deadzone.
	if (fabs(XAxis) < 0.1)
	{
		XAxis = 0;
	}
	if (fabs(YAxis) < 0.1)
	{
		YAxis = 0;
	}

	// Update odometry. (Position on field.)
	m_pOdometry->Update(Rotation2d(degree_t(m_pGyro->GetYaw())), inch_t(m_pLeftMotor1->GetActual(true)), inch_t(m_pRightMotor1->GetActual(true)));

	// Drive the robot.
	if (!m_pDriveController->GetRawButton(1))
	{
		// Set drivetrain powers.
		m_pRobotDrive->ArcadeDrive(YAxis, -XAxis, false);

		// Stop motors if we were previously following a path and reset trajectory.
		if (m_bMotionProfile)
		{
			Stop();
			ResetOdometry();
			m_bMotionProfile = false;
		}
	}
	else
	{
		// Get the current time.
		if (!m_bMotionProfile)
		{
			m_dPathFollowStartTime = m_pTimer->Get();
		}

		// Set that we are currently following a path.
		m_bMotionProfile = true;

		// Follow the pre-generated path.
		FollowTragectory();
	}

	// Update Smartdashboard values.
	SmartDashboard::PutNumber("Left Actual Velocity", m_pLeftMotor1->GetActual(false));
	SmartDashboard::PutNumber("Right Actual Velocity", m_pRightMotor1->GetActual(false));
	SmartDashboard::PutNumber("Left Actual Position", m_pLeftMotor1->GetActual(true));
	SmartDashboard::PutNumber("Right Actual Position", m_pRightMotor1->GetActual(true));
}

/****************************************************************************
	Description:	MotionProfiling method that generates a new trajectory.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::GenerateTragectory()
{
	// Generate the trajectory.
	m_Trajectory = TrajectoryGenerator::GenerateTrajectory(m_StartPoint, m_InteriorWaypoints, m_EndPoint, m_Config);
}

/****************************************************************************
	Description:	MotionProfiling method that follows a pre-generated trajectory.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::FollowTragectory()
{
	// Calculate elapsed time.
	double dElapsedTime = (m_pTimer->Get() - m_dPathFollowStartTime);

	// Sample the trajectory at .06 seconds from the last point.
	const auto m_Goal = m_Trajectory.Sample(second_t(dElapsedTime));
	// Calculate the wheel velocity for the next point in the trajectory path.
	ChassisSpeeds m_pAdjustedSpeeds = m_pRamseteController->Calculate(m_pOdometry->GetPose(), m_Goal);
	// Convert to values we can use for Differential Drive.
	DifferentialDriveWheelSpeeds m_pDriveSpeeds = m_pKinematics->ToWheelSpeeds(m_pAdjustedSpeeds);

	// Disable motor safety.
	m_pRobotDrive->SetSafetyEnabled(false);

	// Set motor powers.
	m_pLeftMotor1->SetSetpoint(double(m_pDriveSpeeds.left), false);
	m_pRightMotor1->SetSetpoint(double(-m_pDriveSpeeds.right), false);

	// Call motor ticks.
	m_pLeftMotor1->Tick();
	m_pRightMotor1->Tick();

	// Put motor powers on dashboard.
	SmartDashboard::PutNumber("Elapsed Time", dElapsedTime);
	SmartDashboard::PutNumber("LeftMotorPower", double(m_pDriveSpeeds.left));
	SmartDashboard::PutNumber("RightMotorPower", double(m_pDriveSpeeds.right));
}

/****************************************************************************
	Description:	Method that resets encoders and odometry.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::ResetOdometry()
{
	// Reset drive encoders.
	m_pLeftMotor1->ResetEncoderPosition();
	m_pRightMotor1->ResetEncoderPosition();

	// Reset field position.
	m_pOdometry->ResetPosition(m_StartPoint, Rotation2d(degree_t(m_pGyro->GetYaw())));
}

/****************************************************************************
	Description:	Method the stops drive motors.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::Stop()
{
	// Stop both drive motors.
	m_pLeftMotor1->Stop();
	m_pRightMotor1->Stop();
}