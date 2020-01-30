/****************************************************************************
	Description:	Implements the CDrive control class.
	Classes:		CDrive
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include <frc/Timer.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/geometry/Translation2d.h>
#include <frc/controller/SimpleMotorFeedforward.h>
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
	m_bJoystickControl		= false;
	m_bMotionProfile		= false;
	m_dBeta					= m_dDefaultBeta;
	m_dZeta					= m_dDefaultZeta;
	m_dProportional			= m_dDefaultProportional;
	m_dIntegral				= m_dDefaultIntegral;
	m_dDerivative			= m_dDefaultDerivative;
	m_dDriveBaseWidth		= m_dDefaultDrivebaseWidth;

	// Create object pointers.
	m_pDriveController 		= pDriveController;
	m_pLeftMotor1			= new CFalconMotion(10);
	m_pLeftMotor2			= new WPI_TalonFX(11);
	m_pRightMotor1			= new CFalconMotion(8);
	m_pRightMotor2			= new WPI_TalonFX(9);
	m_pRobotDrive			= new DifferentialDrive(*m_pLeftMotor1->GetMotorPointer(), *m_pRightMotor1->GetMotorPointer());
	m_pGyro 				= new AHRS(SPI::Port::kMXP);
	m_pTimer				= new Timer();
	m_pOdometry 			= new DifferentialDriveOdometry(Rotation2d(degree_t(-m_pGyro->GetYaw())), m_pTrajectoryConstants.m_StartPoint);	
}

/****************************************************************************
	Description:	CDrive Destructor.
	Arguments:		None
	Derived From:	Nothing
****************************************************************************/
CDrive::~CDrive()
{
	// Delete objects.
	delete m_pLeftMotor1;
	delete m_pLeftMotor2;
	delete m_pRightMotor1;
	delete m_pRightMotor2;
	delete m_pRobotDrive;
	delete m_pGyro;
	delete m_pTimer;

	m_pLeftMotor1			= nullptr;
	m_pLeftMotor2			= nullptr;
	m_pRightMotor1			= nullptr;
	m_pRightMotor2			= nullptr;
	m_pRobotDrive			= nullptr;
	m_pGyro					= nullptr;
	m_pTimer				= nullptr;
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

	// Invert the left side.
	m_pLeftMotor1->SetMotorInverted(true);
	m_pLeftMotor2->SetInverted(true);

	// Reset motor encoders.
	m_pLeftMotor1->ResetEncoderPosition();
	m_pRightMotor1->ResetEncoderPosition();

	// Don't let the DifferentialDrive Invert anything.
	m_pRobotDrive->SetRightSideInverted(false);

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
  	if (m_bJoystickControl)
	{
		// Set variables to joystick values.
		double XAxis = m_pDriveController->GetRawAxis(eRightAxisX);
		double YAxis = -m_pDriveController->GetRawAxis(eLeftAxisY);

		// Check if joystick is in deadzone.
		if (fabs(XAxis) < 0.1)
		{
			XAxis = 0.0;
		}
		if (fabs(YAxis) < 0.1)
		{
			YAxis = 0.0;
		}

    	// Update odometry. (Position on field.)
    m_pOdometry->Update(Rotation2d(degree_t(-m_pGyro->GetYaw())), inch_t(m_pLeftMotor1->GetActual(true)), inch_t(m_pRightMotor1->GetActual(true)));

    // Drive the robot.
    if (!m_pDriveController->GetRawButton(1))
    {
      // Set drivetrain powers to joystick controls.
      m_pRobotDrive->ArcadeDrive(YAxis, XAxis, false);

      // Stop motors if we were previously following a path and reset trajectory.
      if (m_bMotionProfile)
      {
        Stop();
        m_pRamseteCommand->Cancel();
        m_pRobotDrive->SetSafetyEnabled(true);
        m_bMotionProfile = false;
      }
    }
    else
    {
      // If X is pressed regenerate path.
      if (m_pDriveController->GetRawButton(3))
      {
        GenerateTrajectory(m_pTrajectoryConstants.m_InteriorWaypoints, m_pTrajectoryConstants.m_Config);
      }

      // Reset robot values.
      if (!m_bMotionProfile)
      {
        // Reset robot field position and encoders.
        ResetOdometry();
      }

      // Set that we are currently following a path.
      m_bMotionProfile = true;

      // Follow the pre-generated path.
      FollowTrajectory();
    }

    // Update Smartdashboard values.
    SmartDashboard::PutNumber("Left Actual Velocity", (m_pLeftMotor1->GetActual(false)));
    SmartDashboard::PutNumber("Right Actual Velocity", (m_pRightMotor1->GetActual(false)));
    SmartDashboard::PutNumber("Left Actual Position", m_pLeftMotor1->GetActual(true));
    SmartDashboard::PutNumber("Right Actual Position", m_pRightMotor1->GetActual(true));
    SmartDashboard::PutNumber("Odometry Field Position X", double(inch_t(m_pOdometry->GetPose().Translation().X())));
    SmartDashboard::PutNumber("Odometry Field Position Y", double(inch_t(m_pOdometry->GetPose().Translation().Y())));
    SmartDashboard::PutNumber("Odometry Field Position Rotation", double(m_pOdometry->GetPose().Rotation().Degrees()));	

		// Drive the robot.
		m_pRobotDrive->ArcadeDrive(YAxis, XAxis, false);
	}
}

/****************************************************************************
	Description:	SetJoystickControl - Sets the desired joystick control.
	Arguments: 		bool bJoystickControl - True if joystick control enabled,
					false otherwise.
	Returns: 		Nothing
****************************************************************************/
void CDrive::SetJoystickControl(bool bJoystickControl)
{
	m_bJoystickControl = bJoystickControl;
}

/****************************************************************************
	Description:	MotionProfiling method that generates a new trajectory.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::GenerateTrajectory(vector<Pose2d> pWaypoints, TrajectoryConfig pConfig)
{
	// Generate the trajectory.
	m_Trajectory = TrajectoryGenerator::GenerateTrajectory(pWaypoints, pConfig);
	

	  // Setup the RamseteCommand with new trajectory.
	  m_pRamseteCommand = new frc2::RamseteCommand(
		m_Trajectory, 
		[this]() { return m_pOdometry->GetPose(); }, 
		RamseteController(m_dBeta, m_dZeta), 
		SimpleMotorFeedforward<units::meters>(m_dDefaultkS, m_dDefaultkV, m_dDefaultkA), 
		DifferentialDriveKinematics(inch_t(m_dDriveBaseWidth)), 
		[this]() { return GetWheelSpeeds(); }, 
		frc2::PIDController(m_dProportional, m_dIntegral, m_dDerivative), 
		frc2::PIDController(m_dProportional, m_dIntegral, m_dDerivative), 
		[this](auto left, auto right) { SetDrivePowers(left, right); }
	);

	// Go RamseteCommand!
	m_pRamseteCommand->Initialize();
	m_pRamseteCommand->Schedule();
}

/****************************************************************************
	Description:	MotionProfiling method that follows a pre-generated trajectory.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::FollowTrajectory()
{
	// Disable motor safety.
	m_pRobotDrive->SetSafetyEnabled(false);

	// Update RamseteCommand.
	m_pRamseteCommand->Execute();

	// Put trajectory info on SmartDashboard.
	SmartDashboard::PutNumber("Total Trajectory Time", double(m_Trajectory.TotalTime()));
}

/****************************************************************************
	Description:	Method that sets the left and right drivetrain voltages.
	Arguments: 		dLeftVoltage - Left motor voltage.
					dRightVoltage - Right motor voltage.
	Returns: 		Nothing
****************************************************************************/
void CDrive::SetDrivePowers(volt_t dLeftVoltage, volt_t dRightVoltage)
{
	// Set drivetrain powers.
	m_pLeftMotor1->SetMotorVoltage(double(dLeftVoltage));
	m_pRightMotor1->SetMotorVoltage(double(dRightVoltage));

	// Put drive powers on SmartDashboard.
	SmartDashboard::PutNumber("LeftMotorPower", m_pLeftMotor1->GetMotorVoltage());
	SmartDashboard::PutNumber("RightMotorPower", m_pRightMotor1->GetMotorVoltage());
}

/****************************************************************************
	Description:	Method that gets the wheel velocity in meters per second.
	Arguments: 		None
	Returns: 		DifferentialDriveWheelSpeeds - Drivetrain speeds.
****************************************************************************/
DifferentialDriveWheelSpeeds CDrive::GetWheelSpeeds()
{
	// Put wheel speeds on SmartDashboard.
	SmartDashboard::PutNumber("LeftWheelSpeed", m_pLeftMotor1->GetActual(false) / 39.3701);
	SmartDashboard::PutNumber("RightWheelSpeed", m_pRightMotor1->GetActual(false) / 39.3701);
	
	// Return wheel speeds.
	return {meters_per_second_t(m_pLeftMotor1->GetActual(false) / 39.3701), meters_per_second_t(m_pRightMotor1->GetActual(false) / 39.3701)};
}

/****************************************************************************
	Description:	Method that resets encoders and odometry.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::ResetOdometry()
{
	// Reset drive encoders.
	ResetEncoders();

	// Reset field position.
	m_pOdometry->ResetPosition(m_pTrajectoryConstants.m_StartPoint, Rotation2d(degree_t(-m_pGyro->GetYaw())));
}

/****************************************************************************
	Description:	Method that reset encoder values back to zero.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::ResetEncoders()
{
	// Reset drive encoders.
	m_pLeftMotor1->ResetEncoderPosition();
	m_pRightMotor1->ResetEncoderPosition();
}

/****************************************************************************
	Description:	Method that stops drive motors.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::Stop()
{
	// Stop both drive motors.
	m_pLeftMotor1->Stop();
	m_pRightMotor1->Stop();
}

/****************************************************************************
	Description:	SetMotorExpiration - Sets the motor safety expiration
					timeout.
	Arguments: 		double dTimeout - Expiration timeout
	Returns: 		Nothing
****************************************************************************/
void CDrive::SetMotorExpiration(double dTimeout)
{
	m_pRobotDrive->SetExpiration(dTimeout);
}

/****************************************************************************
	Description:	SetMotorSafety - Sets the motor safety enabled for the
					drive motors.
	Arguments: 		bool bEnabled - True to set MotorSafetyEnabled
	Returns: 		Nothing
****************************************************************************/
void CDrive::SetMotorSafety(bool bEnabled)
{
	m_pRobotDrive->SetSafetyEnabled(bEnabled);
}
///////////////////////////////////////////////////////////////////////////////