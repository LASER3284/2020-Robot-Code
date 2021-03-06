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

    // Create object pointers.
    m_pDriveController 		= pDriveController;
    m_pLeftMotor1			= new CFalconMotion(nLeftDriveMotor1);
    m_pLeftMotor2			= new WPI_TalonFX(nLeftDriveMotor2);
    m_pRightMotor1			= new CFalconMotion(nRightDriveMotor1);
    m_pRightMotor2			= new WPI_TalonFX(nRightDriveMotor2);
    m_pRobotDrive			= new DifferentialDrive(*m_pLeftMotor1->GetMotorPointer(), *m_pRightMotor1->GetMotorPointer());
    m_pGyro 				= new AHRS(SPI::Port::kMXP);
    m_pTimer				= new Timer();
    m_pOdometry 			= new DifferentialDriveOdometry(Rotation2d(degree_t(-m_pGyro->GetYaw())), m_pTrajectoryConstants.GetSelectedTrajectoryStartPoint());	
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

        // Set drivetrain powers to joystick controls.
        m_pRobotDrive->ArcadeDrive(YAxis, XAxis, false);
    }

    // Update odometry. (Position on field.)
    m_pOdometry->Update(Rotation2d(degree_t(-m_pGyro->GetYaw())), inch_t(m_pLeftMotor1->GetActual(true)), inch_t(m_pRightMotor1->GetActual(true)));

    // Update Smartdashboard values.
    SmartDashboard::PutNumber("LeftMotorPower", m_pLeftMotor1->GetMotorVoltage());
    SmartDashboard::PutNumber("RightMotorPower", m_pRightMotor1->GetMotorVoltage());
    SmartDashboard::PutNumber("Left Actual Velocity", (m_pLeftMotor1->GetActual(false) / 39.3701));
    SmartDashboard::PutNumber("Right Actual Velocity", (m_pRightMotor1->GetActual(false) / 39.3701));
    SmartDashboard::PutNumber("Left Actual Position", m_pLeftMotor1->GetActual(true));
    SmartDashboard::PutNumber("Right Actual Position", m_pRightMotor1->GetActual(true));
    SmartDashboard::PutNumber("Odometry Field Position X", double(inch_t(m_pOdometry->GetPose().Translation().X())));
    SmartDashboard::PutNumber("Odometry Field Position Y", double(inch_t(m_pOdometry->GetPose().Translation().Y())));
    SmartDashboard::PutNumber("Odometry Field Position Rotation", double(m_pOdometry->GetPose().Rotation().Degrees()));
    SmartDashboard::PutNumber("Total Trajectory Time", double(m_Trajectory.TotalTime()));
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
    Description:	GenerateTrajectory - MotionProfiling method that 
                    generates a new trajectory.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CDrive::GenerateTrajectory(vector<Pose2d> pWaypoints, meters_per_second_t MaxSpeed, meters_per_second_squared_t MaxAcceleration)
{
    // Create voltage constraint to make sure we don't accelerate too fast.
    auto m_VoltageConstraint = DifferentialDriveVoltageConstraint(SimpleMotorFeedforward<units::meters>(m_kS, m_kV, m_kA), m_kDriveKinematics, 10_V);

    // Create the trajectory config.
    auto m_Config = TrajectoryConfig(MaxSpeed, MaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed.
    m_Config.SetKinematics(m_kDriveKinematics);
    // Apply the voltage constraint.
    m_Config.AddConstraint(m_VoltageConstraint);

    // Set trajectory parameters.
    if (m_pTrajectoryConstants.GetIsTrajectoryReversed())
    {
        m_Config.SetReversed(true);
    }
    else
    {
        m_Config.SetReversed(false);
    }

    // Generate the trajectory.
    m_Trajectory = TrajectoryGenerator::GenerateTrajectory(pWaypoints, m_Config);
    
    // Setup the RamseteCommand with new trajectory.
    m_pRamseteCommand = new frc2::RamseteCommand(
        m_Trajectory, 
        [this]() { return m_pOdometry->GetPose(); }, 
        RamseteController(m_dBeta, m_dZeta), 
        SimpleMotorFeedforward<units::meters>(m_kS, m_kV, m_kA), 
        m_kDriveKinematics, 
        [this]() { return GetWheelSpeeds(); }, 
        frc2::PIDController(m_dProportional, m_dIntegral, m_dDerivative), 
        frc2::PIDController(m_dProportional, m_dIntegral, m_dDerivative), 
        [this](auto left, auto right) { SetDrivePowers(left, right); }
    );

    // Go RamseteCommand!
    m_pRamseteCommand->Schedule();
}

/****************************************************************************
    Description:	FollowTrajectory - MotionProfiling method that follows
                    a pre-generated trajectory.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CDrive::FollowTrajectory()
{
    // Disable motor safety.
    m_pRobotDrive->SetSafetyEnabled(false);

    // Update RamseteCommand.
    m_pRamseteCommand->Execute();
}

/****************************************************************************
    Description:	SetDrivePowers - Method that sets the left and 
                    right drivetrain voltages.
    Arguments: 		dLeftVoltage - Left motor voltage.
                    dRightVoltage - Right motor voltage.
    Returns: 		Nothing
****************************************************************************/
void CDrive::SetDrivePowers(volt_t dLeftVoltage, volt_t dRightVoltage)
{
    // Set drivetrain powers.
    m_pLeftMotor1->SetMotorVoltage(double(dLeftVoltage));
    m_pRightMotor1->SetMotorVoltage(double(dRightVoltage));
}

/****************************************************************************
    Description:	DriveToPresetPosition - Drive back to a preset position 
                                        by creating and following a trajectory 
                                        based on our current position.
    Arguments: 		vector<Pose2d> pWaypoints
    Returns: 	    Nothing
****************************************************************************/
void CDrive::GeneratePathFromCurrentPosition()
{
    // Create waypoints vector.
    vector<Pose2d> pWaypoints;

    // Build waypoints.
    pWaypoints.emplace_back(GetRobotPose());
    pWaypoints.emplace_back(m_pTrajectoryConstants.PresetPoint1);
    
    // Pass built waypoints vector to the drive class for trajectory generation.
    GenerateTrajectory(pWaypoints, m_pTrajectoryConstants.kMaxSpeed, m_pTrajectoryConstants.kMaxAcceleration);
}

/****************************************************************************
    Description:	GetWheelSpeeds - Method that gets the wheel velocity 
                    in meters per second.
    Arguments: 		None
    Returns: 		DifferentialDriveWheelSpeeds - Drivetrain speeds.
****************************************************************************/
DifferentialDriveWheelSpeeds CDrive::GetWheelSpeeds()
{
    // Return wheel speeds.
    return {meters_per_second_t(m_pLeftMotor1->GetActual(false) / 39.3701), meters_per_second_t(m_pRightMotor1->GetActual(false) / 39.3701)};
}

/****************************************************************************
    Description:	ResetOdometry - Method that resets encoders and odometry.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CDrive::ResetOdometry()
{
    // Reset drive encoders.
    ResetEncoders();

    // Reset Gyro.
    m_pGyro->ZeroYaw();

    // Reset field position.
    m_pOdometry->ResetPosition(m_pTrajectoryConstants.GetSelectedTrajectoryStartPoint(), Rotation2d(degree_t(-m_pGyro->GetYaw())));
}

/****************************************************************************
    Description:	ResetEncoders - Method that reset encoder values back 
                    to zero.
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
    Description:	Stop - Method that stops drive motors.
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
    Description:	GetIsTrajectoryFinished - Returns of the generated
                    trajectory has successfully reached its last point.
    Arguments: 		None
    Returns: 		bool bTrajectoryIsFinished
****************************************************************************/
bool CDrive::TrajectoryIsFinished()
{
    return m_pRamseteCommand->IsFinished();
}

/****************************************************************************
    Description:	GetTotalTrajectoryTime - Returns the trajectory time.
    Arguments: 		None
    Returns: 		INT nTotalTime
****************************************************************************/
int CDrive::GetTotalTrajectoryTime()
{   
    return int(m_Trajectory.TotalTime());
}

/****************************************************************************
    Description:	GetRobotPose - Return the Pose2d of the robot from the
                                odometry class.
    Arguments: 		None
    Returns: 		POSE2D m_pRobotPose
****************************************************************************/
Pose2d CDrive::GetRobotPose()
{
    return m_pOdometry->GetPose();
}

/****************************************************************************
    Description:	SetSelectedTrajectory - Select trajectory for auto.
    Arguments: 		int nAutoState - The auto state.
    Returns: 		Nothing
****************************************************************************/
void CDrive::SetSelectedTrajectory(int nAutoState)
{
    m_pTrajectoryConstants.SelectTrajectory(nAutoState);
    GenerateTrajectory(m_pTrajectoryConstants.GetSelectedTrajectory(), m_pTrajectoryConstants.kMaxSpeed, m_pTrajectoryConstants.kMaxAcceleration);
    ResetOdometry();
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