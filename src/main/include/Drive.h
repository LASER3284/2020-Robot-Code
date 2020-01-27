/****************************************************************************
	Description:	Defines the CDrive control class.
	Classes:		CDrive
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Drive_h
#define Drive_h

#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/geometry/Pose2d.h>
#include <AHRS.h>
#include "FalconMotion.h"

using namespace frc;
using namespace std;
using namespace units;

// Default constants for the CDrive class.
const auto 		m_dDefaultkS 							= 0.367_V;											//	|
const auto 		m_dDefaultkV 							= 0.0568 * 1_V * 1_s / 1_in;						//	|	Drive characterization constants.
const auto 		m_dDefaultkA 							= 0.00672 * 1_V * 1_s * 1_s / 1_in;					//	|
const double    m_dDefaultBeta                         	= 0.800;	// 2.0
const double    m_dDefaultZeta                         	= 0.350;	// 0.7
const double	m_dDefaultProportional					= 0.308;	// Left drive proportional value. // 0.000179
const double	m_dDefaultIntegral						= 0.000;	// Left drive integral value.
const double 	m_dDefaultDerivative					= 0.000;	// Left drive derivative value.
const auto    	m_dDefaultDrivebaseWidth               	= 22.657;
/////////////////////////////////////////////////////////////////////////////
class CDrive
{
public:
    CDrive(Joystick *pDriveController);
    ~CDrive();
    void Init();
    void Tick();
    void GenerateTragectory();
    void FollowTragectory();
	void SetDrivePowers(volt_t dLeftVoltage, volt_t dRightVoltage);
	DifferentialDriveWheelSpeeds GetWheelSpeeds();
    void ResetOdometry();
	void ResetEncoders();
    void Stop();

private:
    // Object Pointers.
    Joystick*                       m_pDriveController;
    CFalconMotion*                  m_pLeftMotor1;
    WPI_TalonFX*                    m_pLeftMotor2;
    CFalconMotion*                  m_pRightMotor1;
    WPI_TalonFX*                    m_pRightMotor2;
	DifferentialDrive*              m_pRobotDrive;
	AHRS*                           m_pGyro;
    Timer*                          m_pTimer;
    DifferentialDriveOdometry*      m_pOdometry;
	frc2::RamseteCommand*			m_pRamseteCommand;
	Trajectory              		m_Trajectory;

    // Member variables.
    bool                    m_bMotionProfile;
    double                  m_dBeta;
    double                  m_dZeta;
	double					m_dProportional;
	double					m_dIntegral;
	double					m_dDerivative;
	double					m_dDriveBaseWidth;

    // Start and End poses for robot field position.
	const Pose2d m_StartPoint
	{
		0.0_ft,					// X starting position on field in feet.
		0.0_ft,					// Y starting position on field in feet.
		Rotation2d(0_deg)		// Starting rotation on field in degrees.
	};
	const Pose2d m_EndPoint
	{
		14.0_ft,				// X ending position on field in feet.
		7.0_ft,					// Y ending position on field in feet.
		Rotation2d(105_deg)		// Ending rotation on field in degrees.
	};

    // Interior waypoints for trajectory. (The current waypoints make the robot follow a 2 curve snake path.)
	vector<Translation2d> m_InteriorWaypoints
	{
		Translation2d
		{
			6.0_ft,					// X of point 1 on field in feet.
			7.5_ft					// Y of point 1 on field in feet.
		},
		Translation2d
		{
			11.0_ft,				// X of point 2 on field in feet.
			2.5_ft					// Y of point 2 on field in feet.
		},
	};

	// Configure trajectory properties.
	TrajectoryConfig m_Config
	{
		3_fps,					// Max desirable velocity.
		1_fps_sq				// Max desirable acceleration.
	};
};
/////////////////////////////////////////////////////////////////////////////
#endif