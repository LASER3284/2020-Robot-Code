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
#include <frc/controller/RamseteController.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/geometry/Pose2d.h>
#include <AHRS.h>
#include "FalconMotion.h"

using namespace frc;
using namespace std;

// Default constants for the CDrive class.
const double    m_dDefaultBeta                         = 0.228;	// 2.0
const double    m_dDefaultZeta                         = 0.7;	// 0.7
const double    m_dDefaultDrivebaseWidth               = 25.0;
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
    void ResetOdometry();
    void Stop();

private:
    // Object Pointers.
    Joystick*                       m_pDriveController;
    CFalconMotion*                  m_pLeftMotor1;
    WPI_TalonFX*                    m_pLeftMotor2;
    CFalconMotion*                  m_pRightMotor1;
    WPI_TalonFX*                    m_pRightMotor2;
    DifferentialDrive*              m_pRobotDrive;
    DifferentialDriveOdometry*      m_pOdometry;
    DifferentialDriveKinematics*    m_pKinematics;
    RamseteController*              m_pRamseteController;
    AHRS*                           m_pGyro;
    Timer*                          m_pTimer;
	Trajectory              		m_Trajectory;


    // Member variables.
    bool                    m_bMotionProfile;
    double                  m_dBeta;
    double                  m_dZeta;
    double                  m_dDrivebaseWidth;
    double                  m_dPathFollowStartTime;

    // Start and End poses for robot field position.
	const Pose2d m_StartPoint
	{
		5.0_ft,					// X starting position on field in feet.
		12.0_ft,				// Y starting position on field in feet.
		Rotation2d(0_deg)		// Starting rotation on field in degrees.
	};
	const Pose2d m_EndPoint
	{
		15.0_ft,				// X ending position on field in feet.
		12.0_ft,				// Y ending position on field in feet.
		Rotation2d(0_deg)		// Ending rotation on field in degrees.
	};

    // Interior waypoints for trajectory. (The current waypoints make the robot follow a 2 curve snake path.)
	vector<Translation2d> m_InteriorWaypoints
	{
		Translation2d
		{
			10.0_ft,				// X of point 2 on field in feet.
			12.0_ft					// Y of point 2 on field in feet.
		},
		// Translation2d
		// {
		// 	18.0_ft,				// X of point 3 on field in feet.
		// 	30.0_ft					// Y of point 3 on field in feet.
		// },
		// Translation2d
		// {
		// 	25.0_ft,				// X of point 4 on field in feet.
		// 	24.0_ft					// Y of point 4 on field in feet.
		// }
	};

	// Configure trajectory properties.
	TrajectoryConfig m_Config
	{
		15_fps,					// Robot max velocity I think. (Or whatever max velocity or acceleration you wish to put.)
		4_fps_sq				// Robot max acceleration I think.
	};
};
/////////////////////////////////////////////////////////////////////////////
#endif