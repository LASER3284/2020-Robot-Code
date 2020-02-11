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
#include <frc/shuffleboard/Shuffleboard.h>
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
#include "TrajectoryConstants.h"


using namespace frc;
using namespace std;
using namespace units;

// Default constants for the CDrive class.
const auto 		m_dDefaultkS 							= 0.392_V;											//	|
const auto 		m_dDefaultkV 							= 0.0563 * 1_V * 1_s / 1_in;						//	|	Drive characterization constants.
const auto 		m_dDefaultkA 							= 0.00573 * 1_V * 1_s * 1_s / 1_in;					//	|
const double    m_dDefaultBeta                         	= 2.200;	// 1.800
const double    m_dDefaultZeta                         	= 0.900;	// 0.9
const double	m_dDefaultProportional					= 0.259;	// Left drive proportional value. // 0.000179
const double	m_dDefaultIntegral						= 0.000;	// Left drive integral value.
const double 	m_dDefaultDerivative					= 0.000;	// Left drive derivative value.
const auto    	m_dDefaultDrivebaseWidth               	= 29.83;
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CDrive class definition.
    Arguments:		Joystick pDriveController
    Derived From:	Nothing
******************************************************************************/
class CDrive
{
public:
    CDrive(Joystick *pDriveController);
    ~CDrive();
    void Init();
    void Tick();
    void GenerateTrajectory(vector<Pose2d> pWaypoints, meters_per_second_t MaxSpeed, meters_per_second_squared_t MaxAcceleration);
    void FollowTrajectory();
    void SetDrivePowers(volt_t dLeftVoltage, volt_t dRightVoltage);
    DifferentialDriveWheelSpeeds GetWheelSpeeds();
    void ResetOdometry();
    void ResetEncoders();
    void Stop();
    bool GetIsTrajectoryFinished();
    void SetJoystickControl(bool bJoystickControl);
    void SetMotorExpiration(double dTimeout);
    void SetMotorSafety(bool bEnabled);

private:
    // Object Pointers.
    Joystick*                       m_pDriveController;
    CTrajectoryConstants			m_pTrajectoryConstants;
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
    bool					m_bJoystickControl;
    bool                    m_bMotionProfile;
    double                  m_dBeta;
    double                  m_dZeta;
    double					m_dProportional;
    double					m_dIntegral;
    double					m_dDerivative;
    double					m_dDriveBaseWidth;
};
/////////////////////////////////////////////////////////////////////////////
#endif