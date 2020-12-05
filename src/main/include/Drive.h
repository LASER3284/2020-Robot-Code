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
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <AHRS.h>
#include "FalconMotion.h"
#include "TrajectoryConstants.h"


using namespace frc;
using namespace std;
using namespace units;

// Default constants for the CDrive class.
const auto    	                    m_dDrivebaseWidth               	    = 27.019388;
const auto 		                    m_kS 							        = 0.39_V;											        //	|	Drive characterization constants.
const auto 		                    m_kV 							        = 0.0544 * 1_V * 1_s / 1_in;						        //	|	Drive characterization constants.
const auto 		                    m_kA 							        = 0.00583 * 1_V * 1_s * 1_s / 1_in;					        //	|	Drive characterization constants.
const DifferentialDriveKinematics   m_kDriveKinematics                      = DifferentialDriveKinematics(inch_t(m_dDrivebaseWidth));   //  |	Drive characterization constants.
const double                        m_dDefaultBeta                         	= 1.100;	// 1.800
const double                        m_dDefaultZeta                         	= 0.500;	// 0.9
const double	                    m_dDefaultProportional					= 0.265;	// Left drive proportional value. // 0.000179
const double	                    m_dDefaultIntegral						= 0.000;	// Left drive integral value.
const double 	                    m_dDefaultDerivative					= 0.000;	// Left drive derivative value.
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
    bool TrajectoryIsFinished();
    int  GetTotalTrajectoryTime();
    void SetSelectedTrajectory(int nAutoState);
    void SetJoystickControl(bool bJoystickControl);
    void SetMotorExpiration(double dTimeout);
    void SetMotorSafety(bool bEnabled);

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
    CTrajectoryConstants			m_pTrajectoryConstants;
    Trajectory              		m_Trajectory;

    // Member variables.
    bool					m_bJoystickControl;
    bool                    m_bMotionProfile;
    double                  m_dBeta;
    double                  m_dZeta;
    double					m_dProportional;
    double					m_dIntegral;
    double					m_dDerivative;
};
/////////////////////////////////////////////////////////////////////////////
#endif