/******************************************************************************
    Description:	2020 Infinite Recharge Robot Control Software.
    Classes:		CRobotMain
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#ifndef RobotMain_h
#define RobotMain_h

#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <frc/LiveWindow/LiveWindow.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DigitalOutput.h>
#include "Drive.h"
#include "Intake.h"
#include "Turret.h"
#include "Shooter.h"
#include "Hood.h"
#include "Hopper.h"
#include "Lift.h"
#include "Blinkin.h"
#include "TrajectoryConstants.h"

using namespace frc;
using namespace std;
///////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CRobotMain class definition.
    Arguments:		None
    Derived From:	TimedRobot
******************************************************************************/
class CRobotMain : public TimedRobot
{
public:
    CRobotMain();
    ~CRobotMain();
    
private:
    // Override methods.
    void RobotInit() override;
    void RobotPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

    // State machines.
    enum TeleopStates
    {
        eTeleopIdle,
        eTeleopIntake,
        eTeleopAiming,
        eTeleopFiring,
        eTeleopAutoFiring,
        eTeleopClimbing,
        eTeleopUnjam,
        eTeleopFollowing
    };

    enum AutoStates
    {
        eAutoIdle,
        eAutoAllianceTrench1,
        eAutoAllianceTrench2,
        eAutoAllianceTrench3,
        eAutoAllianceTrench4,
        eAutoFrontShieldGenerator1,
        eAutoFrontShieldGenerator2,
        eAutoFrontShieldGenerator3,
        eAutoSideShieldGenerator1,
        eAutoSideShieldGenerator2,
        eAutoSideShieldGenerator3,
        eAutoOpposingTrench1,
        eAutoOpposingTrench2,
        eAutoOpposingTrench3,
        eAutoPowerPort1,
        eAutoPowerPort2,
        eAutoTakePowerCells1,
        eAutoTestPath1
    };

    // Object pointers.
    Joystick*                   m_pDriveController;
    Joystick*                   m_pAuxController;
    Timer*                      m_pTimer;
    CDrive*                     m_pDrive;
    CIntake*                    m_pIntake;
    CTurret*                    m_pTurret;
    CShooter*                   m_pShooter;
    CHood*                      m_pHood;
    CHopper*                    m_pHopper;
    CLift*                      m_pLift;
    Blinkin*                    m_pBlinkin;
    CTrajectoryConstants*		m_pTrajectoryConstants;
    SendableChooser<string>*    m_pAutonomousChooser;

    // Declare variables.
    int         m_nTeleopState;
    int         m_nAutoState;
    int         m_nSelectedTrajectory;
    int         m_nPreviousState;
    double      m_dStartTime;
};
////////////////////////////////////////////////////////////////////////////////
#endif