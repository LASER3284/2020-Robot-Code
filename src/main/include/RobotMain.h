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
#include "Hopper.h"
#include "Blinkin.h"

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
        eTeleopFollowing
    };

    enum AutoStates
    {
        eAllianceTrench = 1, 
        eFrontShieldGenerator, 
        eSideShieldGenerator, 
        eOpposingTrench, 
        ePowerPort, 
        eTakePowerCells, 
        eDoNothing, 
        eTestPath
    };

    // Object pointers.
    Joystick*                   m_pDriveController;
    Timer*                      m_pTimer;
    CDrive*                     m_pDrive;
    CIntake*                    m_pIntake;
    CTurret*                    m_pTurret;
    CShooter*                   m_pShooter;
    CHopper*                    m_pHopper;
    Blinkin*                    m_pBlinkin;
    DigitalOutput*              m_pVisionSwitch;
    CTrajectoryConstants*		m_pTrajectoryConstants;
    SendableChooser<string>*    m_pAutonomousChooser;

    // Declare variables.
    int         m_nTeleopState;
    int         m_nAutoState;
};
////////////////////////////////////////////////////////////////////////////////
#endif