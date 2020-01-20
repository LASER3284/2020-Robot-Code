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
#include "Drive.h"
#include "Intake.h"
#include "Turret.h"

using namespace frc;
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
        eTeleopIdle
    };

    enum AutoStates
    {
        eAutoIdle
    };

    // Object pointers.
    Joystick*           m_pDriveController;
    Timer*              m_pTimer;
    CDrive*             m_pDrive;
    CIntake*            m_pIntake;
    CTurret*            m_pTurret;

    // Declare variables.
    int         m_nTeleopState;
    int         m_nAutoState;
    bool        m_bDriveControllerPOVUpPressed;
	bool        m_bDriveControllerPOVDownPressed;
	bool        m_bDriveControllerPOVLeftPressed;
	bool        m_bDriveControllerPOVRightPressed;
	bool        m_bDriveControllerButtonAPressed;
	bool        m_bDriveControllerButtonBPressed;
	bool        m_bDriveControllerButtonXPressed;
	bool        m_bDriveControllerButtonYPressed;
	bool        m_bDriveControllerButtonLSPressed;
	bool        m_bDriveControllerButtonRSPressed;
};
////////////////////////////////////////////////////////////////////////////////
#endif