/******************************************************************************
	Description:	2020 Infinite Recharge Robot Control Software.
	Classes:		CRobotMain
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotMain.h"
#include "Intake.h"
#include "Turret.h"

using namespace frc;
///////////////////////////////////////////////////////////////////////////////


/******************************************************************************
	Description:	CRobotMain Constructor.
	Arguments:		None
	Derived From:	TimedRobot
******************************************************************************/
CRobotMain::CRobotMain()
{
    // Create object pointers.
    m_pDriveController  = new Joystick(0);
    m_pTimer            = new Timer();
    m_pDrive            = new CDrive(m_pDriveController);
	m_pIntake			= new CIntake();
	m_pTurret			= new CTurret();
}

/******************************************************************************
	Description:	CRobotMain Destructor.
	Arguments:		None
	Derived From:	TimedRobot
******************************************************************************/
CRobotMain::~CRobotMain()
{
    // Delete objects.
    delete m_pDriveController;
    delete m_pTimer;
	delete m_pDrive;
	delete m_pIntake;
	delete m_pTurret;

    // Set pointers to nullptrs.
    m_pDriveController  = nullptr;
    m_pTimer            = nullptr;
	m_pDrive 			= nullptr;
	m_pIntake			= nullptr;
	m_pTurret			= nullptr;
}

/****************************************************************************
	Description:	Ran on initial startup of the robot.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CRobotMain::RobotInit()
{
	m_pDrive->Init();
	m_pIntake->Init();
	m_pTurret->Init();
}

/******************************************************************************
	Description:	Runs every 20ms in a loop after the robot has started.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CRobotMain::RobotPeriodic()
{

}

/****************************************************************************
	Description:	Ran only once, when robot enters Disabled mode.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CRobotMain::DisabledInit()
{
	
}

/******************************************************************************
	Description:	Runs every 20ms in a loop after the robot has entered
					Disabled mode.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CRobotMain::DisabledPeriodic()
{

}

/******************************************************************************
	Description:	Ran only once, after the robot has entered Autonomous mode.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CRobotMain::AutonomousInit()
{
    // Disable joystick control to prevent issues during Autonomous.
    m_pDrive->SetJoystickControl(false);
}

/******************************************************************************
	Description:	Runs every 20ms in a loop after the robot has entered
                    Autonomous mode.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CRobotMain::AutonomousPeriodic()
{

}

/******************************************************************************
	Description:	Ran only once, after robot has entered Teleop mode.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CRobotMain::TeleopInit()
{
    // Enable joystick control for Teleop use.
    m_pDrive->SetJoystickControl(true);
}

/******************************************************************************
	Description:	Runs every 20ms in a loop after the robot has entered 
                    Teleop mode.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CRobotMain::TeleopPeriodic()
{
	if (m_pDriveController->GetRawButtonPressed(eButtonB))
	{
		m_pTurret->SetSetpoint(210.0);
	}

	if (m_pDriveController->GetRawButtonPressed(eButtonA))
	{
		m_pTurret->SetSetpoint(-90.0);
	}

	if (m_pDriveController->GetRawButtonPressed(eButtonX))
	{
		m_pTurret->SetSetpoint(0.0);
	}

	if (m_pDriveController->GetRawButtonPressed(eButtonY))
	{
		m_pTurret->SetState(eTurretIdle);
	}

	// Update Drive.
    m_pDrive->Tick();
	m_pTurret->Tick();

	SmartDashboard::PutNumber("State", (int)m_pTurret->GetState());
}

/******************************************************************************
	Description:	Ran only once, after the robot has entered Test mode.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CRobotMain::TestInit() 
{

}

/******************************************************************************
	Description:	Runs every 20ms in a loop after the robot has entered
                    Test mode.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CRobotMain::TestPeriodic()
{
	
}
///////////////////////////////////////////////////////////////////////////////
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<CRobotMain>(); }
#endif