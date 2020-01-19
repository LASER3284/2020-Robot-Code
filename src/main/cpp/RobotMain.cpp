/******************************************************************************
	Description:	2020 Infinite Recharge Robot Control Software.
	Classes:		CRobotMain
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotMain.h"
#include "Intake.h"

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

    // Set pointers to nullptrs.
    m_pDriveController  = nullptr;
    m_pTimer            = nullptr;
	m_pDrive 			= nullptr;
	m_pIntake			= nullptr;
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
	// Update Drive.
    m_pDrive->Tick();
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