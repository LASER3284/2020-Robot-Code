/******************************************************************************
	Description:	2020 Infinite Recharge Robot Control Software.
	Classes:		CRobotMain
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#include "RobotMain.h"
#include <frc/smartdashboard/SmartDashboard.h>

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
    m_pDrive            = new CDrive(m_pDriveController);
    m_pTimer            = new Timer();
}

/******************************************************************************
	Description:	CRobotMain Destructor.
	Arguments:		None
	Derived From:	TimedRobot
******************************************************************************/
CRobotMain::~CRobotMain()
{
    delete m_pDriveController;
	delete m_pDrive;
    delete m_pTimer;
    m_pDriveController  = nullptr;
	m_pDrive 			= nullptr;
    m_pTimer            = nullptr;
}

/****************************************************************************
	Description:	Ran on initial startup of the robot.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CRobotMain::RobotInit()
{
	m_pDrive->Init();
}

/******************************************************************************
	Description:	Runs every 20ms in a loop after the robot has started.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CRobotMain::RobotPeriodic()
{

}

/******************************************************************************
	Description:	Ran only once, after the robot has entered Autonomous mode.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CRobotMain::AutonomousInit()
{

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
}

/******************************************************************************
	Description:	Runs every 20ms in a loop after the robot has entered 
                    Teleop mode.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CRobotMain::TeleopPeriodic()
{
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