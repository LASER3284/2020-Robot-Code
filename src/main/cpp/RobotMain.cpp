/******************************************************************************
	Description:	2020 Infinite Recharge Robot Control Software.
	Classes:		CRobotMain
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#include "RobotMain.h"
#include "Drive.h"

using namespace frc;

/******************************************************************************
	Description:	CRobotMain Constructor.
	Arguments:		None
	Derived From:	TimedRobot
******************************************************************************/
CRobotMain::CRobotMain()
{
    // Create object pointers.
    m_pDriveController  = new frc::Joystick(0);
    m_pAuxController    = new frc::Joystick(1);
    m_pDrive            = new CDrive(m_pDriveController);
    m_pTimer            = new Timer();
    m_pLiveWindow       = LiveWindow::GetInstance();

    // Assign variables.
    m_nTeleopState                      = eTeleopIdle;
    m_nAutoState                        = eAutoIdle;
    m_bDriveControllerPOVUpPressed 		= false;
	m_bDriveControllerPOVDownPressed	= false;
	m_bDriveControllerPOVLeftPressed	= false;
	m_bDriveControllerPOVRightPressed	= false;
	m_bDriveControllerButtonAPressed	= false;
	m_bDriveControllerButtonBPressed	= false;
	m_bDriveControllerButtonXPressed	= false;
	m_bDriveControllerButtonYPressed	= false;
	m_bDriveControllerButtonLSPressed	= false;
	m_bDriveControllerButtonRSPressed	= false;
}

/******************************************************************************
	Description:	CRobotMain Destructor.
	Arguments:		None
	Derived From:	TimedRobot
******************************************************************************/
CRobotMain::~CRobotMain()
{
    delete m_pDriveController;
    delete m_pAuxController;
    delete m_pDrive;
    delete m_pTimer;

    m_pDriveController  = nullptr;
    m_pAuxController    = nullptr;
    m_pDrive            = nullptr;
    m_pTimer            = nullptr;
}

void CRobotMain::RobotInit()
{
	m_pDrive->Init();
	m_pDrive->GenerateTragectory();
}

void CRobotMain::RobotPeriodic()
{

}

void CRobotMain::AutonomousInit()
{

}

void CRobotMain::AutonomousPeriodic()
{

}

void CRobotMain::TeleopInit()
{

}

void CRobotMain::TeleopPeriodic()
{
	m_pDrive->Tick();
}

void CRobotMain::TestInit() 
{

}

void CRobotMain::TestPeriodic()
{
	
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<CRobotMain>(); }
#endif
