/******************************************************************************
	Description:	2020 Infinite Recharge Robot Control Software.
	Classes:		CRobotMain
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotMain.h"

using namespace frc;
using namespace std;
///////////////////////////////////////////////////////////////////////////////


/******************************************************************************
	Description:	CRobotMain Constructor.
	Arguments:		None
	Derived From:	TimedRobot
******************************************************************************/
CRobotMain::CRobotMain()
{
    // Create object pointers.
    m_pDriveController  	= new Joystick(0);
    m_pTimer            	= new Timer();
    m_pDrive            	= new CDrive(m_pDriveController);
	m_pIntake				= new CIntake();
	m_pTurret				= new CTurret();
	m_pShooter				= new CShooter();
	m_pBlinkin				= new Blinkin(nBlinkinID);
	m_pAutonomousChooser	= new SendableChooser<string>();
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
	delete m_pShooter;
	delete m_pBlinkin;

    // Set pointers to nullptrs.
    m_pDriveController  = nullptr;
    m_pTimer            = nullptr;
	m_pDrive 			= nullptr;
	m_pIntake			= nullptr;
	m_pTurret			= nullptr;
	m_pShooter			= nullptr;
	m_pBlinkin			= nullptr;
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
	SmartDashboard::PutNumber("Idle Color", 78);

	// Add trajectory options to the autonomous chooser.
	m_pAutonomousChooser->AddOption("Autonomous Idle", "Autonomous Idle");
	m_pAutonomousChooser->AddOption("Alliance Trench", "Alliance Trench");
	m_pAutonomousChooser->AddOption("Front Shield Generator", "Front Shield Generator");
	m_pAutonomousChooser->AddOption("Side Sheild Generator", "Side Sheild Generator");
	m_pAutonomousChooser->AddOption("Opposing Trench", "Opposing Trench");
	m_pAutonomousChooser->AddOption("Power Port", "Power Port");
	m_pAutonomousChooser->AddOption("Take Power Cells", "Take Power Cells");
	m_pAutonomousChooser->AddOption("Test Path", "Test Path");
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

	// Get the select auto mode from SmartDashboard.
	string m_strAutonomousSelected = m_pAutonomousChooser->GetSelected();
	if (m_strAutonomousSelected == "Autonomous Idle")
	{
		m_nAutoState = eDoNothing;
	}
	if (m_strAutonomousSelected == "Alliance Trench")
	{
		m_nAutoState = eAllianceTrench;
	}
	if (m_strAutonomousSelected == "Front Shield Generator")
	{
		m_nAutoState = eFrontShieldGenerator;
	}
	if (m_strAutonomousSelected == "Side Sheild Generator")
	{
		m_nAutoState = eSideShieldGenerator;
	}
	if (m_strAutonomousSelected == "Opposing Trench")
	{
		m_nAutoState = eOpposingTrench;
	}
	if (m_strAutonomousSelected == "Power Port")
	{
		m_nAutoState = ePowerPort;
	}
	if (m_strAutonomousSelected == "Take Power Cells")
	{
		m_nAutoState = eTakePowerCells;
	}
	if (m_strAutonomousSelected == "Test Path")
	{
		m_nAutoState = eTestPath;
	}

	// Set the selected trajectory path. 
	if (m_nAutoState == eDoNothing)
	{
		m_pDrive->SetSelectedTrajectory(eDoNothing);
	}
	else
	{
		m_pDrive->SetSelectedTrajectory(m_nAutoState);
	}
}

/******************************************************************************
	Description:	Runs every 20ms in a loop after the robot has entered
                    Autonomous mode.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CRobotMain::AutonomousPeriodic()
{
	// Follow the trajectory.
	m_pDrive->FollowTrajectory();
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
	switch(m_nTeleopState)
	{
		case eTeleopIdle :
			//////////////////////////////////////////////////////////////
			// Idle - Robot is not currently doing anything. May or	  	//
			// may not be driving as well.							  	//
			//////////////////////////////////////////////////////////////
			// Return intake to it's retracted state.
			m_pIntake->Extend(false);
			m_pIntake->MotorSetPoint(eMotorStopped);
			// Idle Shooter, stop Turret, and stop Hood.
			m_pShooter->Stop();
			m_pTurret->Stop();
			// Set robot color.
			m_pBlinkin->SetState(m_pBlinkin->eTwinkle);
			break;

		case eTeleopIntake :
			//////////////////////////////////////////////////////////////
			// Intake - Robot is intaking Energy, and only that.		//
			//////////////////////////////////////////////////////////////
			// Extend intake.
			m_pIntake->Extend(true);
			m_pIntake->MotorSetPoint(eMotorForward);
			// Idle Shooter, stop Turret, and stop Hood.
			m_pShooter->Stop();
			m_pTurret->Stop();
			// Set robot color.
			m_pBlinkin->SetState(m_pBlinkin->eBeatsPerMin);
			break;

		case eTeleopAiming :
			//////////////////////////////////////////////////////////////
			// Aiming - Turret is tracking the position of the high		//
			// goal using the Vision points determined.					//
			//////////////////////////////////////////////////////////////
			// Set the Turret to tracking mode.
			m_pTurret->SetState(eTurretTracking);
			// Set the Hood to tracking mode.
			m_pShooter->SetHoodState(eHoodTracking);
			// Set robot color.
			m_pBlinkin->SetState(m_pBlinkin->eLarsonScanner1);
			break;
		
		case eTeleopFiring :
			//////////////////////////////////////////////////////////////
			// Firing - Robot is firing the Energy into the high goal,	//
			// while tracking the goal actively.						//
			//////////////////////////////////////////////////////////////
			break;

		case eTeleopFollowing :
			//////////////////////////////////////////////////////////////
			// Following - Robot is following a pre-determined path.	//
			//////////////////////////////////////////////////////////////
			break;

		default :
			// Return to idle.
			m_nTeleopState = eTeleopIdle;
			break;
	}

	// Update Subsystems.
    m_pDrive->Tick();
	m_pTurret->Tick();
	m_pShooter->Tick();

	SmartDashboard::PutNumber("State", m_nTeleopState);
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