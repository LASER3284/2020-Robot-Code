/****************************************************************************
	Description:	Implements the CDrive control class.
	Classes:		CDrive
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include "Drive.h"
#include "IOMap.h"

using namespace ctre;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
	Description:	CDrive Constructor.
	Arguments:		Joystick* pDriveController
	Derived From:	Nothing
****************************************************************************/
CDrive::CDrive(Joystick* pDriveController)
{
	m_pDriveController 	= pDriveController;
	m_pLeftMotor1		= new CFalconMotion(1);
	m_pLeftMotor2		= new WPI_TalonFX(2);
	m_pRightMotor1		= new CFalconMotion(3);
	m_pRightMotor2		= new WPI_TalonFX(4);
	m_pRobotDrive		= new DifferentialDrive(*m_pLeftMotor1->GetMotorPointer(), *m_pRightMotor1->GetMotorPointer());
}

/****************************************************************************
	Description:	CDrive Destructor.
	Arguments:		None
	Derived From:	Nothing
****************************************************************************/
CDrive::~CDrive()
{
	delete m_pLeftMotor1;
	delete m_pLeftMotor2;
	delete m_pRightMotor1;
	delete m_pRightMotor2;
	delete m_pRobotDrive;

	m_pLeftMotor1	= nullptr;
	m_pLeftMotor2	= nullptr;
	m_pRightMotor1	= nullptr;
	m_pRightMotor2	= nullptr;
	m_pRobotDrive	= nullptr;
}

/****************************************************************************
	Description:	Initialize drive parameters.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::Init()
{
	// Make motor 2 follow motor 1 on both sides.
	m_pLeftMotor2->Follow(*m_pLeftMotor1->GetMotorPointer());
	m_pRightMotor2->Follow(*m_pRightMotor1->GetMotorPointer());

	// Set max acceleration to .65 seconds.
	m_pLeftMotor1->SetOpenLoopRampRate(.65);
	m_pRightMotor1->SetOpenLoopRampRate(.65);

	// Clear persistant motor controller faults.
	m_pLeftMotor1->ClearStickyFaults();
	m_pRightMotor1->ClearStickyFaults();
}

/****************************************************************************
	Description:	Main method that calls functionality, to be used in a loop.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CDrive::Tick()
{
	// Set variables to joysticks.
	double XAxis = m_pDriveController->GetRawAxis(4);
	double YAxis = -m_pDriveController->GetRawAxis(2);

	// Check if joystick is in deadzone.
	if (fabs(XAxis) < 0.1)
	{
		XAxis = 0;
	}
	if (fabs(YAxis) < 0.1)
	{
		YAxis = 0;
	}

	// Drive the robot.
	m_pRobotDrive->ArcadeDrive(YAxis, XAxis, false);
}
