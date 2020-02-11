/******************************************************************************
    Description:	2020 Infinite Recharge Robot Control Software.
    Classes:		CRobotMain
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotMain.h"

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
    m_pShooter			= new CShooter();
    m_pHopper			= new CHopper();
    m_pBlinkin			= new Blinkin(nBlinkinID);
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
    SmartDashboard::PutNumber("Idle Color", m_pBlinkin->eTwinkle);
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
    switch(m_nAutoState)
    {
        case eAutoIdle :
            break;

        default :
            break;
    }
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
    static bool bIsIntaking = false;
    /********************************************************************
        Drive Controller - Toggle Intake (Button A)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonA))
    {
        if (!bIsIntaking)
        {
            // Set the state to intaking.
            m_nTeleopState = eTeleopIntake;
            bIsIntaking = true;
        }
        else
        {
            // If pressed while still intaking..
            if (m_nTeleopState == eTeleopIntake)
            {
                // Go back to idle.
                m_nTeleopState = eTeleopIdle;
            }
            // If it isn't still intaking, do nothing to prevent it
            // from leaving it's current state.
            bIsIntaking = false;
        }
    }

    /********************************************************************
        Drive Controller - Vision Aiming (Button Y)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonY))
    {
        // Set state to Aiming.
        m_nTeleopState = eTeleopAiming;
    }
    if (m_pDriveController->GetRawButtonReleased(eButtonY))
    {
        // If released while still in aiming...
        if (m_nTeleopState == eTeleopAiming)
        {
            // Go back to idle.
            m_nTeleopState = eTeleopIdle;
        }
        // If the button was released but we didn't change states
        // yet, do nothing to prevent it from leaving it's current
        // state.
    }

    switch(m_nTeleopState)
    {
        case eTeleopIdle :
            /********************************************************************
                Idle - Robot is not currently doing anything.
                       May or may not be driving as well.
            ********************************************************************/
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
            /********************************************************************
                Intake - Robot is intaking Energy.
            ********************************************************************/
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
            /********************************************************************
                Aiming - Turret is tracking the position of the high goal
                         using the Vision points determined.
            ********************************************************************/
            // Set the Turret to tracking mode.
            m_pTurret->SetState(eTurretTracking);
            // Set the Hood to tracking mode.
            m_pShooter->SetHoodState(eHoodTracking);
            // Set robot color.
            m_pBlinkin->SetState(m_pBlinkin->eLarsonScanner1);
            break;
        
        case eTeleopFiring :
            /********************************************************************
                Firing - Robot is firing the Energy into the high goal
                         while tracking the goal actively.
            ********************************************************************/
            break;

        case eTeleopFollowing :
            /********************************************************************
                Following - Robot is following a pre-determined path.
            ********************************************************************/
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