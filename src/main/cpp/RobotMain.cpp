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
    m_pHopper               = new CHopper();
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
    delete m_pHopper;
    delete m_pBlinkin;

    // Set pointers to nullptrs.
    m_pDriveController  = nullptr;
    m_pTimer            = nullptr;
    m_pDrive 			= nullptr;
    m_pIntake			= nullptr;
    m_pTurret			= nullptr;
    m_pShooter			= nullptr;
    m_pHopper           = nullptr;
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
    m_pShooter->Init();
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
    static bool bHasFired       = false;
    static bool bManualMoving   = false;
    
    /********************************************************************
        Drive Controller - Toggle Intake (Button A)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonA))
    {
        if (m_nTeleopState == (int)eTeleopIntake)
        {
            // Leave to idle.
            m_nTeleopState = eTeleopIdle;
        }
        else
        {
            // Start intaking.
            m_nTeleopState = eTeleopIntake;
        }
    }

    /********************************************************************
        Drive Controller - Vision Aiming (Left Bumper)
    ********************************************************************/
    if ((m_pDriveController->GetRawButtonPressed(eButtonLB)) && !(m_pDriveController->GetRawAxis(eRightTrigger) >= 0.65))
    {
        // Set state to Aiming.
        m_nTeleopState = eTeleopAiming;
    }
    if ((m_pDriveController->GetRawButtonReleased(eButtonLB)) && !(m_pDriveController->GetRawAxis(eRightTrigger) >= 0.65))
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

    /********************************************************************
        Drive Controller - Fire (Right Trigger)
    ********************************************************************/
    if (m_pDriveController->GetRawAxis(eRightTrigger) >= 0.65)
    {
        // Set state to Firing.
        m_nTeleopState = eTeleopFiring;
        bHasFired = true;
    }
    else
    {
        if (bHasFired)
        {
            // Has been fired, return to idle.
            m_nTeleopState = eTeleopIdle;
            bHasFired = false;
        }
    }

    /********************************************************************
        Drive Controller - AutoFire (Right Trigger + Left Bumper)
    ********************************************************************/
    if (m_pDriveController->GetRawButton(eButtonLB) && m_pDriveController->GetRawAxis(eRightTrigger) >= 0.65)
    {
        // Set the state to AutoFire.
        m_nTeleopState = eTeleopAutoFiring;
    }
    // Other states related to this button will set state back to Idle.

    /********************************************************************
        Drive Controller - Manual Move Turret Left (Left POV)
    ********************************************************************/
    if (m_pDriveController->GetPOV() == 270)
    {
        // Manually move left.
        m_pTurret->SetState(eTurretManualRev);
        bManualMoving = true;
    }
    else
    {
    /********************************************************************
        Drive Controller - Manual Move Turret Right (Right POV)
    ********************************************************************/
        if (m_pDriveController->GetPOV() == 90)
        {
            // Manually move right.
            m_pTurret->SetState(eTurretManualFwd);
            bManualMoving = true;
        }
        else
        {
            if (bManualMoving)
            {
                // No longer pressing any buttons, move to Idle.
                m_pTurret->SetState(eTurretIdle);
                bManualMoving = false;
            }
        }
    }

    /********************************************************************
        Drive Controller - Toggle Turret "Idle" speed (Button B)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonB))
    {
        m_pShooter->SetShooterState(m_pShooter->GetShooterState() == eShooterIdle ? eShooterStopped : eShooterIdle);
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
            // Stop Shooter, stop Turret, and stop Hood.
            m_pShooter->Stop();
            m_pTurret->Stop();
            // Set robot color.
            m_pBlinkin->SetState(m_pBlinkin->eBeatsPerMin);
            // if (m_pIntake->IsJammed())
            // {
            //     // Jog the belts in reverse.
            //     m_pHopper->Unjam(true);
            // }
            // else
            // {
            //     m_pHopper->Unjam(false);
            // }
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
                Firing - Robot simply fires wherever it is currently aiming.
            ********************************************************************/
            // Set the Turret to idle, we don't want it to move.
            m_pTurret->SetState(eTurretIdle);
            // Set the Shooter to firing speed.
            m_pShooter->SetShooterSetpoint(dShooterFiringVelocity);
            // Start preloading into the shooter.
            m_pHopper->Feed();
            m_pHopper->Preload();
            break;

        case eTeleopAutoFiring  :
            /********************************************************************
                AutoFiring - Robot is firing the Energy into the high goal
                             while tracking the goal actively.
            ********************************************************************/
            // Set the Turret to Tracking mode.
            m_pTurret->SetState(eTurretTracking);
            // Set the Shooter to firing speed.
            m_pShooter->SetShooterSetpoint(dShooterFiringVelocity);
            // Start preloading into the shooter.
            m_pHopper->Feed();
            m_pHopper->Preload();
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
    // Enable joystick control for Teleop use.
    m_pDrive->SetJoystickControl(true);
}

/******************************************************************************
    Description:	Runs every 20ms in a loop after the robot has entered
                    Test mode.
    Arguments:	 	None
    Returns: 		Nothing
******************************************************************************/
void CRobotMain::TestPeriodic()
{
    /********************************************************************
        Drive Controller - Actuate Intake (Button X)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonX))
    {
        m_pIntake->Extend(!m_pIntake->GetExtended());
    }

    /********************************************************************
        Drive Controller - Move Intake Motors (Button B)
    ********************************************************************/
    if (m_pDriveController->GetRawButton(eButtonB))
    {
        m_pIntake->MotorSetPoint(eMotorForward);
    }
    else
    {
        m_pIntake->MotorSetPoint(eMotorStopped);
    }
    
    /********************************************************************
        Drive Controller - Right Winch Up (Button Y)
    ********************************************************************/
    if (m_pDriveController->GetRawButton(eButtonY))
    {
        // Do something.
    }
    else
    {
        // Do nothing.
    }

    /********************************************************************
        Drive Controller - Right Winch Down (Button A)
    ********************************************************************/
    if (m_pDriveController->GetRawButton(eButtonA))
    {
        // Do something.
    }
    else
    {
        // Do nothing.
    }
    
    /********************************************************************
        Drive Controller - Left Winch Up (POV Up)
    ********************************************************************/
    if (m_pDriveController->GetPOV() == 1)
    {
        // Do something.
    }
    else
    {
        // Do nothing.
    }

    /********************************************************************
        Drive Controller - Left Winch Down (POV Down)
    ********************************************************************/
    if (m_pDriveController->GetPOV() == 180)
    {
        // Do something.
    }
    else
    {
        // Do nothing.
    }

    /********************************************************************
        Drive Controller - Turret Left (POV Left)
    ********************************************************************/
    if (m_pDriveController->GetPOV() == 270)
    {
        m_pTurret->SetState(eTurretManualFwd);
    }
    else
    {
    /********************************************************************
        Drive Controller - Turret Right (POV Right)
    ********************************************************************/
        if (m_pDriveController->GetPOV() == 90)
        {
            m_pTurret->SetState(eTurretManualRev);
        }
        else
        {
            m_pTurret->SetState(eTurretIdle);
        }
    }

    /********************************************************************
        Drive Controller - Shooter Forward (Right Trigger, Start for Full)
    ********************************************************************/
    if (m_pDriveController->GetRawAxis(eRightTrigger) >= 0.65)
    {
        if (m_pDriveController->GetRawButton(eStart))
        {
            m_pShooter->SetShooterSetpoint(dShooterFiringVelocity);
        }
        else
        {
            m_pShooter->SetShooterSetpoint(dShooterIdleVelocity);
        }
    }
    else
    {
    /********************************************************************
        Drive Controller - Shooter Reverse (Left Trigger)
    ********************************************************************/
        if (m_pDriveController->GetRawAxis(eLeftTrigger) >= 0.65)
        {
            m_pShooter->SetShooterState(eShooterManualRev);
        }
        else
        {
            m_pShooter->SetShooterState(eShooterStopped);
        }
    }

    /********************************************************************
        Drive Controller - Hood Up (Right Bumper)
    ********************************************************************/
    if (m_pDriveController->GetRawButton(eButtonRB))
    {
        m_pShooter->SetHoodState(eHoodManualFwd);
    }
    else
    {
    /********************************************************************
        Drive Controller - Hood Down (Left Bumper)
    ********************************************************************/
        if (m_pDriveController->GetRawButton(eButtonLB))
        {
            m_pShooter->SetHoodState(eHoodManualRev);
        }
        else
        {
            m_pShooter->SetHoodState(eHoodIdle);
        }
    }

    /********************************************************************
        Drive Controller - Actuate Lift (Left AND Right Stick)
    ********************************************************************/
    if (m_pDriveController->GetRawButton(eButtonLS) && m_pDriveController->GetRawButton(eButtonRS))
    {
        // Do something.
    }
    else
    {
        // Do nothing.
    }
    
    // /********************************************************************
    //     Drive Controller - Toggle Vision LEDs (Back Button)
    // ********************************************************************/
    // if (m_pDriveController->GetRawButtonPressed(eBack))
    // {
    //     m_pVisionSwitch->Set(!m_pVisionSwitch->Get());
    // }
    
    // Update Subsystems.
    m_pDrive->Tick();
    m_pTurret->Tick();
    m_pShooter->Tick();
}
///////////////////////////////////////////////////////////////////////////////
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<CRobotMain>(); }
#endif