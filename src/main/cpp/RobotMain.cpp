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
    m_pAuxController        = new Joystick(1);
    m_pTimer            	= new Timer();
    m_pDrive            	= new CDrive(m_pDriveController);
    m_pIntake				= new CIntake();
    m_pTurret				= new CTurret();
    m_pShooter				= new CShooter();
    m_pHopper               = new CHopper();
    m_pLift                 = new CLift();
    m_pBlinkin				= new Blinkin(nBlinkinID);
    m_pAutonomousChooser	= new SendableChooser<string>();

    m_nTeleopState  = 0;
    m_nAutoState    = 0;
    m_dStartTime    = 0.0;
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
    delete m_pAuxController;
    delete m_pTimer;
    delete m_pDrive;
    delete m_pIntake;
    delete m_pTurret;
    delete m_pShooter;
    delete m_pHopper;
    delete m_pLift;
    delete m_pBlinkin;
    delete m_pAutonomousChooser;

    // Set pointers to nullptrs.
    m_pDriveController   = nullptr;
    m_pAuxController     = nullptr;
    m_pTimer             = nullptr;
    m_pDrive 			 = nullptr;
    m_pIntake			 = nullptr;
    m_pTurret			 = nullptr;
    m_pShooter			 = nullptr;
    m_pHopper            = nullptr;
    m_pLift              = nullptr;
    m_pBlinkin			 = nullptr;
    m_pAutonomousChooser = nullptr;
}

/****************************************************************************
    Description:	Ran on initial startup of the robot.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CRobotMain::RobotInit()
{
    // Call Init of all classes required.
    m_pDrive->Init();
    m_pIntake->Init();
    m_pTurret->Init();
    m_pShooter->Init();
    m_pLift->Init();
    // Put Autonomous things on the Dashboard.
    SmartDashboard::PutNumber("Idle Color", m_pBlinkin->eTwinkle);
    m_pAutonomousChooser->SetDefaultOption("Autonomous Idle", "Autonomous Idle");
    m_pAutonomousChooser->AddOption("Alliance Trench", "Alliance Trench");
    m_pAutonomousChooser->AddOption("Front Shield Generator", "Front Shield Generator");
    m_pAutonomousChooser->AddOption("Test Path", "Test Path");
    SmartDashboard::PutData(m_pAutonomousChooser);
    // Start the Timer.
    m_pTimer->Start();
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
    // Disable LEDs
    m_pShooter->SetVisionLED(false);
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
    std::cout << "STRING : " << m_strAutonomousSelected << std::endl;
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

    std::cout << m_nAutoState << std::endl;
}

/******************************************************************************
    Description:	Runs every 20ms in a loop after the robot has entered
                    Autonomous mode.
    Arguments:	 	None
    Returns: 		Nothing
******************************************************************************/
void CRobotMain::AutonomousPeriodic()
{
    static double dStartTime = m_pTimer->Get();
    switch (m_nAutoState)
    {
        case eDoNothing :
            // Do nothing.
            break;

        case eTestPath :
            // Simply follow the path.
            m_pDrive->FollowTrajectory();
            break;

        case eAllianceTrench :
            std::cout << "ALLIANCE TRENCH" << std::endl;
            if (m_pTimer->Get() - dStartTime < 7.00)
            {
                // Set the Turret to tracking mode.
                m_pTurret->SetVision();
                // Enabled LEDs
                m_pShooter->SetVisionLED(true);
                // Set robot color.
                m_pBlinkin->SetState(m_pBlinkin->eLarsonScanner1);
                // Set the Shooter setpoint.
                m_pShooter->SetShooterSetpoint(dShooterFiringVelocity);
                // Start shooting when ready.
                if (m_pShooter->IsShooterAtSetpoint())
                {
                    // Start preloading into the shooter.
                    m_pHopper->Feed(true);
                    m_pHopper->Preload(true);
                    m_pIntake->RetentionMotor(true);
                }
            }
            // Follow Trajectory after given time to shoot.
            if ((m_pTimer->Get() - dStartTime) >= 7.00)
            {
                // Set the Shooter setpoint.
                m_pShooter->SetShooterSetpoint(dShooterIdleVelocity);
                // Disable LEDs
                m_pShooter->SetVisionLED(false);
                // Start intake, Follow Trajectory.
                m_pIntake->Extend(true);
                m_pIntake->IntakeMotor(true);
                m_pIntake->RetentionMotor(true);
                m_pDrive->FollowTrajectory();
            }
            break;
    }

    // Call all subsystem Ticks.
    m_pDrive->Tick();
    m_pShooter->Tick();
    m_pTurret->Tick();
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
    static bool bTurretMoving   = false;
    static bool bHoodMoving     = false;
    
    /********************************************************************
        Drive Controller - Toggle Intake (Button A)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonA))
    {
        if (m_nTeleopState == eTeleopIntake)
        {
            // Leave to idle.
            m_nTeleopState = eTeleopIdle;
        }
        else
        {
            // Start intaking.
            m_nTeleopState = eTeleopIntake;
            // Set up timer.
            m_dStartTime = m_pTimer->Get();
        }
    }

    /********************************************************************
        Drive Controller - Vision Aiming (Left Bumper)
    ********************************************************************/
    if ((m_pDriveController->GetRawButton(eButtonLB)) && !(m_pDriveController->GetRawAxis(eRightTrigger) >= 0.65))
    {
        // Set state to Aiming.
        m_nTeleopState = eTeleopAiming;
    }
    if (!(m_pDriveController->GetRawButton(eButtonLB)) && !(m_pDriveController->GetRawAxis(eRightTrigger) >= 0.65))
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
    if (m_pDriveController->GetRawAxis(eRightTrigger) >= 0.65 && !m_pDriveController->GetRawButton(eButtonRB))
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
            m_pShooter->SetShooterState(eShooterStopped);
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
        bTurretMoving = true;
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
            bTurretMoving = true;
        }
        else
        {
            if (bTurretMoving)
            {
                // No longer pressing any buttons, move to Idle.
                m_pTurret->SetState(eTurretIdle);
                bTurretMoving = false;
            }
        }
    }

    /********************************************************************
        Drive Controller - Toggle Shooter "Idle" speed (Button B)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonB))
    {
        m_pShooter->SetShooterState(m_pShooter->GetShooterState() == eShooterIdle ? eShooterStopped : eShooterIdle);
    }

    /********************************************************************
        Drive Controller - Manual Move Hood Up (Up POV)
    ********************************************************************/
    if (m_pDriveController->GetPOV() == 1)
    {
        // Manual move up.
        m_pShooter->SetHoodState(eHoodManualFwd);
        bHoodMoving = true;
    }
    else
    {
    /********************************************************************
        Drive Controller - Manual Move Hood Down (Down POV)
    ********************************************************************/
        if (m_pDriveController->GetPOV() == 180)
        {
            // Manual move down.
            m_pShooter->SetHoodState(eHoodManualRev);
            bHoodMoving = true;
        }
        else
        {
            if (bHoodMoving)
            {
                // No longer moving, set to idle.
                m_pShooter->SetHoodState(eHoodIdle);
                bHoodMoving = false;
            }
        }
    }

    /********************************************************************
        Drive Controller - Bump Retention backwards (Aux Button A)
    ********************************************************************/
    if (m_pAuxController->GetRawButtonPressed(eButtonA))
    {
        m_nPreviousState = m_nTeleopState;
        m_nTeleopState = eTeleopUnjam;
    }

    switch(m_nTeleopState)
    {
        case eTeleopIdle :
            /********************************************************************
                Idle - Robot is not currently doing anything.
                       May or may not be driving as well.
            ********************************************************************/
            // Disable LEDs
            m_pShooter->SetVisionLED(false);
            // Return intake to it's retracted state.
            m_pIntake->Extend(false);
            m_pIntake->IntakeMotor(false);
            m_pIntake->RetentionMotor(false);
            // Return Lift arm to it's lower position.
            m_pLift->ExtendArm(false);
            // Idle the arm.
            m_pLift->ReverseIdle(true);
            // Idle Shooter, stop Turret, and stop Hood.
//          m_pShooter->Stop();
            m_pShooter->SetHoodState(eHoodIdle);
            m_pTurret->Stop();
            m_pHopper->Feed(false);
            m_pHopper->Preload(false);
            // Set robot color.
            m_pBlinkin->SetState(m_pBlinkin->eTwinkle);
            break;

        case eTeleopIntake :
            /********************************************************************
                Intake - Robot is intaking Energy.
            ********************************************************************/
            // Disable LEDs
            m_pShooter->SetVisionLED(false);
            // Return Lift arm to it's lower position.
            m_pLift->ExtendArm(false);
            // Idle the arm.
            m_pLift->ReverseIdle(true);
            // Extend intake.
            m_pIntake->Extend(true);
            // Start intake on a half second delay.
            if (m_pTimer->Get() - m_dStartTime == 0.5)
            {
                m_pIntake->IntakeMotor(true);
                m_pIntake->RetentionMotor(true);
            }
            // Stop Shooter, stop Turret, and stop Hood.
            m_pShooter->Stop();
            m_pTurret->Stop();
            m_pHopper->Feed(false);
            m_pHopper->Preload(false);
            // Set robot color.
            m_pBlinkin->SetState(m_pBlinkin->eBeatsPerMin);
            break;

        case eTeleopAiming :
            /********************************************************************
                Aiming - Turret is tracking the position of the high goal
                         using the Vision points determined.
            ********************************************************************/
            // Return Lift arm to it's lower position.
            m_pLift->ExtendArm(false);
            // Idle the arm.
            m_pLift->ReverseIdle(true);
            // Set the Turret to tracking mode.
            m_pTurret->SetVision();
            // Enabled LEDs
            m_pShooter->SetVisionLED(true);
            // Set the Hood to tracking mode.
            m_pShooter->SetHoodState(eHoodTracking);
            // Set robot color.
            m_pBlinkin->SetState(m_pBlinkin->eLarsonScanner1);
            break;
        
        case eTeleopFiring :
            /********************************************************************
                Firing - Robot simply fires wherever it is currently aiming.
            ********************************************************************/
            // Return Lift arm to it's lower position.
            m_pLift->ExtendArm(false);
            // Idle the arm.
            m_pLift->ReverseIdle(true);
            // Enabled LEDs
            m_pShooter->SetVisionLED(false);
            // Set the Turret to idle, we don't want it to move.
            m_pTurret->SetState(eTurretIdle);
            // Set the Shooter to firing speed.
            m_pShooter->SetShooterSetpoint(dShooterFiringVelocity);
            if (m_pShooter->IsShooterAtSetpoint())
            {
                // Start preloading into the shooter.
                m_pHopper->Feed(true);
                m_pHopper->Preload(true);
                m_pIntake->RetentionMotor(true);
            }
            // Set robot color.
            m_pBlinkin->SetState(m_pBlinkin->eStrobe2);
            break;

        case eTeleopAutoFiring  :
            /********************************************************************
                AutoFiring - Robot is firing the Energy into the high goal
                             while tracking the goal actively.
            ********************************************************************/
            // Return Lift arm to it's lower position.
            m_pLift->ExtendArm(false);
            // Idle the arm.
            m_pLift->ReverseIdle(true);
            // Enabled LEDs
            m_pShooter->SetVisionLED(true);
            // Set the Turret to Tracking mode.
            m_pTurret->SetState(eTurretTracking);
            // Set the Shooter to firing speed.
            m_pShooter->SetShooterSetpoint(dShooterFiringVelocity);
            if (m_pShooter->IsShooterAtSetpoint()) 
            {
                // Start preloading into the shooter.
                m_pHopper->Feed();
                m_pHopper->Preload();
                m_pIntake->RetentionMotor(true);
            }
            // Set robot color.
            m_pBlinkin->SetState(m_pBlinkin->eStrobe2);
            break;

        case eTeleopClimbing :
            /********************************************************************
                Climbing - Robot is beginning to climb for Endgame.
            ********************************************************************/
            // Stop idling the arm.
            m_pLift->ReverseIdle(false);
            // Disable LEDs
            m_pShooter->SetVisionLED(false);
            // Start the Lift state machine.
            m_pLift->SetState(eLiftExtend);
            // Set robot color.
            m_pBlinkin->SetState(m_pBlinkin->eOrange);
            break;

        case eTeleopUnjam :
            /********************************************************************
                Unjam - Intake is jammed, attempt unjamming.
            ********************************************************************/
            // Unjam the intake.
            m_pIntake->Unjam();
            // Run for a second.
            if ((m_pTimer->Get() - m_dStartTime) == 1.0)
            {
                // Stop motor.
                m_pIntake->RetentionMotor(false);
                // Go back to previous state.
                m_nTeleopState = m_nPreviousState;
            }
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

    SmartDashboard::PutNumber("Shooter State", m_pShooter->GetShooterState());
    SmartDashboard::PutNumber("Shooter At Setpoint", m_pShooter->IsShooterAtSetpoint());
    SmartDashboard::PutNumber("Retention Amperage", m_pIntake->GetRetentionCurrent());
    SmartDashboard::PutNumber("Intake Amperage", m_pIntake->GetIntakeCurrent());
    SmartDashboard::PutNumber("State", m_nTeleopState);
}

/******************************************************************************
    Description:	Ran only once, after the robot has entered Test mode.
    Arguments:	 	None
    Returns: 		Nothing
******************************************************************************/
void CRobotMain::TestInit() 
{
    // Enable joystick control for Test use.
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
    // Enabled LEDs
    m_pShooter->SetVisionLED(true);

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
        m_pIntake->IntakeMotor(true);
        m_pIntake->RetentionMotor(true);
    }
    else
    {
        m_pIntake->IntakeMotor(false);
        m_pIntake->RetentionMotor(false);
    }
    
    /********************************************************************
        Drive Controller - Right Winch Up (Button Y)
    ********************************************************************/
    if (m_pDriveController->GetRawButton(eButtonY))
    {
        m_pLift->TestRightWinch(0.5);
    }
    else
    {
    /********************************************************************
        Drive Controller - Right Winch Down (Button A)
    ********************************************************************/
        if (m_pDriveController->GetRawButton(eButtonA))
        {
            m_pLift->TestRightWinch(-0.5);
        }
        else
        {
            m_pLift->TestRightWinch(0.0);
        }
    }
    
    /********************************************************************
        Drive Controller - Left Winch Up (POV Up)
    ********************************************************************/
    if (m_pDriveController->GetPOV() == 1)
    {
        m_pLift->TestLeftWinch(0.5);
    }
    else
    {
    /********************************************************************
        Drive Controller - Left Winch Down (POV Down)
    ********************************************************************/
        if (m_pDriveController->GetPOV() == 180)
        {
            m_pLift->TestLeftWinch(-0.5);
        }
        else
        {
            m_pLift->TestLeftWinch(0.0);
        }
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
        Drive Controller - Actuate Lift (Right Stick)
    ********************************************************************/
    if (m_pDriveController->GetRawButton(eButtonRS))
    {
        m_pLift->ExtendArm(true);
    }
    else
    {
        m_pLift->ExtendArm(false);
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