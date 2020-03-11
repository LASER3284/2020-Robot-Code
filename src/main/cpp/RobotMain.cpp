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
    m_pHood                 = new CHood();
    m_pHopper               = new CHopper();
    m_pLift                 = new CLift();
    m_pBlinkin				= new Blinkin(nBlinkinID);
    m_pAutonomousChooser	= new SendableChooser<string>();

    m_nTeleopState      = eTeleopStopped;
    m_nAutoState        = eAutoIdle;
    m_dStartTime        = 0.0;
    m_nPreviousState    = eTeleopStopped;
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
    delete m_pHood;
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
    m_pHood              = nullptr;
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
    m_pHood->Init();
    m_pIntake->Init();
    m_pTurret->Init();
    m_pShooter->Init();
    m_pLift->Init();
    // Put Autonomous things on the Dashboard.
    SmartDashboard::PutNumber("Idle Color", m_pBlinkin->eTwinkle);
    m_pAutonomousChooser->SetDefaultOption("Autonomous Idle", "Autonomous Idle");
    m_pAutonomousChooser->AddOption("Alliance Trench", "Alliance Trench");
    m_pAutonomousChooser->AddOption("Front Shield Generator", "Front Shield Generator");
    m_pAutonomousChooser->AddOption("Side Shield Generator", "Side Sheild Generator");
    m_pAutonomousChooser->AddOption("Opposing Trench", "Opposing Trench");
    m_pAutonomousChooser->AddOption("Power Port", "Power Port");
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
    // Set joystick control.
    m_pDrive->SetJoystickControl(false);
    // Disable LEDs
    m_pShooter->SetVisionLED(true);
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

    // Enable hood safety.
    m_pHood->SetHoodSafety(true);

    // Get Auto start time.
    m_dStartTime = m_pTimer->Get();

    // Get the select auto mode from SmartDashboard.
    string m_strAutonomousSelected = m_pAutonomousChooser->GetSelected();
    std::cout << "STRING : " << m_strAutonomousSelected << std::endl;
    if (m_strAutonomousSelected == "Autonomous Idle")
    {
        m_nAutoState = eAutoIdle;
        m_nSelectedTrajectory = eDoNothing;
    }
    if (m_strAutonomousSelected == "Alliance Trench")
    {
        m_nAutoState = eAutoAllianceTrench1;
        m_nSelectedTrajectory = eAllianceTrench1;
    }
    if (m_strAutonomousSelected == "Front Shield Generator")
    {
        m_nAutoState = eAutoFrontShieldGenerator1;
        m_nSelectedTrajectory = eFrontShieldGenerator1;
    }
    if (m_strAutonomousSelected == "Side Sheild Generator")
    {
        m_nAutoState = eAutoSideShieldGenerator1;
        m_nSelectedTrajectory = eSideShieldGenerator1;
    }
    if (m_strAutonomousSelected == "Opposing Trench")
    {
        m_nAutoState = eAutoOpposingTrench1;
        m_nSelectedTrajectory = eOpposingTrench1;
    }
    if (m_strAutonomousSelected == "Power Port")
    {
        m_nAutoState = eAutoPowerPort1;
        m_nSelectedTrajectory = ePowerPort1;
    }
    if (m_strAutonomousSelected == "Take Power Cells")
    {
        m_nAutoState = eAutoTakePowerCells1;
        m_nSelectedTrajectory = eTakePowerCells1;
    }
    if (m_strAutonomousSelected == "Test Path")
    {
        m_nAutoState = eAutoTestPath1;
        m_nSelectedTrajectory = eTestPath1;
    }

    // Set the selected trajectory path. 
    if (m_nAutoState == eAutoIdle)
    {
        // Stop all robot functions and do nothing.
        m_pDrive->SetSelectedTrajectory(m_nSelectedTrajectory);
    }
    else
    {
        m_pDrive->SetSelectedTrajectory(m_nSelectedTrajectory);
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
    // Check if the turret had overrun.
    m_pTurret->CheckOverrun();

    m_pLift->ExtendArm(false);
    switch (m_nAutoState)
    {
        case eAutoIdle :
            // Do nothing.
            m_pHood->Stop();
            m_pShooter->Stop();
            m_pShooter->SetVisionLED(false);
            m_pTurret->Stop();
            m_pHopper->Feed(false);
            m_pHopper->Preload(false);
            m_pIntake->RetentionMotor(false);
            break;

        case eAutoTestPath1 :
            // Simply follow the path.
            m_pDrive->FollowTrajectory();
            break;

        case eAutoAllianceTrench1 :
            // Fire the 3 balls.
            if (fabs(m_pTimer->Get() - m_dStartTime) < 6.00)
            {
                // Set the Turret to tracking mode.
                m_pTurret->SetVision(true);
                // Enabled LEDs
                m_pShooter->SetVisionLED(true);
                // Set robot color.
                m_pBlinkin->SetState(m_pBlinkin->eLarsonScanner1);
                // Set the Shooter setpoint.
                m_pShooter->SetSetpoint(4000);
                // Start shooting when ready.
                if (m_pShooter->IsAtSetpoint())
                {
                    // Start preloading into the shooter.
                    m_pHopper->Feed(true);
                    m_pHopper->Preload(true);
                    m_pIntake->RetentionMotor(true);
                }
            }
            else
            {
                // Generate new trajectory and move to the next state.
                m_pDrive->SetSelectedTrajectory(eAllianceTrench1);
                m_nAutoState = eAutoAllianceTrench2;
            }
            break;

        case eAutoAllianceTrench2 :
            // Drive through the trench and intake balls.
            // Set the Shooter setpoint.
            m_pShooter->SetSetpoint(4000);
            // Set the Turret to tracking mode.
            m_pTurret->Stop();
            // Disable LEDs
            m_pShooter->SetVisionLED(false);
            // Set the Hood to tracking mode.
            m_pHood->SetSetpoint(2524);
            // Start intake, Follow Trajectory.
            m_pHopper->Feed(false);
            m_pHopper->Preload(false);
            m_pIntake->Extend(true);
            m_pIntake->IntakeMotor(true);
            m_pIntake->RetentionMotor(true);
            m_pDrive->FollowTrajectory();

            // Move to the next state when the robot is done path following.
            if (m_pDrive->TrajectoryIsFinished())
            {

                m_pDrive->SetSelectedTrajectory(eAllianceTrench2);
                m_nAutoState = eAutoAllianceTrench3;
            }
            break;
        
        case eAutoAllianceTrench3 :
            // Back out of trench.
            // Set the Shooter setpoint.
            m_pShooter->SetSetpoint(dShooterFiringVelocity);
            // Set the Turret to tracking mode.
            m_pTurret->Stop();
            // Disable LEDs
            m_pShooter->SetVisionLED(false);
            // Stop intake, Follow Trajectory.
            m_pHopper->Feed(false);
            m_pHopper->Preload(false);
            m_pIntake->Extend(false);
            m_pIntake->IntakeMotor(false);
            m_pIntake->RetentionMotor(false);
            m_pDrive->FollowTrajectory();

            // Move to the next state when the robot is done path following.
            if (m_pDrive->TrajectoryIsFinished())
            {
                m_nAutoState = eAutoAllianceTrench4;
            }
            break;

        case eAutoAllianceTrench4 :
            // Shoot 5 balls.
            if (fabs(m_pTimer->Get() - m_dStartTime) < 15.0)
            {
                // Set the Turret to tracking mode.
                m_pTurret->SetVision(true);
                // Enabled LEDs
                m_pShooter->SetVisionLED(true);
                // Set robot color.
                m_pBlinkin->SetState(m_pBlinkin->eLarsonScanner1);
                // Set the Hood to tracking mode.
                //m_pHood->SetSetpoint(2524);
                // Set the Shooter setpoint.
                m_pShooter->SetSetpoint(dShooterFiringVelocity);
                // Start shooting when ready.
                if (m_pShooter->IsAtSetpoint())
                {
                    // Start preloading into the shooter.
                    m_pHopper->Feed(true);
                    m_pHopper->Preload(true);
                    m_pIntake->RetentionMotor(true);
                }
            }
            else
            {
                m_nAutoState = eAutoIdle;
            }
            break;

        case eAutoFrontShieldGenerator1 :
            // Drive to the front of the Shield Generator while intaking.
            // Start intake, Follow Trajectory.
            m_pHopper->Feed(false);
            m_pHopper->Preload(false);
            m_pIntake->Extend(true);
            m_pIntake->IntakeMotor(true);
            m_pIntake->RetentionMotor(true);
            m_pDrive->FollowTrajectory();

            // Move to the next state when the robot is done path following.
            if (m_pDrive->TrajectoryIsFinished())
            {
                m_pDrive->SetSelectedTrajectory(eFrontShieldGenerator2);
                m_nAutoState = eAutoFrontShieldGenerator2;
            }
            break;

        case eAutoFrontShieldGenerator2 :
            // Stop intaking and drive backwards to the Power Port.
            m_pHopper->Feed(false);
            m_pHopper->Preload(false);
            m_pIntake->Extend(false);
            m_pIntake->IntakeMotor(false);
            m_pIntake->RetentionMotor(false);
            m_pDrive->FollowTrajectory();

            // Move to the next state when the robot is done path following.
            if (m_pDrive->TrajectoryIsFinished())
            {
                m_nAutoState = eAutoFrontShieldGenerator3;
            }
            break;

        case eAutoFrontShieldGenerator3 :
            // Shoot the 5 balls.
            if (fabs(m_pTimer->Get() - m_dStartTime) < 15.0)
            {
                // Set the Turret to tracking mode.
                m_pTurret->SetVision(true);
                // Enabled LEDs
                m_pShooter->SetVisionLED(true);
                // Set robot color.
                m_pBlinkin->SetState(m_pBlinkin->eLarsonScanner1);
                // Set the Hood to tracking mode.
                m_pHood->SetSetpoint(SmartDashboard::GetNumber("Target Distance", 0.0));
                // Set the Shooter setpoint.
                m_pShooter->SetSetpoint(dShooterFiringVelocity);
                // Start shooting when ready.
                if (m_pShooter->IsAtSetpoint())
                {
                    // Start preloading into the shooter.
                    m_pHopper->Feed(true);
                    m_pHopper->Preload(true);
                    m_pIntake->RetentionMotor(true);
                }
            }
            else
            {
                // Move to auto idle.
                m_nAutoState = eAutoIdle;
            }
            break;

        case eAutoSideShieldGenerator1 : 
            // Drive to the side of the Shield Generator while intaking.
            // Start intake, Follow Trajectory.
            m_pHopper->Feed(false);
            m_pHopper->Preload(false);
            m_pIntake->Extend(true);
            m_pIntake->IntakeMotor(true);
            m_pIntake->RetentionMotor(true);
            m_pDrive->FollowTrajectory();

            // Move to the next state when the robot is done path following.
            if (m_pDrive->TrajectoryIsFinished())
            {
                m_pDrive->SetSelectedTrajectory(eSideShieldGenerator2);
                m_nAutoState = eAutoSideShieldGenerator2;
            }
            break;

        case eAutoSideShieldGenerator2 :
            // Stop intaking and drive backwards to the Power Port.
            m_pHopper->Feed(false);
            m_pHopper->Preload(false);
            m_pIntake->Extend(false);
            m_pIntake->IntakeMotor(false);
            m_pIntake->RetentionMotor(false);
            m_pDrive->FollowTrajectory();

            // Move to the next state when the robot is done path following.
            if (m_pDrive->TrajectoryIsFinished())
            {
                m_nAutoState = eAutoFrontShieldGenerator3;
            }
            break;
        
        case eAutoSideShieldGenerator3 :
            // Shoot the 5 balls.
            if (fabs(m_pTimer->Get() - m_dStartTime) < 15.0)
            {
                // Set the Turret to tracking mode.
                m_pTurret->SetVision(true);
                // Enabled LEDs
                m_pShooter->SetVisionLED(true);
                // Set robot color.
                m_pBlinkin->SetState(m_pBlinkin->eLarsonScanner1);
                // Set the Hood to tracking mode.
                m_pHood->SetSetpoint(SmartDashboard::GetNumber("Target Distance", 0.0));
                // Set the Shooter setpoint.
                m_pShooter->SetSetpoint(dShooterFiringVelocity);
                // Start shooting when ready.
                if (m_pShooter->IsAtSetpoint())
                {
                    // Start preloading into the shooter.
                    m_pHopper->Feed(true);
                    m_pHopper->Preload(true);
                    m_pIntake->RetentionMotor(true);
                }
            }
            else
            {
                // Move to auto idle.
                m_nAutoState = eAutoIdle;
            }
            break;

        case eAutoOpposingTrench1 :
            // Follow path to the opposing trench while intaking.
            // Start intake, Follow Trajectory.
            m_pHopper->Feed(false);
            m_pHopper->Preload(false);
            m_pIntake->Extend(true);
            m_pIntake->IntakeMotor(true);
            m_pIntake->RetentionMotor(true);
            m_pDrive->FollowTrajectory();

            // Move to the next state when the robot is done path following.
            if (m_pDrive->TrajectoryIsFinished())
            {
                m_pDrive->SetSelectedTrajectory(eOpposingTrench2);
                m_nAutoState = eAutoOpposingTrench2;
            }
            break;

        case eAutoOpposingTrench2 :
            // Stop intaking and drive backwards to the Power Port.
            m_pHopper->Feed(false);
            m_pHopper->Preload(false);
            m_pIntake->Extend(false);
            m_pIntake->IntakeMotor(false);
            m_pIntake->RetentionMotor(false);
            m_pDrive->FollowTrajectory();

            // Move to the next state when the robot is done path following.
            if (m_pDrive->TrajectoryIsFinished())
            {
                m_nAutoState = eAutoOpposingTrench3;
            }
            break;

        case eAutoOpposingTrench3 : 
            // Shoot the 5 balls.
            if (fabs(m_pTimer->Get() - m_dStartTime) < 15.0)
            {
                // Set the Turret to tracking mode.
                m_pTurret->SetVision(true);
                // Enabled LEDs
                m_pShooter->SetVisionLED(true);
                // Set robot color.
                m_pBlinkin->SetState(m_pBlinkin->eLarsonScanner1);
                // Set the Hood to tracking mode.
                m_pHood->SetSetpoint(SmartDashboard::GetNumber("Target Distance", 0.0));
                // Set the Shooter setpoint.
                m_pShooter->SetSetpoint(dShooterFiringVelocity);
                // Start shooting when ready.
                if (m_pShooter->IsAtSetpoint())
                {
                    // Start preloading into the shooter.
                    m_pHopper->Feed(true);
                    m_pHopper->Preload(true);
                    m_pIntake->RetentionMotor(true);
                }
            }
            else
            {
                // Move to auto idle.
                m_nAutoState = eAutoIdle;
            }
            break;

        case eAutoPowerPort1 :
            // Fire the 3 balls.
            if (fabs(m_pTimer->Get() - m_dStartTime) < 6.00)
            {
                // Set the Turret to tracking mode.
                m_pTurret->SetVision(true);
                // Enabled LEDs
                m_pShooter->SetVisionLED(true);
                // Set robot color.
                m_pBlinkin->SetState(m_pBlinkin->eLarsonScanner1);
                // Use vision to set the hood angle.
                m_pHood->SetSetpoint(SmartDashboard::GetNumber("Target Distance", 0.0));
                // Set the Shooter setpoint.
                m_pShooter->SetSetpoint(4000);
                // Start shooting when ready.
                if (m_pShooter->IsAtSetpoint())
                {
                    // Start preloading into the shooter.
                    m_pHopper->Feed(true);
                    m_pHopper->Preload(true);
                    m_pIntake->RetentionMotor(true);
                }
            }
            else
            {
                m_pDrive->SetSelectedTrajectory(ePowerPort1);
                m_nAutoState = eAutoPowerPort2;
            }
            break;

        case eAutoPowerPort2 :
            // Follow the path to the power port.
            m_pDrive->FollowTrajectory();

            if (m_pDrive->TrajectoryIsFinished())
            {
                m_nAutoState = eAutoIdle;
            }
            break;
    }

    SmartDashboard::PutNumber("Robot Timer", m_pTimer->Get());
    SmartDashboard::PutNumber("Elapsed Time", fabs(m_pTimer->Get() - m_dStartTime));
    SmartDashboard::PutNumber("Start Time", m_dStartTime);

    // Call all subsystem Ticks.
    m_pDrive->Tick();
    m_pShooter->Tick();
    m_pTurret->Tick();
    m_pHood->Tick();
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

    // Enable hood safety.
    m_pHood->SetHoodSafety(true);

    // Clean-up from auto.
    m_nTeleopState = eTeleopStopped;
    m_pDrive->Init();
    m_pIntake->Init();
    m_pHood->Init();
    m_pTurret->Init();
    m_pShooter->Init();
    m_pLift->Init();
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
        Drive Controller - Toggle Intake (Right Bumper)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonRB))
    {
        if (m_nTeleopState == eTeleopIntake)
        {
            // Leave to idle.
            m_nTeleopState = eTeleopStopped;
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
            m_pShooter->SetState(eShooterIdle);
            m_nTeleopState = eTeleopStopped;
            bHasFired = false;
        }
    }

    /********************************************************************
        Drive Controller - AutoFire (Right Trigger + Aux Right Trigger)
    ********************************************************************/
    if ((m_pDriveController->GetRawAxis(eRightTrigger) >= 0.65) && (m_pAuxController->GetRawAxis(eRightTrigger) >= 0.65))
    {
        // Set the state to AutoFire.
        m_nTeleopState = eTeleopAutoFiring;
    }
    // Other states related to this button will set state back to Idle.

    /********************************************************************
        Drive Controller - Manual Move Turret Left (Left Bumper)
    ********************************************************************/
    if (m_pAuxController->GetRawButton(eButtonLB))
    {
        // Manually move left.
        m_pTurret->SetState(eTurretManualRev);
        bTurretMoving = true;
    }
    else
    {
    /********************************************************************
        Drive Controller - Manual Move Turret Right (Right Bumper)
    ********************************************************************/
        if (m_pAuxController->GetRawButton(eButtonRB))
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
        Drive Controller - Manual Move Hood Up (Up POV)
    ********************************************************************/
    if (m_pAuxController->GetPOV() == 0)
    {
        // Manual move up.
        m_pHood->SetState(eHoodManualFwd);
        bHoodMoving = true;
    }
    else
    {
    /********************************************************************
        Drive Controller - Manual Move Hood Down (Down POV)
    ********************************************************************/
        if (m_pAuxController->GetPOV() == 180)
        {
            // Manual move down.
            m_pHood->SetState(eHoodManualRev);
            bHoodMoving = true;
        }
        else
        {
            if (bHoodMoving)
            {
                // No longer moving, set to idle.
                m_pHood->Stop();
                bHoodMoving = false;
            }
        }
    }

    /********************************************************************
        Aux Controller - Unjam belts (Back)
    ********************************************************************/
    if (m_pAuxController->GetRawButtonPressed(eBack))
    {
        // Reverse the belts.
        m_pHopper->Unjam(true);
    }
    if (m_pAuxController->GetRawButtonReleased(eBack))
    {
        // Stop the belts.
        m_pHopper->Unjam(false);
    }
    
    /********************************************************************
        Aux Controller - Vision Aiming (Left Trigger)
    ********************************************************************/
    if ((m_pAuxController->GetRawAxis(eRightTrigger) > 0.65) && !(m_pDriveController->GetRawAxis(eRightTrigger) > 0.65))
    {
        // Set state to Aiming.
        m_nTeleopState = eTeleopAiming;
    }
    if (!(m_pAuxController->GetRawAxis(eRightTrigger) > 0.65) && !(m_pDriveController->GetRawAxis(eRightTrigger) > 0.65))
    {
        // If released while still in aiming...
        if (m_nTeleopState == eTeleopAiming)
        {
            // Go back to idle.
            m_nTeleopState = eTeleopStopped;
        }
        // If the button was released but we didn't change states
        // yet, do nothing to prevent it from leaving it's current
        // state.
    }

    /********************************************************************
        Aux Controller - Toggle Shooter "Idle" speed (Button B)
    ********************************************************************/
    if (m_pAuxController->GetRawButtonPressed(eButtonB))
    {
        m_pShooter->SetState(m_pShooter->GetState() == eShooterIdle ? eShooterStopped : eShooterIdle);
    }

    /********************************************************************
        Aux Controller - Bump Retention backwards (Button A)
    ********************************************************************/
    if (m_pAuxController->GetRawButtonPressed(eButtonA))
    {
        // Unjam the intake.
        m_pIntake->Unjam(true);
    }
    if (m_pAuxController->GetRawButtonReleased(eButtonA))
    {
        // Stop the Unjamming sequence.
        m_pIntake->Unjam(false);
    }

    /********************************************************************
        Aux Controller - Zero Hood Encoder (Button X)
    ********************************************************************/
    if (m_pAuxController->GetRawButtonPressed(eButtonX))
    {
        m_pHood->Rezero();
    }

    /********************************************************************
        Aux Controller - Right Winch Up (Left Stick Y)
    ********************************************************************/
    if (m_pAuxController->GetRawAxis(eLeftAxisY) > 0.2)
    {
        m_pLift->TestRightWinch(-1.0);
    }
    else
    {
    /********************************************************************
        Aux Controller - Right Winch Down (Left Stick Y)
    ********************************************************************/
        if (m_pAuxController->GetRawAxis(eLeftAxisY) < -0.2)
        {
            m_pLift->TestRightWinch(1.0);
        }
        else
        {
            m_pLift->TestRightWinch(0.0);
        }
    }
    
    /********************************************************************
        Aux Controller - Left Winch Up (Left Stick Y)
    ********************************************************************/
    if (m_pAuxController->GetRawAxis(eLeftAxisY) > 0.2)
    {
        m_pLift->TestLeftWinch(-1.0);
    }
    else
    {
    /********************************************************************
        Aux Controller - Left Winch Down (Left Stick Y)
    ********************************************************************/
        if (m_pAuxController->GetRawAxis(eLeftAxisY) < -0.2)
        {
            m_pLift->TestLeftWinch(1.0);
        }
        else
        {
            m_pLift->TestLeftWinch(0.0);
        }
    }

    /********************************************************************
        Aux Controller - Actuate Lift (Start Button)
    ********************************************************************/
    if (m_pAuxController->GetRawButtonPressed(eStart))
    {
        m_pLift->ExtendArm(!m_pLift->IsExtended());
    }

    switch(m_nTeleopState)
    {
        case eTeleopStopped :
            /********************************************************************
                Stopped - Robot is not currently doing anything. Reset everything.
                          May or may not be driving as well.
            ********************************************************************/
            // Disable LEDs
            m_pShooter->SetVisionLED(false);
            m_pTurret->SetVision(false);
            // Return intake to it's retracted state.
            m_pIntake->Extend(false);
            m_pIntake->IntakeMotor(false);
            m_pIntake->RetentionMotor(false);
            // Return Lift arm to it's lower position.
            // m_pLift->ExtendArm(false);
            // Idle the arm.
            // m_pLift->ReverseIdle(true);
            // Idle the Hood, Turret, and Hopper.
            m_pHood->SetState(eHoodReset);
            // m_pTurret->Stop();
            m_pHopper->Feed(false);
            m_pHopper->Preload(false);
            // Set robot color.
            m_pBlinkin->SetState(m_pBlinkin->eTwinkle);
            // Move back to TeleopIdle.
            m_nTeleopState = eTeleopIdle;
            break;
        
        case eTeleopIdle :
            /********************************************************************
                Idle - Robot is currently Idling, nothing is returning to rested
                       state either.
            ********************************************************************/
            // Do nothing.
            break;

        case eTeleopIntake :
            /********************************************************************
                Intake - Robot is intaking Energy.
            ********************************************************************/
            // Disable LEDs
            m_pShooter->SetVisionLED(false);
            m_pTurret->SetVision(false);
            // Return Lift arm to it's lower position.
            // m_pLift->ExtendArm(false);
            // Idle the arm.
            // m_pLift->ReverseIdle(true);
            // Extend intake.
            m_pIntake->Extend(true);
            // Start intake on a half second delay.
            if ((m_pTimer->Get() - m_dStartTime) >= 0.5)
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
            // m_pLift->ExtendArm(false);
            // Idle the arm.
            // m_pLift->ReverseIdle(true);
            // Set the Turret to tracking mode.
            m_pTurret->SetVision(true);
            // Enabled LEDs
            m_pShooter->SetVisionLED(true);
            // Idle shooter.
            m_pShooter->SetSetpoint(dShooterIdleVelocity);
            // Stop Preloader.
            m_pHopper->Preload(false);
            m_pHopper->Feed(false);
            // Set the Hood to tracking mode.
            m_pHood->SetSetpoint(SmartDashboard::GetNumber("Target Distance", 0.0));
            // Set robot color.
            m_pBlinkin->SetState(m_pBlinkin->eLarsonScanner1);
            break;
        
        case eTeleopFiring :
            /********************************************************************
                Firing - Robot simply fires wherever it is currently aiming.
            ********************************************************************/
            // Return Lift arm to it's lower position.
            // m_pLift->ExtendArm(false);
            // Idle the arm.
            // m_pLift->ReverseIdle(true);
            // Enabled LEDs
            m_pShooter->SetVisionLED(false);
            m_pTurret->SetVision(false);
            // Set the Turret to idle, we don't want it to move.
            m_pTurret->SetState(eTurretIdle);
            // Set the Shooter to firing speed.
            m_pShooter->SetSetpoint(dShooterFiringVelocity);
            if (m_pShooter->IsAtSetpoint())
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
            // m_pLift->ExtendArm(false);
            // Idle the arm.
            // m_pLift->ReverseIdle(true);
            // Enabled LEDs
            m_pShooter->SetVisionLED(true);
            m_pTurret->SetVision(true);
            // Set the Turret to Tracking mode.
            m_pTurret->SetState(eTurretTracking);
            // Set the Shooter to firing speed.
            m_pShooter->SetSetpoint(dShooterFiringVelocity);
            if (m_pShooter->IsAtSetpoint()) 
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
            // // Stop idling the arm.
            // m_pLift->ReverseIdle(false);
            // // Disable LEDs
            // m_pShooter->SetVisionLED(false);
            // // Start the Lift state machine.
            // m_pLift->SetState(eLiftExtend);
            // // Set robot color.
            // m_pBlinkin->SetState(m_pBlinkin->eOrange);
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
    m_pHood->Tick();
    m_pTurret->Tick();
    m_pShooter->Tick();

    SmartDashboard::PutNumber("Shooter At Setpoint", m_pShooter->IsAtSetpoint());
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

    // disable hood safety.
    m_pHood->SetHoodSafety(false);
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
        m_pLift->TestRightWinch(-0.5);
    }
    else
    {
    /********************************************************************
        Drive Controller - Right Winch Down (Button A)
    ********************************************************************/
        if (m_pDriveController->GetRawButton(eButtonA))
        {
            m_pLift->TestRightWinch(0.5);
        }
        else
        {
            m_pLift->TestRightWinch(0.0);
        }
    }
    
    /********************************************************************
        Drive Controller - Left Winch Up (POV Up)
    ********************************************************************/
    if (m_pDriveController->GetPOV() == 0)
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
            m_pShooter->SetSetpoint(dShooterFiringVelocity);
        }
        else
        {
            m_pShooter->SetSetpoint(dShooterIdleVelocity);
        }
    }
    else
    {
    /********************************************************************
        Drive Controller - Shooter Reverse (Left Trigger)
    ********************************************************************/
        if (m_pDriveController->GetRawAxis(eLeftTrigger) >= 0.65)
        {
            m_pShooter->SetState(eShooterManualRev);
        }
        else
        {
            m_pShooter->SetState(eShooterStopped);
        }
    }

    /********************************************************************
        Drive Controller - Hood Up (Right Bumper)
    ********************************************************************/
    if (m_pDriveController->GetRawButton(eButtonRB))
    {
        m_pHood->SetState(eHoodManualFwd);
    }
    else
    {
    /********************************************************************
        Drive Controller - Hood Down (Left Bumper)
    ********************************************************************/
        if (m_pDriveController->GetRawButton(eButtonLB))
        {
            m_pHood->SetState(eHoodManualRev);
        }
        else
        {
            m_pHood->SetState(eHoodStopped);
        }
    }

    /********************************************************************
        Drive Controller - Actuate Lift (Right Stick)
    ********************************************************************/
    if (m_pDriveController->GetRawButtonPressed(eButtonRS))
    {
        m_pLift->ExtendArm(!m_pLift->IsExtended());
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
    m_pHood->Tick();
    m_pLift->SetState(999);
    m_pLift->Tick();
}
///////////////////////////////////////////////////////////////////////////////
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<CRobotMain>(); }
#endif