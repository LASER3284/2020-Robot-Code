/****************************************************************************
    Description:	Implements the CShooter control class.
    Classes:		CShooter
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include "Shooter.h"

using namespace frc;
using namespace rev;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
    Description:	CShooter Constructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CShooter::CShooter()
{
    // Create Object Pointers.
    m_pLeftShooter		= new CANSparkMax(nShooterLeft, CANSparkMax::MotorType::kBrushless);
    m_pRightShooter		= new CANSparkMax(nShooterRight, CANSparkMax::MotorType::kBrushless);
    m_pShooterPID		= new CANPIDController(*m_pLeftShooter);
    m_pVisionSwitch     = new DigitalOutput(nVisionLEDChannel);
    m_pTimer			= new Timer();

    // Initialize variables.
    m_bMotionMagic	    = false;
    m_dProportional		= dShooterProportional;
    m_dIntegral			= dShooterIntegral;
    m_dDerivative		= dShooterDerivative;
    m_dFeedForward		= dShooterFeedForward;
    m_dTolerance	    = dShooterTolerance;
    m_dSetpoint			= 0.0;
    m_dActual			= GetActual();
    m_nState			= eShooterStopped;
    m_bIsReady			= false;
    m_bIsReady			= IsReady();

    // Set Right shooter to follow Left shooter.
    m_pRightShooter->Follow(*m_pLeftShooter, true);
}

/****************************************************************************
    Description:	CShooter Destructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CShooter::~CShooter()
{
    // Delete objects.
    delete m_pLeftShooter;
    delete m_pRightShooter;
    delete m_pTimer;
    delete m_pVisionSwitch;

    // Set objects to nullptrs.
    m_pLeftShooter		= nullptr;
    m_pRightShooter		= nullptr;
    m_pTimer			= nullptr;
    m_pVisionSwitch     = nullptr;
}

/****************************************************************************
    Description:	Initialize Shooter parameters.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CShooter::Init()
{
    // Disable the LEDs
    m_pVisionSwitch->Set(false);
    // Set the shooter motors inverted from each other.
    m_pLeftShooter->SetInverted(false);
    m_pRightShooter->SetInverted(true);
    // Set the peak (maximum) motor output for both controllers.
    m_pShooterPID->SetOutputRange(-1.0, 1.0);
    // Set the tolerances.
    SetTolerance(m_dTolerance);
    // Set the PID and feed forward values.
    SetPID(m_dProportional, m_dIntegral, m_dDerivative, m_dFeedForward);
    // Stop the mechanism.
    Stop();
    // Set the neutral mode of the Shooter to coast.
    m_pLeftShooter->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_pRightShooter->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    // Set acceleration (seconds from neutral to full output).
    m_pLeftShooter->SetClosedLoopRampRate(dShooterClosedLoopRamp);
    m_pRightShooter->SetClosedLoopRampRate(dShooterClosedLoopRamp);
    // Clear any faults in memory.
    m_pLeftShooter->ClearFaults();
    m_pRightShooter->ClearFaults();
    // Turn off the LEDs.
    m_pVisionSwitch->Set(false);
    // Start the timer.
    m_pTimer->Start();
}

/****************************************************************************
    Description:	Main method that calls functionality, to be used in a loop.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CShooter::Tick()
{
    // Update Actual variables.
    m_dActual = m_pLeftShooter->GetEncoder().GetVelocity();

    // Shooter state machine.
    switch(m_nState)
    {
        case eShooterStopped :
            // Stopped - Motor is off, and ready to move again.
            m_pLeftShooter->Set(0.00);
            m_dSetpoint = 0;
            m_bIsReady = true;
            break;

        case eShooterIdle :
            // Idle - Motor is free spinning at a constant velocity to
            // reduce current draw.
            m_pShooterPID->SetReference(dShooterIdleVelocity, ControlType::kVelocity);
            m_dSetpoint = dShooterIdleVelocity;
            m_bIsReady = true;
            break;

        case eShooterFinding :
            // Finding - Motor uses built-in PID controller to seek the
            // given Setpoint, and performs checks to ensure we are within
            // the given tolerance.
            // Move the motor to a given point.
            m_pShooterPID->SetReference(m_dSetpoint, m_bMotionMagic ? kSmartVelocity : kVelocity);
            // Check to make sure it is or is not at setpoint.
            m_bIsReady = (m_dSetpoint - m_dActual) <= m_dTolerance;
            break;

        case eShooterManualFwd :
            // ManualFwd - Manually move the motor forward at a constant speed.
            m_pLeftShooter->Set(dShooterManualFwdSpeed);
            break;
        
        case eShooterManualRev :
            // ManualRev - Manually move the motor backwards at a constant speed.
            m_pLeftShooter->Set(dShooterManualRevSpeed);
            break;

        default :
            m_nState = eShooterIdle;
            break;
    }

    SmartDashboard::PutNumber("Shooter Setpoint", m_dSetpoint);
    SmartDashboard::PutNumber("Shooter Actual", m_dActual);
}

/****************************************************************************
    Description:	Set the variables of the PID controller for position.
    Arguments: 		double dProportional
                    double dIntegral
                    double dDerivative
                    double dFeedForward
    Returns: 		Nothing
****************************************************************************/
void CShooter::SetPID(double dProportional, double dIntegral, double dDerivative, double dFeedForward)
{
    // Configure PID controller to use the given values.
    m_pShooterPID->SetP(dProportional);
    m_pShooterPID->SetI(dIntegral);
    m_pShooterPID->SetD(dDerivative);
    m_pShooterPID->SetFF(dFeedForward);
}

/****************************************************************************
    Description:	Set the setpoint of the Shooter's PID controller and move
                    to that position.
    Arguments: 		double dSetpoint - Units in degrees
    Returns: 		Nothing
****************************************************************************/
void CShooter::SetSetpoint(double dSetpoint)
{
    // Check the bounds of the setpoint. Change if neccessary.
    if (dSetpoint > dShooterMaxVelocity)
    {
        dSetpoint = dShooterMaxVelocity;
    }
    if (dSetpoint < dShooterMinVelocity)
    {
        dSetpoint = dShooterMinVelocity;
    }

    // Set the member variable.
    m_dSetpoint = dSetpoint;

    // Give the PID controller a setpoint.
    if (m_bMotionMagic)
    {
        m_pShooterPID->SetReference(m_dSetpoint, ControlType::kSmartVelocity);
    }
    else
    {
        m_pShooterPID->SetReference(m_dSetpoint, ControlType::kVelocity);
    }
    

    // Start the timer for beginning finding.
    m_dFindingStartTime = m_pTimer->Get();

    // Set Shooter state to finding.
    SetState(eShooterFinding);
}

/****************************************************************************
    Description:	Sets the tolerance to be used in PID controller.
    Arguments: 		double dSetpoint - Units in degrees
    Returns: 		Nothing
****************************************************************************/
void CShooter::SetTolerance(double dTolerance)
{
    // Set member variable.
    m_dTolerance = dTolerance;
}

/****************************************************************************
    Description:	Stops the Shooter at it's position, resets state to idle.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CShooter::Stop()
{
    // Stop the motor.
    m_pShooterPID->SetReference(0.0, ControlType::kCurrent);
    m_pLeftShooter->StopMotor();
    // Set the state to idle.
    SetState(eShooterStopped);
}

/****************************************************************************
    Description:	Checks to see if Shooter is within tolerance of the setpoint.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
bool CShooter::IsAtSetpoint()
{
    return (m_dSetpoint - m_dActual) < m_dTolerance;
}

/****************************************************************************
    Description:	Sets the state of the Vision LEDs
    Arguments: 		bool - True to enable.
    Returns: 		Nothing
****************************************************************************/
void CShooter::SetVisionLED(bool bEnabled)
{
    m_pVisionSwitch->Set(bEnabled);
}
/////////////////////////////////////////////////////////////////////////////
