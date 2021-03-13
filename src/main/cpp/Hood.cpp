/****************************************************************************
    Description:	Implements the CHood control class.
    Classes:		CHood
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include "Hood.h"

using namespace frc;
/////////////////////////////////////////////////////////////////////////////

/****************************************************************************
    Description:	CHood Constructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CHood::CHood()
{
    // Create object pointers.
    m_pHoodMotor        = new CANSparkMax(nHoodMotor, CANSparkMax::MotorType::kBrushless);
    m_pHoodPID			= new frc2::PIDController(dHoodProportional, dHoodIntegral, dHoodDerivative);
    m_pTimer            = new Timer();

    // Initialize member variables.
    m_pHoodMotor->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_pHoodPID->SetIntegratorRange(-0.35, 0.35);
    m_pHoodMotor->SetOpenLoopRampRate(0.0);
    m_pHoodMotor->DisableVoltageCompensation();
    m_dProportional			= dHoodProportional;
    m_dIntegral				= dHoodIntegral;
    m_dDerivative			= dHoodDerivative;
    m_dTolerance			= dHoodTolerance;
    m_dSetpoint				= 0.0;
    m_dActual				= GetActual();
    m_dMaxFindingTime		= dHoodFindingTime;
    m_dFindingStartTime		= 0.0;
    m_nState				= eHoodIdle;
    m_bIsReady              = false;
    m_bHoodSafety           = true;

    // Ensure the Hood doesn't wrap around.
    m_pHoodPID->DisableContinuousInput();
}

/****************************************************************************
    Description:	CHood Destructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CHood::~CHood()
{
    // Delete objects.
    delete m_pHoodMotor;
    delete m_pHoodPID;
    delete m_pTimer;

    // Set objects to nullptrs.
    m_pHoodMotor		= nullptr;
    m_pHoodPID			= nullptr;
    m_pTimer            = nullptr;
}

/****************************************************************************
    Description:	Init - Begins Initialization of the Subsystem
    Arguments:		None
    Returns:		Nothing
****************************************************************************/
void CHood::Init()
{
    SmartDashboard::PutNumber("Proportional", dHoodProportional);
    SmartDashboard::PutNumber("Integral", dHoodIntegral);
    SmartDashboard::PutNumber("Derivative", dHoodDerivative);
    SmartDashboard::PutNumber("Hood Position Far", dHoodPresetPositionFar);
    SmartDashboard::PutNumber("Hood Position Near", dHoodPresetPositionNear);

    // Set the state back to Idle.
    m_nState = eHoodStopped;
    // Set the Tolerance of the PID controller.
    SetTolerance(m_dTolerance);
    // Set the PID of the controller.
    SetPID(m_dProportional, m_dIntegral, m_dDerivative);
    // Stop the Hood.
    Stop();
    // Start the timer.
    m_pTimer->Start();
}

/****************************************************************************
    Description:	Tick - Implements functionality when called in a loop
    Arguments:		None
    Returns:		Nothing
****************************************************************************/
void CHood::Tick()
{
    // Set the PID
    SetPID(SmartDashboard::GetNumber("Proportional", dHoodProportional), SmartDashboard::GetNumber("Integral", dHoodIntegral), SmartDashboard::GetNumber("Derivative", dHoodDerivative));
    // Get the Hood actual position.
    m_dActual = m_pHoodMotor->GetEncoder().GetPosition() / (20.0 / 310.0) + dHoodMinPosition;
    // Hood state machine.
    switch(m_nState)
    {
        case eHoodIdle :
            // Idle - Servo is off and ready to move again.
            // Put state on SmartDashboard.
            SmartDashboard::PutString("Hood State", "Idle");
            m_pHoodPID->Reset();
            // Set servo to ready.
            m_bIsReady = true;
            break;

        case eHoodStopped :
            // Stopped - Stops the whole mechanism.
            // Put state on SmartDashboard.
            SmartDashboard::PutString("Hood State", "Stopped");
            // Stop the servo.
            SetSpeed(0.0);
            // Move to Idle.
            SetState(eHoodIdle);
            break;

        case eHoodReset :
            // Reset - Resets the mechanism back to zero.
            // Put the state on SmartDashboard.
            SmartDashboard::PutString("Hood State", "Reset");
            // Move to zero.
            SetSetpoint(0.0);
            break;
            
        case eHoodTracking :
            // Tracking - Utilizes Vision to find the setpoint of the hood.
            // Put state on SmartDashboard.
            SmartDashboard::PutString("Hood State", "Tracking");
            // Check if the distance has changed, which means we've found a target.
            SetSetpoint(SmartDashboard::GetNumber("Target Distance", 0.0));
            break;

        case eHoodFinding :
            // Finding - Use PID Controller to drive to a given
            // Setpoint, and check if we are within the given
            // tolerance.
            // Put state on SmartDashboard.
            SmartDashboard::PutString("Hood State", "Finding");
            if (fabs(m_pHoodPID->GetPositionError()) < m_dTolerance)
            {
                // At our setpoint, return to idle.
                SetState(eHoodStopped);
            }
            else
            {
                // Not at the setpoint, continue.
                SetSpeed(m_pHoodPID->Calculate(m_dActual));
            }
            break;

        case eHoodManualFwd :
            // ManualForward - Move the Hood forward at a constant speed.
            // Put state on SmartDashboard.
            SmartDashboard::PutString("Hood State", "Manual Fwd");
            SetSpeed(dHoodManualFwdSpeed);
            break;

        case eHoodManualRev :
            // ManualReverse - Move the Hood backwards at a constant speed.
            // Put state on SmartDashboard.
            SmartDashboard::PutString("Hood State", "Manual Rev");
            SetSpeed(dHoodManualRevSpeed);
            break;

        default :
            // Put state on SmartDashboard.
            SmartDashboard::PutString("Hood State", "ERROR");
            break;
    }

    SmartDashboard::PutNumber("Hood Setpoint", m_dSetpoint);
    SmartDashboard::PutNumber("Hood Actual", m_dActual);
    SmartDashboard::PutNumber("Hood Output", m_pHoodPID->Calculate(m_dActual));
}

/****************************************************************************
    Description:	Set the variables of the PID controller for position.
    Arguments: 		double dProportional
                    double dIntegral
                    double dDerivative
    Returns: 		Nothing
****************************************************************************/
void CHood::SetPID(double dProportional, double dIntegral, double dDerivative)
{
    // Configure PID controller to use the given values.
    m_pHoodPID->SetPID(dProportional, dIntegral, dDerivative);
}

/****************************************************************************
    Description:	Set and convert speed to the Hood angle in percent output.
    Arguments: 		double dSpeed - Speed in percent output
    Returns: 		Nothing
****************************************************************************/
void CHood::SetSpeed(double dSpeed)
{
    // Cap values.
    if (dSpeed > 1.0)
    {
        dSpeed = 1.0;
    }
    if (dSpeed < -1.0)
    {
        dSpeed = -1.0;
    }

    // Enable or Disable soft limits for the hood.
    if (m_bHoodSafety)
    {
        // Soft limits for the hood.
        if ((m_dActual <= dHoodMaxPosition) && (m_dActual >= dHoodMinPosition))
        {
            // Set the "Speed" of the continuous rotation servo.
            m_pHoodMotor->Set(dSpeed);
        }
        else
        {
            if ((m_dActual >= dHoodMaxPosition) && (dSpeed < 0.0))
            {
                // Set the "Speed" of the continuous rotation servo.
                m_pHoodMotor->Set(dSpeed);
            }
            else
            {
                if ((m_dActual <= dHoodMinPosition) && (dSpeed > 0.0))
                {
                    // Set the "Speed" of the continuous rotation servo.
                    m_pHoodMotor->Set(dSpeed);
                }
                else
                {
                    m_pHoodMotor->Set(0);
                }            
            }
        }
    }
    else
    {
        // Cap values.
        if (dSpeed > 1.0)
        {
            dSpeed = 1.0;
        }
        if (dSpeed < -1.0)
        {
            dSpeed = -1.0;
        }
        // Set the "Speed" of the continuous rotation servo.
        m_pHoodMotor->Set(dSpeed);
    }
    

    SmartDashboard::PutNumber("Hood Output", dSpeed);
}

/****************************************************************************
    Description:	Set the setpoint of the Hood's PID controller and move
                    to that position.
    Arguments: 		double dSetpoint - Units in degrees
    Returns: 		Nothing
****************************************************************************/
void CHood::SetSetpoint(double dSetpoint)
{
    // Set the member variable.
    m_dSetpoint = dSetpoint;

    // Give the PID controller a setpoint.
    m_pHoodPID->SetSetpoint(m_dSetpoint);

    // Start the timer for beginning finding.
    m_dFindingStartTime = m_pTimer->Get();

    // Set Hood state to finding.
    SetState(eHoodFinding);
}

/****************************************************************************
    Description:	Sets the tolerance to be used in PID controller.
    Arguments: 		double dSetpoint - Units in degrees
    Returns: 		Nothing
****************************************************************************/
void CHood::SetTolerance(double dTolerance)
{
    // Set member variable.
    m_dTolerance = dTolerance;
    m_pHoodPID->SetTolerance(m_dTolerance);
}

/****************************************************************************
    Description:	Stop - Stops the mechanism.
    Arguments:		None
    Returns:		Nothing
****************************************************************************/
void CHood::Stop()
{
    // Disable the PID controller to stop any movement.
    m_pHoodPID->Reset();
    // Set state to stopped.
    SetState(eHoodStopped);
}

/****************************************************************************
    Description:	Checks to see if the hood is within tolerance of the setpoint.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
bool CHood::IsAtSetpoint()
{
    return m_pHoodPID->AtSetpoint();
}

/****************************************************************************
    Description:	Rezero - Zero the encoder running the Hood.
    Arguments:		None
    Returns:		Nothing
****************************************************************************/
void CHood::Rezero()
{
    m_pHoodMotor->GetEncoder().SetPosition(0);
}
///////////////////////////////////////////////////////////////////////////////