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
    m_pHoodServo		= new Servo(nHoodServo);
    m_pHoodEncoder		= new Encoder(nHoodEncoderChannelA, nHoodEncoderChannelB);
    m_pHoodPID			= new frc2::PIDController(dHoodProportional, dHoodIntegral, dHoodDerivative);

    // Initialize member variables.
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
    delete m_pHoodServo;
    delete m_pHoodEncoder;
    delete m_pHoodPID;

    // Set objects to nullptrs.
    m_pHoodServo		= nullptr;
    m_pHoodEncoder		= nullptr;
    m_pHoodPID			= nullptr;
}

/****************************************************************************
    Description:	Init - Begins Initialization of the Subsystem
    Arguments:		None
    Returns:		Nothing
****************************************************************************/
void CHood::Init()
{
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
    // Get the Hood actual position.
    m_dActual	 = m_pHoodEncoder->Get() / (20.0 / 310.0) + dHoodMinPosition;
    // Hood state machine.
    switch(m_nState)
    {
        case eHoodIdle :
            // Idle - Servo is off and ready to move again.
            // Put state on SmartDashboard.
            SmartDashboard::PutString("Hood State", "Idle");
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
                SetState(eHoodIdle);
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
    // Convert a -1 -> 1 value to a 0 -> 1 value.
    double dFakeSpeed = dSpeed + 1.0;
    dFakeSpeed /= 2.0;
    // Set the "Speed" of the continuous rotation servo.
    m_pHoodServo->Set(dFakeSpeed);

    SmartDashboard::PutNumber("Servo Output", dFakeSpeed);
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
    SetState(eHoodIdle);
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
///////////////////////////////////////////////////////////////////////////////