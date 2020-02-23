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
    m_pHoodServo		= new Servo(nHoodServo);
    m_pLeftShooter		= new CANSparkMax(nShooterLeft, CANSparkMax::MotorType::kBrushless);
    m_pRightShooter		= new CANSparkMax(nShooterRight, CANSparkMax::MotorType::kBrushless);
    m_pShooterPID		= new CANPIDController(*m_pLeftShooter);
    m_pHoodEncoder		= new Encoder(nHoodEncoderChannelA, nHoodEncoderChannelB);
    m_pVisionSwitch     = new DigitalOutput(nVisionLEDChannel);
    m_pHoodPID			= new frc2::PIDController(1.0, 0.0, 0.0);
    m_pTimer			= new Timer();

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
    delete m_pHoodServo;
    delete m_pLeftShooter;
    delete m_pRightShooter;
    delete m_pTimer;
    delete m_pHoodEncoder;
    delete m_pVisionSwitch;
    delete m_pHoodPID;

    // Set objects to nullptrs.
    m_pHoodServo		= nullptr;
    m_pLeftShooter		= nullptr;
    m_pRightShooter		= nullptr;
    m_pTimer			= nullptr;
    m_pHoodEncoder		= nullptr;
    m_pVisionSwitch     = nullptr;
    m_pHoodPID			= nullptr;
}

/****************************************************************************
    Description:	Initialize Shooter parameters.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CShooter::Init()
{
    // Initialize variables.
    m_bMotionMagic				= false;
    m_dShooterProportional		= dShooterProportional;
    m_dShooterIntegral			= dShooterIntegral;
    m_dShooterDerivative		= dShooterDerivative;
    m_dShooterFeedForward		= dShooterFeedForward;
    m_dShooterTolerance			= dShooterTolerance;
    m_dShooterSetpoint			= 0.0;
    m_dShooterActual			= GetShooterActual();
    m_nShooterState				= eShooterStopped;
    m_bShooterIsReady			= false;

    m_dHoodProportional			= dHoodProportional;
    m_dHoodIntegral				= dHoodIntegral;
    m_dHoodDerivative			= dHoodDerivative;
    m_dHoodTolerance			= dHoodTolerance;
    m_dHoodSetpoint				= 0.0;
    m_dHoodActual				= GetHoodActual();
    m_dHoodMaxFindingTime		= dHoodFindingTime;
    m_dHoodFindingStartTime		= 0.0;
    m_nHoodState				= eHoodIdle;
    m_bHoodIsReady              = false;

    m_bIsReady					= IsReady();

    // Set the shooter motors inverted from each other.
    m_pLeftShooter->SetInverted(false);
    m_pRightShooter->SetInverted(true);
    // Set the peak (maximum) motor output for both controllers.
    m_pShooterPID->SetOutputRange(-1.0, 1.0);
    // Set the tolerances.
    SetShooterTolerance(m_dShooterTolerance);
    SetHoodTolerance(m_dHoodTolerance);
    // Set the PID and feed forward values.
    SetShooterPID(m_dShooterProportional, m_dShooterIntegral, m_dShooterDerivative, m_dShooterFeedForward);
    SetHoodPID(m_dHoodProportional, m_dHoodIntegral, m_dHoodDerivative);
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
    m_dShooterActual = m_pLeftShooter->GetEncoder().GetVelocity();
    m_dHoodActual	 = m_pHoodEncoder->GetDistance()  / (20.0 / 310.0) + dHoodMinPosition;

    // Shooter state machine.
    switch(m_nShooterState)
    {
        case eShooterStopped :
            // Stopped - Motor is off, and ready to move again.
            m_pLeftShooter->Set(0.00);
            m_dShooterSetpoint = 0;
            m_bIsReady = true;
            break;

        case eShooterIdle :
            // Idle - Motor is free spinning at a constant velocity to
            // reduce current draw.
            m_pShooterPID->SetReference(dShooterIdleVelocity, ControlType::kVelocity);
            m_dShooterSetpoint = dShooterIdleVelocity;
            m_bIsReady = true;
            break;

        case eShooterFinding :
            // Finding - Motor uses built-in PID controller to seek the
            // given Setpoint, and performs checks to ensure we are within
            // the given tolerance.
            // Move the motor to a given point.
            m_pShooterPID->SetReference(m_dShooterSetpoint, m_bMotionMagic ? kSmartVelocity : kVelocity);
            // Check to make sure it is or is not at setpoint.
            m_bShooterIsReady = (m_dShooterSetpoint - m_dShooterActual) <= m_dShooterTolerance;
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
            m_nShooterState = eShooterIdle;
            break;
    }

    // Hood state machine.
    switch(m_nHoodState)
    {
        case eHoodIdle :
            // Idle - Servo is off and ready to move again.
            // Stop the servo.
            SetHoodSpeed(0.0);
            // Set servo to ready.
            m_bHoodIsReady = true;
            break;

        case eHoodTracking :
            // Tracking - Utilizes Vision to find the setpoint of the hood.
            // Check if the distance has changed, which means we've found a target.
            // static int nCounter = 0;
            // static double dDistance = SmartDashboard::GetNumber("Target Distance", 0.0);
            // static double dTheta = 1.0;
            // static double dCalculatedAngle = 0.0;
            // while (dTheta - fabs(dCalculatedAngle) > 0.25)
            // {
            //     dCalculatedAngle = acos(((dDistance + (44.0 * 44.0) * (sin(dTheta) * sin(dTheta)) / 64.0)) * (32.0 / (44.0 * 44.0 * sin(dTheta))));
            //     dTheta = dCalculatedAngle;
            //     nCounter++;
            //     std::cout << "Iterations - " << nCounter << std::endl;
            // }

            // SmartDashboard::PutNumber("Theta", dTheta);
            // SmartDashboard::PutNumber("Calculated Angle", dCalculatedAngle);
            // nCounter = 0;
            // SetHoodSetpoint(dCalculatedAngle);
            break;

        case eHoodFinding :
            // Finding - Use PID Controller to drive to a given
            // Setpoint, and check if we are within the given
            // tolerance.
            // Set the new Proportional term.
            m_pHoodPID->SetP(m_dHoodProportional);
            if (m_dHoodSetpoint - m_dHoodActual < m_dHoodTolerance)
            {
                // At our setpoint, return to idle.
                SetHoodState(eHoodIdle);
            }
            else
            {
                // Not at the setpoint, continue.
                SetHoodSpeed(m_pHoodPID->Calculate(m_dHoodActual));
            }
            break;

        case eHoodManualFwd :
            // ManualForward - Move the Hood forward at a constant speed.
            SetHoodSpeed(dHoodManualFwdSpeed);
            break;

        case eHoodManualRev :
            // ManualReverse - Move the Hood backwards at a constant speed.
            SetHoodSpeed(dHoodManualRevSpeed);
            break;
    }

    SmartDashboard::PutNumber("Shooter Setpoint", m_dShooterSetpoint);
    SmartDashboard::PutNumber("Shooter Actual", m_dShooterActual);
    SmartDashboard::PutNumber("Hood Setpoint", m_dHoodSetpoint);
    SmartDashboard::PutNumber("Hood Actual", m_dHoodActual);
}

/****************************************************************************
    Description:	Set the variables of the PID controller for position.
    Arguments: 		double dProportional
                    double dIntegral
                    double dDerivative
                    double dFeedForward
    Returns: 		Nothing
****************************************************************************/
void CShooter::SetShooterPID(double dProportional, double dIntegral, double dDerivative, double dFeedForward)
{
    // Configure PID controller to use the given values.
    m_pShooterPID->SetP(dProportional);
    m_pShooterPID->SetI(dIntegral);
    m_pShooterPID->SetD(dDerivative);
    m_pShooterPID->SetFF(dFeedForward);
}

/****************************************************************************
    Description:	Set the variables of the PID controller for position.
    Arguments: 		double dProportional
                    double dIntegral
                    double dDerivative
    Returns: 		Nothing
****************************************************************************/
void CShooter::SetHoodPID(double dProportional, double dIntegral, double dDerivative)
{
    // Configure PID controller to use the given values.
    m_pHoodPID->SetPID(dProportional, dIntegral, dDerivative);
}

/****************************************************************************
    Description:	Set the setpoint of the Shooter's PID controller and move
                    to that position.
    Arguments: 		double dSetpoint - Units in degrees
    Returns: 		Nothing
****************************************************************************/
void CShooter::SetShooterSetpoint(double dSetpoint)
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
    m_dShooterSetpoint = dSetpoint;

    // Give the PID controller a setpoint.
    if (m_bMotionMagic)
    {
        m_pShooterPID->SetReference(m_dShooterSetpoint, ControlType::kSmartVelocity);
    }
    else
    {
        m_pShooterPID->SetReference(m_dShooterSetpoint, ControlType::kVelocity);
    }
    

    // Start the timer for beginning finding.
    m_dShooterFindingStartTime = m_pTimer->Get();

    // Set Shooter state to finding.
    SetShooterState(eShooterFinding);
}

/****************************************************************************
    Description:	Set and convert speed to the Hood angle in percent output.
    Arguments: 		double dSpeed - Speed in percent output
    Returns: 		Nothing
****************************************************************************/
void CShooter::SetHoodSpeed(double dSpeed)
{
    // Convert a -1 -> 1 value to a 0 -> 1 value.
    double dFakeSpeed = dSpeed + 1.0;
    dFakeSpeed /= 2.0;
    // Set the "Speed" of the continuous rotation servo.
    m_pHoodServo->Set(dFakeSpeed);
}

/****************************************************************************
    Description:	Set the setpoint of the Hood's PID controller and move
                    to that position.
    Arguments: 		double dSetpoint - Units in degrees
    Returns: 		Nothing
****************************************************************************/
void CShooter::SetHoodSetpoint(double dSetpoint)
{
    // Check the bounds of the setpoint. Change if neccessary.
    if (dSetpoint > dHoodMaxPosition)
    {
        dSetpoint = dHoodMaxPosition;
    }
    if (dSetpoint < dHoodMinPosition)
    {
        dSetpoint = dHoodMinPosition;
    }

    // Set the member variable.
    m_dHoodSetpoint = dSetpoint;

    // Give the PID controller a setpoint.
    m_pHoodPID->SetSetpoint(m_dHoodSetpoint);

    // Start the timer for beginning finding.
    m_dHoodFindingStartTime = m_pTimer->Get();

    // Set Shooter state to finding.
    SetShooterState(eShooterFinding);
}

/****************************************************************************
    Description:	Sets the tolerance to be used in PID controller.
    Arguments: 		double dSetpoint - Units in degrees
    Returns: 		Nothing
****************************************************************************/
void CShooter::SetShooterTolerance(double dTolerance)
{
    // Set member variable.
    m_dShooterTolerance = dTolerance;
}

/****************************************************************************
    Description:	Sets the tolerance to be used in PID controller.
    Arguments: 		double dSetpoint - Units in degrees
    Returns: 		Nothing
****************************************************************************/
void CShooter::SetHoodTolerance(double dTolerance)
{
    // Set member variable.
    m_dHoodTolerance = dTolerance;
    m_pHoodPID->SetTolerance(m_dHoodTolerance);
}


/****************************************************************************
    Description:	Stops the Shooter at it's position, resets state to idle.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CShooter::Stop()
{
    // Disable the PID controller to stop any movement.
    m_pHoodPID->Reset();
    // Stop the motor.
    m_pShooterPID->SetReference(0.0, ControlType::kDutyCycle);
    // Set the state to idle.
    SetShooterState(eShooterStopped);
    SetHoodState(eHoodIdle);
}

/****************************************************************************
    Description:	Checks to see if the hood is within tolerance of the setpoint.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
bool CShooter::IsHoodAtSetpoint()
{
    return (m_pHoodPID->AtSetpoint());
}

/****************************************************************************
    Description:	Checks to see if Shooter is within tolerance of the setpoint.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
bool CShooter::IsShooterAtSetpoint()
{
    return (m_dShooterSetpoint - m_dShooterActual) <= m_dShooterTolerance;
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
