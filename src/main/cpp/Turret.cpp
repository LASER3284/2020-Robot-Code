/****************************************************************************
	Description:	Implements the CTurret control class.
	Classes:		CTurret
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include "Turret.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
using namespace ctre;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
	Description:	CTurret Constructor.
	Arguments:		None
	Derived From:	Nothing
****************************************************************************/
CTurret::CTurret()
{
    // Create Object Pointers.
	m_pTurretMotor	= new WPI_TalonSRX(nTurretMotor);
	m_pTimer		= new Timer();
	m_pPIDController= new frc2::PIDController(1.0, 0.0, 0.0);
}

/****************************************************************************
	Description:	CTurret Destructor.
	Arguments:		None
	Derived From:	Nothing
****************************************************************************/
CTurret::~CTurret()
{
	// Delete objects.
	delete m_pTurretMotor;
	delete m_pTimer;
	delete m_pPIDController;
	// Set objects to nullptrs.
	m_pTurretMotor		= nullptr;
	m_pTimer			= nullptr;
	m_pPIDController	= nullptr;
}

/****************************************************************************
	Description:	Initialize Turret parameters.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CTurret::Init()
{
	// Initialize member variables.
	m_nState						= eTurretIdle;
	m_bIsReady						= true;
	m_bMotionMagic					= false;
	m_dActual						= (m_pTurretMotor->GetSensorCollection().GetPulseWidthRiseToFallUs() - 1024) / 8.0;
	m_dSetpoint						= 0.00;
	m_dTolerance					= 5.00;
	m_dMaxFindingTime				= 3.50;
	m_dFindingStartTime				= 0.00;

	// Set up the feedback device for an analog encoder.
	m_pTurretMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition);
	// Set encoder to not wrap around.
	m_pPIDController->DisableContinuousInput();
	// Set the encoder and motor as both positive.
	m_pTurretMotor->SetInverted(false);
	m_pTurretMotor->SetSensorPhase(false);
	// Set the peak (maximum) motor output for both directions.
	m_pTurretMotor->ConfigPeakOutputForward(1.00);
	m_pTurretMotor->ConfigPeakOutputReverse(-1.00);
	// Set the tolerance.
	SetTolerance(m_dTolerance);
	// Set the PID and feed forward values.
	SetPID(1.0, 0.0, 0.0);
	// Stop the motor.
	Stop();
	// Set the neutral mode to brake.
	m_pTurretMotor->SetNeutralMode(NeutralMode::Brake);
	// Set acceleration (seconds from neutral to full output).
	m_pTurretMotor->ConfigOpenloopRamp(dTurretOpenLoopRamp);

	// Clear the sticky faults in memory.
	m_pTurretMotor->ClearStickyFaults();

	// Start the timer.
	m_pTimer->Start();
}

/****************************************************************************
	Description:	Main method that calls functionality, to be used in a loop.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CTurret::Tick()
{
	// Update Actual variable.
	m_dActual = (m_pTurretMotor->GetSensorCollection().GetPulseWidthRiseToFallUs() - 1024) / 8.0;

	switch(m_nState)
	{
		case eTurretIdle :
			// Idle - Motor is off, and ready to move again.
			m_pPIDController->Reset();
			m_pTurretMotor->Set(ControlMode::PercentOutput, 0.00);
			m_bIsReady = true;
			break;

		case eTurretFinding :
			// Finding - Motor uses built-in PID controller to seek the
			// given Setpoint, and performs checks to ensure we are within
			// the given tolerance.
			if (IsAtSetpoint() || ((m_pTimer->Get() - m_dFindingStartTime) > m_dMaxFindingTime))
			{
				// Stop the motor, return to Idle.
				Stop();
			}
			else
			{
				// Not at setpoint, continue.
				m_pPIDController->Calculate(m_dActual);
			}
			break;

		case eTurretManualFwd :
			// ManualFwd - Manually move the motor forward at a constant speed.
			m_pTurretMotor->Set(ControlMode::PercentOutput, dTurretManualFwdSpeed);
			break;
		
		case eTurretManualRev :
			// ManualRev - Manually move the motor backwards at a constant speed.
			m_pTurretMotor->Set(ControlMode::PercentOutput, dTurretManualRevSpeed);
			break;

		default :
			m_nState = eTurretIdle;
			break;
	}

	SmartDashboard::PutNumber("Turret Position", m_dActual / dTurretRevsPerUnit / dTurretPulsesPerRev);
	SmartDashboard::PutNumber("Turret User Setpoint", m_dSetpoint * dTurretPulsesPerRev* dTurretRevsPerUnit);
	SmartDashboard::PutNumber("Turret Internal Setpoint", m_pPIDController->GetSetpoint());
	SmartDashboard::PutNumber("Turret Error", m_pPIDController->GetPositionError() / dTurretRevsPerUnit / dTurretPulsesPerRev);
}

/****************************************************************************
	Description:	Set the variables of the PID controller for position.
	Arguments: 		double dProportional
					double dIntegral
					double dDerivative
					double dFeedForward
	Returns: 		Nothing
****************************************************************************/
void CTurret::SetPID(double dProportional, double dIntegral, double dDerivative)
{
	// Configure PID controller to use the given values.
	m_pPIDController->SetPID(dProportional, dIntegral, dDerivative);
}

/****************************************************************************
	Description:	Set the setpoint of the turret's PID controller and move
					to that position.
	Arguments: 		double dSetpoint - Units in degrees
	Returns: 		Nothing
****************************************************************************/
void CTurret::SetSetpoint(double dSetpoint)
{
	// Check the bounds of the setpoint. Change if neccessary.
	if (dSetpoint > dTurretMaxPosition)
	{
		dSetpoint = dTurretMaxPosition;
	}
	if (dSetpoint < dTurretMinPosition)
	{
		dSetpoint = dTurretMinPosition;
	}

	// Set the member variable.
	m_dSetpoint = dSetpoint;

	m_pPIDController->SetSetpoint(m_dSetpoint * dTurretPulsesPerRev * dTurretRevsPerUnit);

	// Start the timer for beginning finding.
	m_dFindingStartTime = m_pTimer->Get();

	// Set Turret state to finding.
	SetState(eTurretFinding);

	std::cout << "Setpoint - " << m_dSetpoint << std::endl;
}

/****************************************************************************
	Description:	Set the State Machine to the given state.
	Arguments: 		TurretState nState - Given state in state machine
	Returns: 		Nothing
****************************************************************************/
void CTurret::SetState(TurretState nState)
{
	// Set member variable.
	m_nState = nState;
}

/****************************************************************************
	Description:	Sets the tolerance to be used in PID controller.
	Arguments: 		double dSetpoint - Units in degrees
	Returns: 		Nothing
****************************************************************************/
void CTurret::SetTolerance(double dTolerance)
{
	// Set member variable.
	m_dTolerance = dTolerance;
}

/****************************************************************************
	Description:	Stops the turret at it's position, resets state to idle.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CTurret::Stop()
{
	// Disable the PID controller to stop any movement.
	m_pPIDController->Reset();
	// Stop the motor.
	m_pTurretMotor->Set(ControlMode::PercentOutput, 0.00);
	// Set the state to idle.
	SetState(eTurretIdle);
}

/****************************************************************************
	Description:	Checks to see if Turret is within tolerance of the setpoint.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
bool CTurret::IsAtSetpoint()
{
	return ((fabs(m_pPIDController->GetPositionError() * dTurretRevsPerUnit * dTurretPulsesPerRev) <= m_dTolerance)
		     ? true : false);
}
/////////////////////////////////////////////////////////////////////////////
