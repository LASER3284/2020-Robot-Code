/******************************************************************************
	Description:	Implements the CSparkMotion control class.
	Classes:		CSparkMotion
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 FIRST Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#include "SparkMotion.h"

using namespace frc;
using namespace rev;
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
	Description:	CSparkMotion Constructor.
	Arguments:		int nDeviceID - CAN Bus Device ID
	Derived From:	Nothing
******************************************************************************/
CSparkMotion::CSparkMotion(int nDeviceID)
{
	m_nDeviceID = nDeviceID;
	// Create the object pointers.
	m_pMotor						= new CANSparkMax(nDeviceID, CANSparkMax::MotorType::kBrushless);
	m_pTimer						= new Timer();

	// Initialize member variables.
	m_nCurrentState					= eIdle;
	m_bReady						= true;
	m_bFwdLimitSwitchNormallyOpen	= true;
	m_bRevLimitSwitchNormallyOpen	= true;
	m_bHomingComplete				= false;
	m_bBackOffHome					= true;
	m_bMotionMagic					= false;
	m_dSetpoint						= 0.000;
	m_nPulsesPerRev					= nDefaultSparkMotionPulsesPerRev;
	m_dRevsPerUnit					= dDefaultSparkMotionRevsPerUnit;
	m_dFwdMoveSpeed					= dDefualtSparkMotionManualFwdSpeed;
	m_dRevMoveSpeed					= dDefualtSparkMotionManualRevSpeed;
	m_dFwdHomeSpeed					= dDefaultSparkMotionFwdHomeSpeed;
	m_dRevHomeSpeed					= dDefaultSparkMotionRevHomeSpeed;
	m_dTolerance					= dDefaultSparkMotionTolerance;
	m_dLowerSoftLimit				= dDefaultSparkMotionLowerSoftLimit;
	m_dUpperSoftLimit				= dDefaultSparkMotionUpperSoftLimit;
	m_dIZone						= dDefaultSparkMotionIZone;
	m_dMaxHomingTime				= dDefaultSparkMotionMaxHomingTime;
	m_dMaxFindingTime				= dDefaultSparkMotionMaxFindingTime;
	m_dHomingStartTime				= 0.000;
	m_dFindingStartTime				= 0.000;

	// Reset the encoder count to zero.
	ResetEncoderPosition();
	// Set the motor as positive.
	SetMotorInverted(false);
	// Set up the nominal motor output for both directions.
	SetNominalOutputVoltage(0.000, 0.000);
	// Set the peak (maximum) motor output for both directions.
	SetPeakOutputPercent(1.000, -1.000);
	// Set the tolerance.
	SetTolerance(m_dTolerance);
	// Set the PID and feed forward values.
	SetPIDValues(dDefaultSparkMotionProportional, dDefaultSparkMotionIntegral, dDefaultSparkMotionDerivative, dDefaultSparkMotionFeedForward);
	// Stop the motor.
	Stop();
	// Set the neutral mode to brake.
	m_pMotor->SetIdleMode(CANSparkMax::IdleMode::kBrake);
	// Disable both forward and reverse limit switches.
	m_pMotor->GetForwardLimitSwitch(CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);	
	m_pMotor->GetReverseLimitSwitch(CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);	
	// Set acceleration (seconds from neutral to full output).
	SetOpenLoopRampRate(dDefaultSparkMotionVoltageRampRate);
	SetClosedLoopRampRate(dDefaultSparkMotionVoltageRampRate);
	// Clear the sticky faults in memory.
	ClearStickyFaults();
	// Set the Integral Zone. Accumulated integral is reset to zero when the error exceeds this value.
	SetAccumIZone(m_dIZone);
	// Clear the sticky faults in memory.
	ClearStickyFaults();

	// Start the timer.
	m_pTimer->Start();
}

/******************************************************************************
	Description:	CSparkMotion Destructor.
	Arguments:		None
	Derived From:	Nothing
******************************************************************************/
CSparkMotion::~CSparkMotion()
{
	// Delete our object pointers.
	delete	m_pMotor;
	delete	m_pTimer;

	// Set the objects to NULL.
	m_pMotor	= nullptr;
	m_pTimer	= nullptr;
}

/******************************************************************************
	Description:	Tick - main method that does functionality.
					Called each time through robot main loop to update state.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::Tick()
{
	// State machine
	switch(m_nCurrentState)
	{
		case eIdle :
			// Stop the motor.
			m_pMotor->GetPIDController().SetReference(0, ControlType::kVoltage);
			m_bReady = true;
			break;

		case eHomingReverse :
			// If the state is eHomingReverse, the motor will move toward
			// the home switch, and then turn off and go to eHomingForward.
			m_bReady = false;

			// Check to see if the home limit is pressed or if we have exceeded the maximum homing time.
			if ((IsRevLimitSwitchPressed()) ||
				((m_dMaxHomingTime > 0.000) && (m_pTimer->Get() > (m_dHomingStartTime + m_dMaxHomingTime))))
			{
				// At the home limit switch, turn off the motor.
				m_pMotor->GetPIDController().SetReference(0, ControlType::kVoltage);
				if (m_bBackOffHome)
				{
					// Set the state to eHomingForward.
					m_nCurrentState = eHomingForward;
				}
				else
				{
					// Reset the encoder to zero.
//TODO: Find equivalent of this! Very Important!
//					m_pMotor->SetSelectedSensorPosition(0);
					// Stop the motor and change the control mode to position.
					m_pMotor->GetPIDController().SetReference(0, ControlType::kVoltage);
					// Set flag that homing is complete.
					m_bHomingComplete = true;
					// Move to idle.
					m_nCurrentState = eIdle;
				}
			}
			else
			{
				// Not yet at the home limit switch, keep moving.
				m_pMotor->GetPIDController().SetReference(m_dRevHomeSpeed, ControlType::kDutyCycle);
			}
			break;

		case eHomingForward :
			// If the state is eHomingForward, the motor will slowly
			// move (forward) off the limit switch. Once the switch releases,
			// the motor will stop and the encoder will be reset.
			m_bReady = false;

			// Check to see we are off the home limit switch or the homing timeout has been reached.
			if ((!IsRevLimitSwitchPressed()) ||
				((m_dMaxHomingTime > 0.000) && (m_pTimer->Get() > (m_dHomingStartTime + m_dMaxHomingTime))))
			{
				// Reset the encoder to zero.
//TODO: Find equivalent of this! Very Important!
//				m_pMotor->SetSelectedSensorPosition(0);
				// Stop the motor and change the control mode to position.
				m_pMotor->GetPIDController().SetReference(0, ControlType::kPosition);
				// Set flag that homing is complete.
				m_bHomingComplete = true;
				// Set the state to eIdle.
				m_nCurrentState = eIdle;
			}
			else
			{
				// Still on the home limit switch, keep moving.
				m_pMotor->GetPIDController().SetReference(m_dFwdHomeSpeed, ControlType::kDutyCycle);
			}
			break;

		case eFinding :
			// If the state is eFinding, the motor will continue until
			// the PID reaches the target or until the limit switch in
			// the direction of travel is pressed. The state then becomes idle.
			m_bReady = false;
			// Check to see if position is within tolerance or limit switch
			// is activated in direction of travel.
			if (IsAtSetpoint() ||
			   (((GetSetpoint() > GetActual()) && IsFwdLimitSwitchPressed()) ||
			    ((GetSetpoint() < GetActual()) && IsRevLimitSwitchPressed()) ||
				((m_dMaxFindingTime > 0.000) && (m_pTimer->Get() > (m_dFindingStartTime + m_dMaxFindingTime)))))
			{
				// Stop the motor and set the current state to eIdle.
				Stop();
			}
			break;

		case eManualForward :
			if (!IsFwdLimitSwitchPressed())
			{
				// Manually move position forward.
				m_pMotor->GetPIDController().SetReference(m_dFwdMoveSpeed, ControlType::kDutyCycle);
				m_bReady = false;
			}
			else
			{
				// Change the state to eIdle.
				SetState(eIdle);
				m_bReady = true;
			}
			break;

		case eManualReverse :
			if (!IsRevLimitSwitchPressed())
			{
				// Manually move position backwards.
				m_pMotor->GetPIDController().SetReference(m_dRevMoveSpeed, ControlType::kDutyCycle);
				m_bReady = false;
			}
			else
			{
				// Change the state to eIdle.
				SetState(eIdle);
				m_bReady = true;
			}
			break;

		default :
			break;
	}
}

/******************************************************************************
	Description:	SetSetpoint - Sets the position for the motor.
	Arguments:	 	dPosition - The position to move to in desired units.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetSetpoint(double dPosition)
{
	// Clamp the new setpoint within soft limits.
	if (dPosition > m_dUpperSoftLimit)
	{
		dPosition = m_dUpperSoftLimit;
	}
	else
	{
		if (dPosition < m_dLowerSoftLimit)
		{
			dPosition = m_dLowerSoftLimit;
		}
	}

	// Set the setpoint member variable.
	m_dSetpoint = dPosition;

	// Get current time.
	m_dFindingStartTime = m_pTimer->Get();

	// Set the motor to the desired position.
	if (m_bMotionMagic)
	{
		m_pMotor->GetPIDController().SetReference((dPosition * m_dRevsPerUnit * m_nPulsesPerRev), ControlType::kSmartMotion);
	}
	else
	{
		m_pMotor->GetPIDController().SetReference((dPosition * m_dRevsPerUnit * m_nPulsesPerRev), ControlType::kPosition);
	}

    // Prints can slow down the processing time of the RoboRIO, so these are for debugging.
//	printf("CSparkMotion::SetSetpoint - Setpoint = %7.3f\n", dPosition);
//	printf("CSparkMotion::SetSetpoint - Revs Per Unit = %7.3f\n", m_dRevsPerUnit);

	// Set the state to eFinding.
	m_nCurrentState = eFinding;
}

/******************************************************************************
	Description:	GetSetpoint - Returns the current setpoint of the motor's
					PID in desired units of measure.
	Arguments:	 	None
	Returns: 		The setpoint of the motor's PID in desired units of measure.
******************************************************************************/
double CSparkMotion::GetSetpoint()
{
	return m_dSetpoint;
}

/******************************************************************************
	Description:	StartHoming - Initializes the homing sequence.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::StartHoming()
{
	// Stop the motor and set the control mode for percent output.
	m_pMotor->GetPIDController().SetReference(0, ControlType::kDutyCycle);

	// Get the homing start time.
	m_dHomingStartTime = m_pTimer->Get();

	// Set flag that homing is not complete.
	m_bHomingComplete = false;

	// Set the current state to eHomingReverse.
	m_nCurrentState = eHomingReverse;
}

/******************************************************************************
	Description:	Stop - Stop the motor.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::Stop()
{
	// Stop the motor.
	m_pMotor->GetPIDController().SetReference(0, ControlType::kDutyCycle);

	// Set the current state to eIdle.
	m_nCurrentState = eIdle;
}

/******************************************************************************
	Description:	SetTolerance - Sets the tolerance of the PID in desired
					units of measure.
	Arguments:	 	dValue - Tolerance in the desired units.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetTolerance(double dValue)
{
	// Set the member variable.
	m_dTolerance = dValue;

	// Set the allowed error for the PID. This is in quadrature pulses.
//TODO: Find equivalent for "non-Motion Magic" closed loop error.
	m_pMotor->GetPIDController().SetSmartMotionAllowedClosedLoopError(m_dTolerance * m_dRevsPerUnit * m_nPulsesPerRev);
}

/******************************************************************************
	Description:	GetTolerance - Returns the tolerance in the desired units.
	Arguments:	 	None
	Returns: 		dValue - Tolerance in the desired units.
******************************************************************************/
double CSparkMotion::GetTolerance()
{
	return m_dTolerance;
}

/******************************************************************************
	Description:	SetSoftLimits - Sets soft limits for minimum and maximum travel.
	Arguments:	 	dMinValue - Minimum travel distance.
					dMaxValue - Maximum travel distance.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetSoftLimits(double dMinValue, double dMaxValue)
{
	// Set the member variables.
	m_dLowerSoftLimit	= dMinValue;
	m_dUpperSoftLimit	= dMaxValue;
}

/******************************************************************************
	Description:	ConfigLimitSwitches - Sets up the limit switches as
					normally open or normally closed.
	Arguments:	 	bool bFwdLimit - True if normally open, false if normally closed.
					bool bRevLimit - True if normally open, false if normally closed.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::ConfigLimitSwitches(bool bFwdLimit, bool bRevLimit)
{
	// Set the member variables.
	m_bFwdLimitSwitchNormallyOpen = bFwdLimit;
	m_bRevLimitSwitchNormallyOpen = bRevLimit;

	m_pMotor->GetForwardLimitSwitch(bFwdLimit ?
									CANDigitalInput::LimitSwitchPolarity::kNormallyOpen : 
									CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).EnableLimitSwitch(true);	
	m_pMotor->GetReverseLimitSwitch(bRevLimit ?
									CANDigitalInput::LimitSwitchPolarity::kNormallyOpen : 
									CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).EnableLimitSwitch(true);	
}

/******************************************************************************
	Description:	SetAccumIZone - sets the IZone for the accumulated integral.
	Arguments:	 	double dIZone - The accumulated integral is reset to zero
					when the error exceeds this value. This value is in the
					units of measure.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetAccumIZone(double dIZone)
{
	// Set the member variable.
	m_dIZone = dIZone;

	// Set the Integral Zone. Accumulated integral is reset to zero when the error exceeds this value.
	m_pMotor->GetPIDController().SetIZone(m_dIZone * m_dRevsPerUnit * m_nPulsesPerRev);
}

/******************************************************************************
	Description:	IsFwdLimitSwitchPressed - Returns true if forward limit
					switch is pressed, false otherwise.
	Arguments:	 	None
	Returns: 		bool - True if pressed, false otherwise
******************************************************************************/
bool CSparkMotion::IsFwdLimitSwitchPressed()
{
	return ((m_bFwdLimitSwitchNormallyOpen && m_pMotor->GetForwardLimitSwitch(m_bFwdLimitSwitchNormallyOpen ?
										      CANDigitalInput::LimitSwitchPolarity::kNormallyOpen : 
											  CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get()) ||
		   (!m_bFwdLimitSwitchNormallyOpen && !m_pMotor->GetForwardLimitSwitch(m_bFwdLimitSwitchNormallyOpen ?
										      CANDigitalInput::LimitSwitchPolarity::kNormallyOpen : 
											  CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get()));
}

/******************************************************************************
	Description:	IsRevLimitSwitchPressed - Returns true if reverse limit
					switch is pressed, false otherwise.
	Arguments:	 	None
	Returns: 		bool - True if pressed, false otherwise
******************************************************************************/
bool CSparkMotion::IsRevLimitSwitchPressed()
{
		return ((m_bRevLimitSwitchNormallyOpen && m_pMotor->GetReverseLimitSwitch(m_bRevLimitSwitchNormallyOpen ?
										      CANDigitalInput::LimitSwitchPolarity::kNormallyOpen : 
											  CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get()) ||
		   (!m_bRevLimitSwitchNormallyOpen && !m_pMotor->GetReverseLimitSwitch(m_bRevLimitSwitchNormallyOpen ?
										      CANDigitalInput::LimitSwitchPolarity::kNormallyOpen : 
											  CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get()));
}

/******************************************************************************
	Description:	IsAtSetpoint - Returns whether or not the motor has reached
					the desired setpoint.
	Arguments:	 	None
	Returns: 		bool - True if at setpoint, false otherwise.
******************************************************************************/
bool CSparkMotion::IsAtSetpoint()
{
	return (((fabs(GetSetpoint() - GetActual())) < m_dTolerance) && (fabs(m_pMotor->GetBusVoltage()) < 1.000));
}

/******************************************************************************
	Description:	ResetEncoderPosition - Sets the encoder position to zero.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::ResetEncoderPosition()
{
	// Reset the encoder count to zero.
//TODO: Figure this out!
//	m_pMotor->SetSelectedSensorPosition(0);
}

/******************************************************************************
	Description:	SetPeakOutputPercent - Sets the maximum output for the
					motors. This is in PercentOutput (-1, to 1).
	Arguments:	 	double dMaxFwdOutput - The maximum forward output.
					double dMaxRevOutput - The maximum reverse output.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetPeakOutputPercent(double dMaxFwdOutput, double dMaxRevOutput)
{
	m_pMotor->GetPIDController().SetOutputRange(dMaxFwdOutput, dMaxRevOutput);
}

/******************************************************************************
	Description:	SetNominalOutputVoltage - Sets the nominal output for the
					motors. This is in PercentOutput (-1, to 1).
	Arguments:	 	double dNominalFwdOutput - The nominal forward output.
					double dNominalRevOutput - The nominal reverse output.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetNominalOutputVoltage(double dNominalFwdOutput, double dNominalRevOutput)
{
//	m_pMotor->ConfigNominalOutputForward(dNominalFwdOutput);
//	m_pMotor->ConfigNominalOutputReverse(dNominalRevOutput);
}

/******************************************************************************
	Description:	SetOpenLoopRampRate - Sets the acceleration for open loop.
	Arguments:	 	double dOpenLoopRampRate - Acceleration in seconds from
					off to full output.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetOpenLoopRampRate(double dOpenLoopRampRate)
{
	m_pMotor->SetOpenLoopRampRate(dOpenLoopRampRate);
}

/******************************************************************************
	Description:	SetClosedLoopRampRate - Sets the acceleration for closed loop.
	Arguments:	 	double dClosedLoopRampRate - Acceleration in seconds from
					off to full output.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetClosedLoopRampRate(double dClosedLoopRampRate)
{
	m_pMotor->SetClosedLoopRampRate(dClosedLoopRampRate);
}

/******************************************************************************
	Description:	SetMotorNeutralMode - Sets the stop mode to brake or coast.
	Arguments:	 	int nMode - Mode, 1 is coast, 2 is brake.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetMotorNeutralMode(int nMode)
{
	m_pMotor->SetIdleMode((nMode == 1) ? CANSparkMax::IdleMode::kCoast : CANSparkMax::IdleMode::kBrake);
}

/******************************************************************************
	Description:	GetCurrentPositionInUnits - Returns the current position in units.
	Arguments:	 	None
	Returns: 		double - Position of the motor.
******************************************************************************/
double CSparkMotion::GetActual()
{
	return (m_pMotor->GetEncoder().GetPosition() / m_dRevsPerUnit / m_nPulsesPerRev);
}

/******************************************************************************
	Description:	SetHomeSpeeds - Sets the home speeds for the state machine.
	Arguments:	 	double dFwdSpeed - Speed for homing forward, coming off of
					home switch.
					double dRevSpeed - Speed for homing backward, moving towards
					home switch.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetHomeSpeeds(double dFwdSpeed, double dRevSpeed)
{
	m_dFwdHomeSpeed = dFwdSpeed;
	m_dRevHomeSpeed = dRevSpeed;
}

/******************************************************************************
	Description:	SetPulsesPerRev - Sets the pulses per revolution for the PID
					controller.
	Arguments:	 	int nPPR - Encoder pulses per revolution.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetPulsesPerRev(int nPPR)
{
	m_nPulsesPerRev = nPPR;
}

/******************************************************************************
	Description:	SetRevsPerUnit - Sets the revolutions per unit of measure.
	Arguments:	 	double dRPU - Revolutions per unit of measure.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetRevsPerUnit(double dRPU)
{
	m_dRevsPerUnit = dRPU;
}

/******************************************************************************
	Description:	SetPIDValues - Sets the PID and Feed Forward gain values.
	Arguments:	 	double dProportional 	- Proportion Gain
					double dIntegral		- Integral Gain
					double dDerivative		- Derivative Gain
					double dFeedForward		- Feed Forward Gain
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetPIDValues(double dProportional, double dIntegral, double dDerivative, double dFeedForward)
{
	m_pMotor->GetPIDController().SetP(dProportional);
	m_pMotor->GetPIDController().SetI(dIntegral);
	m_pMotor->GetPIDController().SetD(dDerivative);
	m_pMotor->GetPIDController().SetFF(dFeedForward);
}

/******************************************************************************
	Description:	SetMotorInverted - Inverts the motor output.
	Arguments:	 	bool bInverted - True to invert motor output.
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetMotorInverted(bool bInverted)
{
	m_pMotor->SetInverted(bInverted);
}

/******************************************************************************
	Description:	ClearStickyFaults - Clears the controller's sticky faults.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::ClearStickyFaults()
{
	m_pMotor->ClearFaults();
	m_pMotor->ClearError();
}

/******************************************************************************
	Description:	SetManualSpeed - Set the Manual Move Speed.
	Arguments:	 	double dForward, double dReverse
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetManualSpeed(double dForward, double dReverse)
{
	m_dFwdMoveSpeed = dForward;
	m_dRevMoveSpeed = dReverse;
}

/******************************************************************************
	Description:	SetAcceleration - Set the Motion Magic Acceleration.
	Arguments:	 	double dRPS
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetAcceleration(double dRPS)
{
	m_pMotor->GetPIDController().SetSmartMotionMaxAccel(dRPS);
}

/******************************************************************************
	Description:	SetCruiseRPM - Set the Motion Magic Cruise RPM.
	Arguments:	 	double dRPM
	Returns: 		Nothing
******************************************************************************/
void CSparkMotion::SetCruiseRPM(double dRPM)
{
	m_pMotor->GetPIDController().SetSmartMotionMaxVelocity(dRPM);
}
///////////////////////////////////////////////////////////////////////////////