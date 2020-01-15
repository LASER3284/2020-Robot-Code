/******************************************************************************
	Description:	Implements the CFalconMotion control class.
	Classes:		CFalconMotion
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 FIRST Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#include "FalconMotion.h"

using namespace frc;
using namespace ctre::phoenix::motorcontrol;
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
	Description:	CFalconMotion Constructor.
	Arguments:		int nDeviceID - CAN Bus Device ID
	Derived From:	Nothing
******************************************************************************/
CFalconMotion::CFalconMotion(int nDeviceID)
{
	m_nDeviceID = nDeviceID;
	// Create the object pointers.
	m_pMotor						= new WPI_TalonFX(nDeviceID);
	m_pTimer						= new Timer();

	// Initialize member variables.
	m_nCurrentState					= eIdle;
	m_bReady						= true;
	m_bFwdLimitSwitchNormallyOpen	= true;
	m_bRevLimitSwitchNormallyOpen	= true;
	m_bHomingComplete				= false;
	m_bBackOffHome					= true;
	m_bMotionMagic					= false;
	m_bUsePosition					= true;
	m_dSetpoint						= 0.000;
	m_nPulsesPerRev					= nDefaultFalconMotionPulsesPerRev;
	m_dRevsPerUnit					= dDefaultFalconMotionRevsPerUnit;
	m_dTimeUnitInterval				= dDefaultFalconMotionTimeUnitInterval;
	m_dFwdMoveSpeed					= dDefualtFalconMotionManualFwdSpeed;
	m_dRevMoveSpeed					= dDefualtFalconMotionManualRevSpeed;
	m_dFwdHomeSpeed					= dDefaultFalconMotionFwdHomeSpeed;
	m_dRevHomeSpeed					= dDefaultFalconMotionRevHomeSpeed;
	m_dTolerance					= dDefaultFalconMotionTolerance;
	m_dLowerPositionSoftLimit		= dDefaultFalconMotionLowerPositionSoftLimit;
	m_dUpperPositionSoftLimit		= dDefaultFalconMotionUpperPositionSoftLimit;
	m_dLowerVelocitySoftLimit		= dDefaultFalconMotionLowerVelocitySoftLimit;
	m_dUpperVelocitySoftLimit		= dDefaultFalconMotionUpperVelocitySoftLimit;
	m_dIZone						= dDefaultFalconMotionIZone;
	m_dMaxHomingTime				= dDefaultFalconMotionMaxHomingTime;
	m_dMaxFindingTime				= dDefaultFalconMotionMaxFindingTime;
	m_dHomingStartTime				= 0.000;
	m_dFindingStartTime				= 0.000;

	// Set up the feedback device for a quadrature encoder.
	m_pMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder);
	// Reset the encoder count to zero.
	ResetEncoderPosition();
	// Set the encoder and motor as both positive.
	SetMotorInverted(false);
	SetSensorInverted(false);
	// Set up the nominal motor output for both directions.
	SetNominalOutputVoltage(0.000, 0.000);
	// Set the peak (maximum) motor output for both directions.
	SetPeakOutputPercent(1.000, -1.000);
	// Set the tolerance.
	SetTolerance(m_dTolerance);
	// Set the PID and feed forward values.
	SetPIDValues(dDefaultFalconMotionProportional, dDefaultFalconMotionIntegral, dDefaultFalconMotionDerivative, dDefaultFalconMotionFeedForward);
	// Stop the motor.
	Stop();
	// Set the neutral mode to brake.
	m_pMotor->SetNeutralMode(NeutralMode::Brake);
	// Disable both forward and reverse limit switches.
	m_pMotor->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_Disabled);
	m_pMotor->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_Disabled);
	// Set acceleration (seconds from neutral to full output).
	SetOpenLoopRampRate(dDefaultFalconMotionVoltageRampRate);
	SetClosedLoopRampRate(dDefaultFalconMotionVoltageRampRate);
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
	Description:	CFalconMotion Destructor.
	Arguments:		None
	Derived From:	Nothing
******************************************************************************/
CFalconMotion::~CFalconMotion()
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
void CFalconMotion::Tick()
{
	// State machine
	switch(m_nCurrentState)
	{
		case eIdle :
			// Stop the motor.
			m_pMotor->Set(ControlMode::PercentOutput, 0.000);
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
				m_pMotor->Set(ControlMode::PercentOutput, 0.00);
				if (m_bBackOffHome)
				{
					// Set the state to eHomingForward.
					m_nCurrentState = eHomingForward;
				}
				else
				{
					// Reset the encoder to zero.
					m_pMotor->SetSelectedSensorPosition(0);
					// Stop the motor and change the control mode to position.
					m_pMotor->Set(ControlMode::Position, 0.000);
					// Set flag that homing is complete.
					m_bHomingComplete = true;
					// Move to idle.
					m_nCurrentState = eIdle;
				}
			}
			else
			{
				// Not yet at the home limit switch, keep moving.
				m_pMotor->Set(ControlMode::PercentOutput, m_dRevHomeSpeed);
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
				m_pMotor->SetSelectedSensorPosition(0);
				// Stop the motor and change the control mode to position.
				m_pMotor->Set(ControlMode::Position, 0.000);
				// Set flag that homing is complete.
				m_bHomingComplete = true;
				// Set the state to eIdle.
				m_nCurrentState = eIdle;
			}
			else
			{
				// Still on the home limit switch, keep moving.
				m_pMotor->Set(ControlMode::PercentOutput, m_dFwdHomeSpeed);
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
				m_pMotor->Set(ControlMode::PercentOutput, m_dFwdMoveSpeed);
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
				m_pMotor->Set(ControlMode::PercentOutput, m_dRevMoveSpeed);
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
	Arguments:	 	dSetpoint - The position to move to in desired units.
					dUsePosition - Select position or velocity setpoint.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetSetpoint(double dSetpoint, bool bUsePosition)
{
	// Set the bUsePosition member variable.
	m_bUsePosition = bUsePosition;

	// Get current time.
	m_dFindingStartTime = m_pTimer->Get();

	// Use either position or velocity setpoints.
	if (bUsePosition)
	{
		// Clamp the new setpoint within soft limits.
		if (dSetpoint > m_dUpperPositionSoftLimit)
		{
			dSetpoint = m_dUpperPositionSoftLimit;
		}
		else
		{
			if (dSetpoint < m_dLowerPositionSoftLimit)
			{
				dSetpoint = m_dLowerPositionSoftLimit;
			}
		}
		
		// Set the dSetpoint member variable.
		m_dSetpoint = dSetpoint;

		// Set the motor to the desired position.
		if (m_bMotionMagic)
		{
			m_pMotor->Set(ControlMode::MotionMagic, (dSetpoint * m_dRevsPerUnit * m_nPulsesPerRev));
		}
		else
		{
			m_pMotor->Set(ControlMode::Position, (dSetpoint * m_dRevsPerUnit * m_nPulsesPerRev));
		}
	}
	else
	{
		// Clamp the new setpoint within soft limits.
		if (dSetpoint > m_dUpperVelocitySoftLimit)
		{
			dSetpoint = m_dUpperVelocitySoftLimit;
		}
		else
		{
			if (dSetpoint < m_dLowerVelocitySoftLimit)
			{
				dSetpoint = m_dLowerVelocitySoftLimit;
			}
		}
		
		// Set the dSetpoint member variable.
		m_dSetpoint = dSetpoint;

		// Set the motor to the desired position.
		m_pMotor->Set(ControlMode::Velocity, dSetpoint * (84 / 8 * m_nPulsesPerRev) / m_dTimeUnitInterval);
	}
	

    // Prints can slow down the processing time of the RoboRIO, so these are for debugging.
//	printf("CFalconMotion::SetSetpoint - Setpoint = %7.3f\n", dSetpoint);
//	printf("CFalconMotion::SetSetpoint - Revs Per Unit = %7.3f\n", m_dRevsPerUnit);

	// Set the state to eFinding.
	m_nCurrentState = eFinding;
}

/******************************************************************************
	Description:	GetSetpoint - Returns the current setpoint of the motor's
					PID in desired units of measure.
	Arguments:	 	None
	Returns: 		The setpoint of the motor's PID in desired units of measure.
******************************************************************************/
double CFalconMotion::GetSetpoint()
{
	return m_dSetpoint;
}

/******************************************************************************
	Description:	StartHoming - Initializes the homing sequence.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::StartHoming()
{
	// Stop the motor and set the control mode for percent output.
	m_pMotor->Set(ControlMode::PercentOutput, 0.000);

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
void CFalconMotion::Stop()
{
	// Stop the motor.
	m_pMotor->Set(ControlMode::PercentOutput, 0.000);

	// Set the current state to eIdle.
	m_nCurrentState = eIdle;
}

/******************************************************************************
	Description:	SetTolerance - Sets the tolerance of the PID in desired
					units of measure.
	Arguments:	 	dValue - Tolerance in the desired units.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetTolerance(double dValue)
{
	// Set the member variable.
	m_dTolerance = dValue;

	// Set the allowed error for the PID. This is in quadrature pulses.
	m_pMotor->ConfigAllowableClosedloopError(0, m_dTolerance * m_dRevsPerUnit * m_nPulsesPerRev);
}

/******************************************************************************
	Description:	GetTolerance - Returns the tolerance in the desired units.
	Arguments:	 	None
	Returns: 		dValue - Tolerance in the desired units.
******************************************************************************/
double CFalconMotion::GetTolerance()
{
	return m_dTolerance;
}

/******************************************************************************
	Description:	SetPositionSoftLimits - Sets soft limits for minimum and maximum travel.
	Arguments:	 	dMinValue - Minimum travel distance.
					dMaxValue - Maximum travel distance.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetPositionSoftLimits(double dMinValue, double dMaxValue)
{
	// Set the member variables.
	m_dLowerPositionSoftLimit	= dMinValue;
	m_dUpperPositionSoftLimit	= dMaxValue;
}

/******************************************************************************
	Description:	SetVelocitySoftLimits - Sets soft limits for minimum and maximum speed.
	Arguments:	 	dMinValue - Minimum travel distance.
					dMaxValue - Maximum travel distance.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetVelocitySoftLimits(double dMinValue, double dMaxValue)
{
	// Set the member variables.
	m_dLowerVelocitySoftLimit	= dMinValue;
	m_dUpperVelocitySoftLimit	= dMaxValue;
}

/******************************************************************************
	Description:	ConfigLimitSwitches - Sets up the limit switches as
					normally open or normally closed.
	Arguments:	 	bool bFwdLimit - True if normally open, false if normally closed.
					bool bRevLimit - True if normally open, false if normally closed.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::ConfigLimitSwitches(bool bFwdLimit, bool bRevLimit)
{
	// Set the member variables.
	m_bFwdLimitSwitchNormallyOpen = bFwdLimit;
	m_bRevLimitSwitchNormallyOpen = bRevLimit;

	m_pMotor->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                                            (bFwdLimit ? LimitSwitchNormal::LimitSwitchNormal_NormallyOpen :
                                                         LimitSwitchNormal::LimitSwitchNormal_NormallyClosed));
	m_pMotor->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                                            (bRevLimit ? LimitSwitchNormal::LimitSwitchNormal_NormallyOpen :
                                                         LimitSwitchNormal::LimitSwitchNormal_NormallyClosed));
}

/******************************************************************************
	Description:	SetAccumIZone - sets the IZone for the accumulated integral.
	Arguments:	 	double dIZone - The accumulated integral is reset to zero
					when the error exceeds this value. This value is in the
					units of measure.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetAccumIZone(double dIZone)
{
	// Set the member variable.
	m_dIZone = dIZone;

	// Set the Integral Zone. Accumulated integral is reset to zero when the error exceeds this value.
	m_pMotor->Config_IntegralZone(0, (m_dIZone * m_dRevsPerUnit * m_nPulsesPerRev));
}

/******************************************************************************
	Description:	IsFwdLimitSwitchPressed - Returns true if forward limit
					switch is pressed, false otherwise.
	Arguments:	 	None
	Returns: 		bool - True if pressed, false otherwise
******************************************************************************/
bool CFalconMotion::IsFwdLimitSwitchPressed()
{
	return ((m_bFwdLimitSwitchNormallyOpen && m_pMotor->GetSensorCollection().IsFwdLimitSwitchClosed()) ||
		   (!m_bFwdLimitSwitchNormallyOpen && !m_pMotor->GetSensorCollection().IsFwdLimitSwitchClosed()));
}

/******************************************************************************
	Description:	IsRevLimitSwitchPressed - Returns true if reverse limit
					switch is pressed, false otherwise.
	Arguments:	 	None
	Returns: 		bool - True if pressed, false otherwise
******************************************************************************/
bool CFalconMotion::IsRevLimitSwitchPressed()
{
	return ((m_bRevLimitSwitchNormallyOpen && m_pMotor->GetSensorCollection().IsRevLimitSwitchClosed()) ||
		   (!m_bRevLimitSwitchNormallyOpen && !m_pMotor->GetSensorCollection().IsRevLimitSwitchClosed()));
}

/******************************************************************************
	Description:	IsAtSetpoint - Returns whether or not the motor has reached
					the desired setpoint.
	Arguments:	 	None
	Returns: 		bool - True if at setpoint, false otherwise.
******************************************************************************/
bool CFalconMotion::IsAtSetpoint()
{
	return (((fabs(GetSetpoint() - GetActual())) < m_dTolerance) && (fabs(m_pMotor->GetMotorOutputVoltage()) < 1.000));
}

/******************************************************************************
	Description:	ResetEncoderPosition - Sets the encoder position to zero.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::ResetEncoderPosition()
{
	// Reset the encoder count to zero.
	m_pMotor->SetSelectedSensorPosition(0);
}

/******************************************************************************
	Description:	SetPeakOutputPercent - Sets the maximum output for the
					motors. This is in PercentOutput (-1, to 1).
	Arguments:	 	double dMaxFwdOutput - The maximum forward output.
					double dMaxRevOutput - The maximum reverse output.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetPeakOutputPercent(double dMaxFwdOutput, double dMaxRevOutput)
{
	m_pMotor->ConfigPeakOutputForward(dMaxFwdOutput);
	m_pMotor->ConfigPeakOutputReverse(dMaxRevOutput);
}

/******************************************************************************
	Description:	SetNominalOutputVoltage - Sets the nominal output for the
					motors. This is in PercentOutput (-1, to 1).
	Arguments:	 	double dNominalFwdOutput - The nominal forward output.
					double dNominalRevOutput - The nominal reverse output.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetNominalOutputVoltage(double dNominalFwdOutput, double dNominalRevOutput)
{
	m_pMotor->ConfigNominalOutputForward(dNominalFwdOutput);
	m_pMotor->ConfigNominalOutputReverse(dNominalRevOutput);
}

/******************************************************************************
	Description:	SetOpenLoopRampRate - Sets the acceleration for open loop.
	Arguments:	 	double dOpenLoopRampRate - Acceleration in seconds from
					off to full output.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetOpenLoopRampRate(double dOpenLoopRampRate)
{
	m_pMotor->ConfigOpenloopRamp(dOpenLoopRampRate);
}

/******************************************************************************
	Description:	SetClosedLoopRampRate - Sets the acceleration for closed loop.
	Arguments:	 	double dClosedLoopRampRate - Acceleration in seconds from
					off to full output.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetClosedLoopRampRate(double dClosedLoopRampRate)
{
	m_pMotor->ConfigClosedloopRamp(dClosedLoopRampRate);
}

/******************************************************************************
	Description:	SetMotorNeutralMode - Sets the stop mode to brake or coast.
	Arguments:	 	int nMode - Mode, 1 is coast, 2 is brake.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetMotorNeutralMode(int nMode)
{
	m_pMotor->SetNeutralMode((nMode == 1) ? NeutralMode::Coast : NeutralMode::Brake);
}

/******************************************************************************
	Description:	GetCurrentPositionInUnits - Returns the current position in units.
	Arguments:	 	None
	Returns: 		double - Position of the motor.
******************************************************************************/
double CFalconMotion::GetActual()
{
	// Create instance variables.
	double dActual = 0.000;

	if (m_bUsePosition)
	{
		dActual = (m_pMotor->GetSelectedSensorPosition() / m_dRevsPerUnit / m_nPulsesPerRev);
	}
	else
	{
		dActual = (m_pMotor->GetSelectedSensorVelocity() / (84 / 8 * m_nPulsesPerRev) * m_dTimeUnitInterval);
	}

	return dActual;
}

/******************************************************************************
	Description:	SetHomeSpeeds - Sets the home speeds for the state machine.
	Arguments:	 	double dFwdSpeed - Speed for homing forward, coming off of
					home switch.
					double dRevSpeed - Speed for homing backward, moving towards
					home switch.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetHomeSpeeds(double dFwdSpeed, double dRevSpeed)
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
void CFalconMotion::SetPulsesPerRev(int nPPR)
{
	m_nPulsesPerRev = nPPR;
}

/******************************************************************************
	Description:	SetRevsPerUnit - Sets the revolutions per unit of measure.
	Arguments:	 	double dRPU - Revolutions per unit of measure.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetRevsPerUnit(double dRPU)
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
void CFalconMotion::SetPIDValues(double dProportional, double dIntegral, double dDerivative, double dFeedForward)
{
	m_pMotor->Config_kP(0, dProportional);
	m_pMotor->Config_kI(0, dIntegral);
	m_pMotor->Config_kD(0, dDerivative);
	m_pMotor->Config_kF(0, dFeedForward);
}

/******************************************************************************
	Description:	SetMotorInverted - Inverts the motor output.
	Arguments:	 	bool bInverted - True to invert motor output.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetMotorInverted(bool bInverted)
{
	m_pMotor->SetInverted(bInverted);
}

/******************************************************************************
	Description:	SetSensorInverted - Inverts the sensor input.
	Arguments:	 	bool bInverted - True to invert sensor input.
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetSensorInverted(bool bInverted)
{
	m_pMotor->SetSensorPhase(bInverted);
}

/******************************************************************************
	Description:	ClearStickyFaults - Clears the controller's sticky faults.
	Arguments:	 	None
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::ClearStickyFaults()
{
	m_pMotor->ClearStickyFaults();
}

/******************************************************************************
	Description:	SetManualSpeed - Set the Manual Move Speed.
	Arguments:	 	double dForward, double dReverse
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetManualSpeed(double dForward, double dReverse)
{
	m_dFwdMoveSpeed = dForward;
	m_dRevMoveSpeed = dReverse;
}

/******************************************************************************
	Description:	SetAcceleration - Set the Motion Magic Acceleration.
	Arguments:	 	double dRPS
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetAcceleration(double dRPS)
{
	m_pMotor->ConfigMotionAcceleration(dRPS);
}

/******************************************************************************
	Description:	SetCruiseRPM - Set the Motion Magic Cruise RPM.
	Arguments:	 	double dRPM
	Returns: 		Nothing
******************************************************************************/
void CFalconMotion::SetCruiseRPM(double dRPM)
{
	m_pMotor->ConfigMotionCruiseVelocity(dRPM);
}
///////////////////////////////////////////////////////////////////////////////
