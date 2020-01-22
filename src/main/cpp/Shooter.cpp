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
	m_pLeftPID			= &m_pLeftShooter->GetPIDController();
	m_pRightPID			= &m_pRightShooter->GetPIDController();
	m_pServoController	= new frc2::PIDController(1.0, 0.0, 0.0);
	m_pTimer			= new Timer();
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
	delete m_pServoController;
	// Set objects to nullptrs.
	m_pHoodServo		= nullptr;
	m_pLeftShooter		= nullptr;
	m_pRightShooter		= nullptr;
	m_pTimer			= nullptr;
	m_pServoController	= nullptr;
}

/****************************************************************************
	Description:	Initialize Shooter parameters.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CShooter::Init()
{
    // Initialize variables.
	m_bMotionMagic				=	  		   false;
	m_dShooterProportional		=		 	  0.0004;
	m_dShooterIntegral			=			     0.0;
	m_dShooterDerivative		=		 	     0.0;
	m_dShooterFeedForward		=			  0.0001;
	m_dShooterTolerance			=			   100.0;
	m_dShooterSetpoint			=		         0.0;
	m_dShooterActual			= GetShooterActual();
	m_nShooterState				= 		eShooterIdle;
	m_bShooterIsReady			=			   false;

	m_dHoodProportional			=		         1.0;
	m_dHoodIntegral				=		         0.0;
	m_dHoodDerivative			=		         0.0;
	m_dHoodTolerance			= 		         2.0;
	m_dHoodSetpoint				=		         0.0;
	m_dHoodActual				=    GetHoodActual();
	m_dHoodMaxFindingTime		=		         3.0;
	m_dHoodFindingStartTime		=		         0.0;
	m_nHoodState				=          eHoodIdle;
	m_bHoodIsReady              =              false;

	m_bIsReady					=		   IsReady();

	// Set the shooter motors inverted from each other.
	m_pLeftShooter->SetInverted(false);
    m_pRightShooter->SetInverted(true);
	// Set the peak (maximum) motor output for both directions.
	m_pLeftShooter->GetPIDController().SetOutputRange(-1.0, 1.0);
	m_pRightShooter->GetPIDController().SetOutputRange(-1.0, 1.0);
	// Set the tolerances.
    SetShooterTolerance(m_dShooterTolerance);
	SetHoodTolerance(m_dHoodTolerance);
	// Set the PID and feed forward values.
	SetShooterPID(m_dShooterProportional, m_dShooterIntegral, m_dShooterDerivative, m_dShooterFeedForward);
    SetHoodPID(m_dHoodProportional, m_dHoodIntegral, m_dHoodDerivative);
	// Stop the mechanism.
	Stop();
	// Set the neutral mode to brake.
	m_pLeftShooter->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_pRightShooter->SetIdleMode(CANSparkMax::IdleMode::kCoast);
	// Set acceleration (seconds from neutral to full output).
	m_pLeftShooter->SetClosedLoopRampRate(dShooterClosedLoopRamp);
    m_pRightShooter->SetClosedLoopRampRate(dShooterClosedLoopRamp);
	// Clear any faults in memory.
	m_pLeftShooter->ClearFaults();
    m_pRightShooter->ClearFaults();

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
	// Update Actual variable.
	m_dActual = (m_pShooterMotor->GetSensorCollection().GetPulseWidthRiseToFallUs() - 1024) / 8.0;

	switch(m_nState)
	{
		case eShooterIdle :
			// Idle - Motor is off, and ready to move again.
			m_pPIDController->Reset();
			m_pShooterMotor->Set(ControlMode::PercentOutput, 0.00);
			m_bIsReady = true;
			break;

		case eShooterFinding :
			// Finding - Motor uses built-in PID controller to seek the
			// given Setpoint, and performs checks to ensure we are within
			// the given tolerance.
			std::cout << "FINDING" << std::endl;
			// Move the motor to a given point.
			m_pShooterMotor->Set(ControlMode::PercentOutput, m_pPIDController->Calculate(m_dActual));
			// Check to make sure it is or is not at setpoint.
			if (IsAtSetpoint() /*|| ((m_pTimer->Get() - m_dFindingStartTime) > m_dMaxFindingTime)*/)
			{
				// Stop the motor, return to Idle.
				Stop();
			}
			break;

		case eShooterManualFwd :
			// ManualFwd - Manually move the motor forward at a constant speed.
			m_pShooterMotor->Set(ControlMode::PercentOutput, dShooterManualFwdSpeed);
			break;
		
		case eShooterManualRev :
			// ManualRev - Manually move the motor backwards at a constant speed.
			m_pShooterMotor->Set(ControlMode::PercentOutput, dShooterManualRevSpeed);
			break;

		default :
			m_nState = eShooterIdle;
			break;
	}

	SmartDashboard::PutNumber("PID Output", m_pPIDController->Calculate(m_dActual));
	SmartDashboard::PutNumber("Shooter Position", m_dActual / dShooterRevsPerUnit / dShooterPulsesPerRev);
	SmartDashboard::PutNumber("Shooter User Setpoint", m_dSetpoint);
	SmartDashboard::PutNumber("Shooter Internal Setpoint", m_pPIDController->GetSetpoint());
	SmartDashboard::PutNumber("Shooter Error", m_pPIDController->GetPositionError() / dShooterRevsPerUnit / dShooterPulsesPerRev);
	SmartDashboard::PutNumber("Shooter Tolerance", m_dTolerance * dShooterPulsesPerRev * dShooterRevsPerUnit);
}

/****************************************************************************
	Description:	Set the variables of the PID controller for position.
	Arguments: 		double dProportional
					double dIntegral
					double dDerivative
					double dFeedForward
	Returns: 		Nothing
****************************************************************************/
void CShooter::SetPID(double dProportional, double dIntegral, double dDerivative)
{
	// Configure PID controller to use the given values.
	m_pPIDController->SetPID(dProportional, dIntegral, dDerivative);
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
	if (dSetpoint > dShooterMaxPosition)
	{
		dSetpoint = dShooterMaxPosition;
	}
	if (dSetpoint < dShooterMinPosition)
	{
		dSetpoint = dShooterMinPosition;
	}

	// Set the member variable.
	m_dSetpoint = dSetpoint;

	// Give the PID controller a setpoint.
	m_pPIDController->SetSetpoint(m_dSetpoint * dShooterPulsesPerRev * dShooterRevsPerUnit);

	// Start the timer for beginning finding.
	m_dFindingStartTime = m_pTimer->Get();

	// Set Shooter state to finding.
	SetState(eShooterFinding);

	std::cout << "Setpoint - " << m_dSetpoint << std::endl;
}

/****************************************************************************
	Description:	Set the State Machine to the given state.
	Arguments: 		ShooterState nState - Given state in state machine
	Returns: 		Nothing
****************************************************************************/
void CShooter::SetState(ShooterState nState)
{
	// Set member variable.
	m_nState = nState;
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
	m_pPIDController->SetTolerance(dTolerance * dShooterPulsesPerRev * dShooterRevsPerUnit);
}

/****************************************************************************
	Description:	Stops the Shooter at it's position, resets state to idle.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
void CShooter::Stop()
{
	// Disable the PID controller to stop any movement.
	m_pPIDController->Reset();
	// Stop the motor.
	m_pShooterMotor->Set(ControlMode::PercentOutput, 0.00);
	// Set the state to idle.
	SetState(eShooterIdle);
}

/****************************************************************************
	Description:	Checks to see if Shooter is within tolerance of the setpoint.
	Arguments: 		None
	Returns: 		Nothing
****************************************************************************/
bool CShooter::IsAtSetpoint()
{
	return (m_pPIDController->AtSetpoint());
}
/////////////////////////////////////////////////////////////////////////////
