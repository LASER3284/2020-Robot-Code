/****************************************************************************
    Description:	Implements the CIntake control class.
    Classes:		CIntake
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include "Intake.h"

using namespace frc;
using namespace rev;
using namespace ctre;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
    Description:	CIntake Constructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CIntake::CIntake()
{
    // Create Object Pointers.
    m_pIntakeMotor		= new WPI_TalonSRX(nIntakeMotor);
    m_pRetentionMotor   = new CANSparkMax(nIntakeRetentionMotor, CANSparkMax::MotorType::kBrushless);
    m_pIntakeActuator	= new Solenoid(nIntakeSolenoid);
}

/****************************************************************************
    Description:	CIntake Destructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CIntake::~CIntake()
{
    // Delete objects.
    delete m_pIntakeMotor;
    delete m_pRetentionMotor;
    delete m_pIntakeActuator;

    // Set objects to nullptrs.
    m_pIntakeMotor		= nullptr;
    m_pRetentionMotor   = nullptr;
    m_pIntakeActuator	= nullptr;
}

/****************************************************************************
    Description:	Initialize Intake parameters.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CIntake::Init()
{
    // Retract Intake Mechanism.
    Extend(false);

    // Turn off Intake Motor.
    MotorSetPoint(eMotorStopped);
}

/****************************************************************************
    Description:	Extends or retracts the intake system.
    Arguments: 		bool bExtend - true to extend, false to retract
    Returns: 		Nothing
****************************************************************************/
void CIntake::Extend(bool bExtend)
{
    // Extend or Retract Intake Mechanism.
    m_pIntakeActuator->Set(bExtend);
}

/****************************************************************************
    Description:	Returns the state of the Intake Actuator.
    Arguments: 		None
    Returns: 		bool - True for extended, false for retracted.
****************************************************************************/
bool CIntake::GetExtended()
{
    // Returns the state of the Intake Actuator.
    return m_pIntakeActuator->Get();
}

/****************************************************************************
    Description:	Turns Intake Motor on or off.
    Arguments: 		int nState - eMotorReverse, eMotorStopped, eMotorForward
    Returns: 		Nothing
****************************************************************************/
void CIntake::MotorSetPoint(int nState)
{
    // Start or stop intake motor.
    switch (nState)
    {
        case eMotorReverse :
            m_pIntakeMotor->Set(dIntakeRevSpeed);
            m_pRetentionMotor->Set(dIntakeRevSpeed);
            break;

        case eMotorForward :
            m_pIntakeMotor->Set(ControlMode::PercentOutput, dIntakeFwdSpeed);
            m_pRetentionMotor->Set(dIntakeFwdSpeed);
            break;

        default :
            m_pIntakeMotor->Set(0.0);
            m_pRetentionMotor->Set(0.0);
            break;		
    }
}
/////////////////////////////////////////////////////////////////////////////
