/****************************************************************************
    Description:	Implements the CLift control class.
    Classes:		CLift
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include "Lift.h"

using namespace frc;
using namespace rev;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
    Description:	CLift Constructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CLift::CLift()
{
    // Create object pointers.
    m_pWinchMotorLeft   = new CANSparkMax(nClimberMotorLeft, CANSparkMax::MotorType::kBrushless);
    m_pWinchMotorRight  = new CANSparkMax(nClimberMotorRight, CANSparkMax::MotorType::kBrushless);
    m_pLiftSolenoid     = new Solenoid(nLiftSolenoid);

    // Initialize member variables.
    m_nState    = eLiftIdle;
    m_bIsReady  = false;
    m_dActual   = m_pWinchMotorLeft->GetEncoder().GetPosition() / dLiftWinchPPR / dLiftWinchRPU;
    m_dSetpoint = dLiftWinchSetpoint;
}

/****************************************************************************
    Description:	CLift Destructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CLift::~CLift()
{
    // Delete objects.
    delete m_pWinchMotorLeft;
    delete m_pWinchMotorRight;    
    delete m_pLiftSolenoid;

    // Set objects to nullptrs.
    m_pWinchMotorLeft   = nullptr;
    m_pWinchMotorRight  = nullptr;
    m_pLiftSolenoid     = nullptr;
}

/****************************************************************************
    Description:	Init - Called once when the robot is initially turned on.
    Arguments:		None
    Returns:		Nothing
****************************************************************************/
void CLift::Init()
{
    // Set the motors to brake just in case.
    m_pWinchMotorLeft->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_pWinchMotorRight->SetIdleMode(CANSparkMax::IdleMode::kBrake);

    // Make sure the solenoid is extended. (When going into Auto or Teleop, will engage)
    ExtendArm(false);
}

/****************************************************************************
    Description:	Tick - Called in a loop, implements functionality.
    Arguments:		None
    Returns:		Nothing
****************************************************************************/
void CLift::Tick()
{
    // Update the actual.
    m_dActual = m_pWinchMotorRight->GetEncoder().GetPosition();

    if (m_bIsIdling)
    {
        m_pWinchMotorLeft->SetSmartCurrentLimit(1.5);
        m_pWinchMotorRight->SetSmartCurrentLimit(1.5);
        m_pWinchMotorLeft->Set(-0.05);
        m_pWinchMotorRight->Set(-0.05);
    }
    else
    {
        m_pWinchMotorLeft->SetSmartCurrentLimit(60);
        m_pWinchMotorRight->SetSmartCurrentLimit(60);
    }
    
    switch (m_nState)
    {
        case eLiftIdle :
            // Idle - Do nothing, but extend cylinders to keep the height down.
            m_dSetpoint = dLiftWinchSetpoint;
            ExtendArm(false);
            m_pWinchMotorLeft->Set(0.0);
            m_pWinchMotorRight->Set(0.0);
            break;

        case eLiftExtend :
            // Extend - Retract cylinders to extend arms upwards, begin moving.
            ExtendArm(true);
            m_dSetpoint = dLiftWinchSetpoint;
            MoveToSetpoint();
            // Move to Raising.
            m_nState = eLiftRaising;
            break;

        case eLiftRaising :
            // Raising - Check to see if motor is at setpoint.
            if (IsReady())
            {
                // At setpoint, stop motors.
                Stop();
                // Move to Stopped.
                m_nState = eLiftStopped;
            }
            break;

        case eLiftStopped :
            // Stopped - Wait for driver input to start retracting.
            break;
        
        case eLiftRetract1 :
            // Retract 1 - Begin moving the winch back to zero.
            m_dSetpoint = dLiftWinchRaisedSetpoint;
            MoveToSetpoint();
            m_nState = eLiftRetract2;
            break;

        case eLiftRetract2 :
            // Retract 2 - Check if it is now at zero.
            if (IsReady())
            {
                // Stop the motors.
                Stop();
            }
            break;

        default :
            break;
    }

    SmartDashboard::PutNumber("Lift Actual", m_dActual);
}

/****************************************************************************
    Description:	ExtendArm - Extends or retracts the cylinders for the Arm.
    Arguments:		bool bExtend
    Returns:		Nothing
****************************************************************************/
void CLift::ExtendArm(bool bExtend)
{
    m_pLiftSolenoid->Set(!bExtend);
}

/****************************************************************************
    Description:	MoveToSetpoint - Move to the constant setpoint for lifting.
    Arguments:		None
    Returns:		Nothing
****************************************************************************/
void CLift::MoveToSetpoint()
{
    m_pWinchMotorLeft->GetPIDController().SetReference(m_dSetpoint, ControlType::kPosition);
    m_pWinchMotorRight->GetPIDController().SetReference(-m_dSetpoint, ControlType::kPosition);
}

/****************************************************************************
    Description:	Stop - Stops the winch motor.
    Arguments:		None
    Returns:		Nothing
****************************************************************************/
void CLift::Stop()
{
    m_pWinchMotorLeft->StopMotor();
    m_pWinchMotorRight->StopMotor();
}

/****************************************************************************
    Description:	IsReady - Returns whether the winch is at setpoint.
    Arguments:		None
    Returns:		bool - True for at setpoint
****************************************************************************/
bool CLift::IsReady()
{
    return (m_dSetpoint - m_dActual) <= dLiftWinchTolerance;
}

/****************************************************************************
    Description:	TestLeftWinch - Manually drive left winch motor for
                    testing.
    Arguments:		double - Speed in percent.
    Returns:		Nothing
****************************************************************************/
void CLift::TestLeftWinch(double dSpeed)
{
    m_pWinchMotorLeft->Set(dSpeed);
}

/****************************************************************************
    Description:	TestRightWinch - Manually drive right winch motor for
                    testing.
    Arguments:		double - Speed in percent.
    Returns:		Nothing
****************************************************************************/
void CLift::TestRightWinch(double dSpeed)
{
    m_pWinchMotorRight->Set(dSpeed);
}

/****************************************************************************
    Description:	SetState - Set the state of the Lift machine
    Arguments:		enum LiftState
    Returns:		Nothing
****************************************************************************/
void CLift::SetState(int nNewState)
{
    m_nState = nNewState;
}

/****************************************************************************
    Description:	IsExtended
    Arguments:		None
    Returns:		bool
****************************************************************************/
bool CLift::IsExtended()
{
    return !m_pLiftSolenoid->Get();
}

/****************************************************************************
    Description:	ReverseIdle
    Arguments:		bool
    Returns:		Nothing
****************************************************************************/
void CLift::ReverseIdle(bool bEnabled)
{
    m_bIsIdling = bEnabled;
}
///////////////////////////////////////////////////////////////////////////////