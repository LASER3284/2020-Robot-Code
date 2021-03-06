/****************************************************************************
    Description:	Defines the CIntake control class.
    Classes:		CIntake
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Intake_h
#define Intake_h

#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include "IOMap.h"

using namespace frc;
using namespace rev;

// Intake Contants.
const double dRetentionFwdSpeed =  1.00;
const double dIntakeFwdSpeed	=  1.00;
const double dRetentionRevSpeed =  -0.25;
const double dRetentionCurrent  =  20.0;
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CIntake class definition.
    Arguments:		None
    Derived From:	Nothing
******************************************************************************/
class CIntake
{
public:
    CIntake();
    ~CIntake();

    // Public Methods.
    void Init();
    void Extend(bool bExtend);
    bool GetExtended();
    void IntakeMotor(bool bEnabled);
    void RetentionMotor(bool bEnabled);
    void Unjam(bool bEnabled);
    bool IsJammed();
    double GetRetentionCurrent();
    double GetIntakeCurrent();

private:
    // Object pointers. 
    WPI_TalonSRX*		m_pIntakeMotor;
    CANSparkMax*        m_pRetentionMotor;
    Solenoid*			m_pIntakeActuator;
    Timer*              m_pTimer;

    // Declare variables.
};
/////////////////////////////////////////////////////////////////////////////
#endif
