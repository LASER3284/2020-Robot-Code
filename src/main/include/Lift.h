/****************************************************************************
    Description:	Defines the CLift control class.
    Classes:		CLift
    Project:        2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Lift_h
#define Lift_h

#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "IOMap.h"

using namespace frc;
using namespace rev;

// Default constants for the CLift class.
const int       dLiftWinchPPR               =  52;              // NEO Integrated pulses per revolution.
const double    dLiftWinchRPU               =  14.6;            // Revolutions per unit (Inches).
const double    dLiftWinchSetpoint          =  180.0;           // Constant setpoint for climbing.
const double    dLiftWinchRaisedSetpoint    =  80.0;            // Constant setpoint for raising during climbing.
const double    dLiftWinchTolerance         =  5.0;             // Tolerance for winch PID controller.
const double    dLiftProportional           =  0.1;             // Winch PID Proportional.
const double    dLiftIntegral               =  0.0;             // Winch PID Integral.
const double    dLiftDerivative             =  0.0;             // Winch PID Derivative.

// Lift Enum
enum LiftStates {eLiftIdle = 0, eLiftExtend, eLiftRaising, eLiftStopped, eLiftRetract1, eLiftRetract2};
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CLift class definition.
    Arguments:		None
    Derived From:	Nothing
******************************************************************************/
class CLift
{
public:
    CLift();
    ~CLift();
    void Init();
    void Tick();
    void ExtendArm(bool bExtend = true);
    void MoveToSetpoint();
    void Stop();
    bool IsExtended();
    void ReverseIdle(bool bEnabled);
    bool IsReady();
    void TestLeftWinch(double dSpeed);
    void TestRightWinch(double dSpeed);
    void SetState(int nNewState);

private:
    // Object Pointers.
    CANSparkMax*    m_pWinchMotorLeft;
    CANSparkMax*    m_pWinchMotorRight;
    Solenoid*       m_pLiftSolenoid;

    // Member variables.
    int     m_nState;
    bool    m_bIsReady;
    bool    m_bIsIdling;
    double  m_dActual;
    double  m_dSetpoint;

};
/////////////////////////////////////////////////////////////////////////////
#endif