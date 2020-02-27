/****************************************************************************
    Description:	Defines the CHood control class.
    Classes:		CHood
    Project:	    2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Hood_h
#define Hood_h

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Servo.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include "IOMap.h"

using namespace frc;

// Hood Constants.
const double dHoodMaxPosition			=    3750.0;
const double dHoodMinPosition			=      60.0;
const double dHoodManualFwdSpeed 		=     1.000;
const double dHoodManualRevSpeed		=    -1.000;
const double dHoodOpenLoopRamp			=     0.250;
const double dHoodClosedLoopRamp		=     0.250;
const int	 dHoodPulsesPerRev			=      1024;
const double dHoodRevsPerUnit			= 	1.0/360;
const double dHoodProportional          =      1e-3;
const double dHoodIntegral              =       0.0;
const double dHoodDerivative            =       0.0;
const double dHoodFeedForward           =       0.0;
const double dHoodTolerance             =       0.5;
const double dHoodFindingTime           =       0.0;

// Hood enum.
enum HoodState 		{eHoodIdle, eHoodStopped, eHoodTracking, eHoodFinding, eHoodManualFwd, eHoodManualRev};
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CHood class definition.
    Arguments:		None
    Derived From:	Nothing
******************************************************************************/
class CHood
{
public:
    CHood();
    ~CHood();
    void            Init();
    void 	        Tick();
    bool            IsReady();
    void            Stop();
    void			SetPID(double dProportional, double dIntegral, double dDerivative);
    void 			SetSetpoint(double dSetpoint);
    void			SetSpeed(double dSpeed);
    void 			SetState(HoodState nState)										{	m_nState = nState;								};
    void 			SetTolerance(double dTolerance);
    bool			IsAtSetpoint();
    double			GetActual()														{	return m_pHoodEncoder->GetDistance();				};
    double 			GetSetpoint()													{	return m_dSetpoint;								};
    double			GetTolerance()													{	return m_dTolerance;							};
    HoodState		GetState()														{	return m_nState;								};

private:
    // Object Pointers.
    Servo*					m_pHoodServo;
    Encoder*				m_pHoodEncoder;
    frc2::PIDController*	m_pHoodPID;
    Timer*                  m_pTimer;

    // Member variables.
    double 			m_dProportional;
    double 			m_dTrackingP;
    double 			m_dIntegral;
    double 			m_dDerivative;
    double 			m_dTolerance;
    double 			m_dSetpoint;
    double			m_dActual;
    double			m_dMaxFindingTime;
    double			m_dFindingStartTime;
    HoodState		m_nState;
    bool			m_bIsReady;
};
/////////////////////////////////////////////////////////////////////////////
#endif