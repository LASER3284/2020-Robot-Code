/****************************************************************************
    Description:	Defines the CHood control class.
    Classes:		CHood
    Project:	    2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Hood_h
#define Hood_h

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include "IOMap.h"

using namespace frc;
using namespace rev;

// Hood Constants.
const double dHoodMaxPosition			=     400.0;
const double dHoodMinPosition			=     0.000;
const double dHoodIdlePosition          =     50.00;
const double dHoodManualFwdSpeed 		=     0.150;
const double dHoodManualRevSpeed		=    -0.150;
const double dHoodOpenLoopRamp			=     0.250;
const double dHoodClosedLoopRamp		=     0.250;
const int	 dHoodPulsesPerRev			=        42;
const double dHoodRevsPerUnit			= 	1.0/360;
const double dHoodProportional          =   0.00400;
const double dHoodIntegral              =  0.001600;
const double dHoodDerivative            = 0.0000010;
const double dHoodFeedForward           =      0.01;
const double dHoodTolerance             =      0.25;
const double dHoodFindingTime           =       15.0;
const double dHoodPresetPositionFar     =     250.0;
const double dHoodPresetPositionNear    =     100.0;
const double dHoodHomingSpeed           =      -0.2;
const double dHoodHomingCurrent         =      15.0;


// Hood enum.
enum HoodState 		{eHoodIdle, eHoodStopped, eHoodReset, eHoodHoming, eHoodTracking, eHoodFinding, eHoodManualFwd, eHoodManualRev};
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
    void            Rezero();
    void			SetPID(double dProportional, double dIntegral, double dDerivative);
    void 			SetSetpoint(double dSetpoint);
    void			SetSpeed(double dSpeed);
    void 			SetState(HoodState nState)										{	m_nState = nState;								    };
    void            SetHoodSafety(bool bEnabled)                                    {   m_bHoodSafety = bEnabled;                           };
    void 			SetTolerance(double dTolerance);
    bool			IsAtSetpoint();
    double			GetActual()														{	return m_pHoodMotor->GetEncoder().GetPosition() / (20.0 / 310.0) + dHoodMinPosition;	};
    double 			GetSetpoint()													{	return m_dSetpoint;								    };
    double			GetTolerance()													{	return m_dTolerance;							    };
    HoodState		GetState()														{	return m_nState;								    };

private:
    // Object Pointers.
    CANSparkMax*            m_pHoodMotor;
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
    double          m_dHomingStartTime;
    HoodState		m_nState;
    bool			m_bIsReady;
    bool            m_bHoodSafety;
};
/////////////////////////////////////////////////////////////////////////////
#endif