/****************************************************************************
    Description:	Defines the CTurret control class.
    Classes:		CTurret
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Turret_h
#define Turret_h

#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include "IOMap.h"

using namespace frc;
using namespace ctre;

// Turret Constants.
const double dTurretMaxPosition		=       2.0;
const double dTurretMinPosition		=     -25.0;
const double dTurretProportional	=	  0.020;
const double dTurretIntegral		=	  0.005;
const double dTurretDerivative		=     0.000;
const double dTurretManualFwdSpeed 	=     0.250;
const double dTurretManualRevSpeed	=    -0.250;
const double dTurretOpenLoopRamp	=     0.250;
const double dTurretClosedLoopRamp	=     0.250;
const int	 nTurretZeroOffset		=	  -3496;
const int	 nTurretPulsesPerRev	=      4096;
const double dTurretRevsPerUnit		= 	1.0/360;

// Turret enum.
enum TurretState {eTurretIdle = 0, eTurretStopped, eTurretFinding, eTurretTracking, eTurretManualFwd, eTurretManualRev};
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CTurret class definition.
    Arguments:		None
    Derived From:	Nothing
******************************************************************************/
class CTurret
{
public:
    CTurret();
    ~CTurret();

    // Public Methods.
    void 		Init();
    void 		Tick();
    void		SetPID(double dProportional, double dIntegral, double dDerivative);
    void 		SetSetpoint(double dSetpoint);
    void        SetVision(bool bEnabled);
    void 		SetState(TurretState nState);
    void 		SetTolerance(double dTolerance);
    void 		Stop();
    bool		IsAtSetpoint();
    double 		GetSetpoint()							{	return m_dSetpoint;		};
    double		GetTolerance()							{	return m_dTolerance;	};
    TurretState GetState()								{	return m_nState;		};		
    bool		IsReady()								{	return m_bIsReady;		};
    bool        CheckOverrun();

private:
    // Object pointers.
    WPI_TalonSRX*			m_pTurretMotor;
    frc2::PIDController*	m_pPIDController;
    Timer*					m_pTimer;

    // Declare variables.
    double 		m_dProportional;
    double 		m_dIntegral;
    double 		m_dDerivative;
    double 		m_dFeedForward;
    double 		m_dTolerance;
    double 		m_dSetpoint;
    double		m_dActual;
    double      m_dFakeActual;
    double		m_dMaxFindingTime;
    double		m_dFindingStartTime;
    TurretState	m_nState;
    bool		m_bIsReady;
    bool		m_bMotionMagic;
    bool        m_bHasOverrun;
};
/////////////////////////////////////////////////////////////////////////////
#endif