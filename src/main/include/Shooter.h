/****************************************************************************
    Description:	Defines the CShooter control class.
    Classes:		CShooter
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Shooter_h
#define Shooter_h

#include <frc/Solenoid.h>
#include <frc/Servo.h>
#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalOutput.h>
#include "IOMap.h"

using namespace frc;

// Shooter Constants.
const double dShooterOpenLoopRamp		=     0.250;
const double dShooterClosedLoopRamp		=     0.250;
const double dShooterManualFwdSpeed		=	  0.500;
const double dShooterManualRevSpeed		=	 -0.500;
const double dShooterMaxVelocity		= 	5700.00;
const double dShooterIdleVelocity		=	2400.00;
const double dShooterFiringVelocity     =   4500.00;
const double dShooterMinVelocity		=	 200.00;
// Hood Constants.
const double dHoodMaxPosition			=      35.0;
const double dHoodMinPosition			=    	0.0;
const double dHoodManualFwdSpeed 		=     1.000;
const double dHoodManualRevSpeed		=    -1.000;
const double dHoodOpenLoopRamp			=     0.250;
const double dHoodClosedLoopRamp		=     0.250;
const int	 dHoodPulsesPerRev			=      1024;
const double dHoodRevsPerUnit			= 	1.0/360;

// Shooter enum.
enum ShooterState	{eShooterStopped, eShooterIdle, eShooterFinding, eShooterManualFwd, eShooterManualRev};
// Hood enum.
enum HoodState 		{eHoodIdle, eHoodFinding, eHoodTracking, eHoodManualFwd, eHoodManualRev};
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CShooter class definition.
    Arguments:		None
    Derived From:	Nothing
******************************************************************************/
class CShooter
{
public:
    CShooter();
    ~CShooter();

    // Public methods.
    void 			Init();
    void 			Tick();
    bool			IsReady()															{	return (m_bHoodIsReady && m_bShooterIsReady);		};
    void 			Stop();

    // Shooter methods.
    void			SetShooterPID(double dProportional, double dIntegral, double dDerivative, double dFeedForward);
    void 			SetShooterSetpoint(double dSetpoint);
    void 			SetShooterState(ShooterState nState)								{	m_nShooterState = nState;							};
    void 			SetShooterTolerance(double dTolerance);
    bool			IsShooterAtSetpoint();
    double			GetShooterActual()													{	return m_pLeftShooter->GetEncoder().GetVelocity();	};
    double 			GetShooterSetpoint()												{	return m_dShooterSetpoint;							};
    double			GetShooterTolerance()												{	return m_dShooterTolerance;							};
    ShooterState 	GetShooterState()													{	return m_nShooterState;								};		
    
    // Hood methods.
    void			SetHoodPID(double dProportional, double dIntegral, double dDerivative);
    void 			SetHoodSetpoint(double dSetpoint);
    void			SetHoodSpeed(double dSpeed);
    void 			SetHoodState(HoodState nState)										{	m_nHoodState = nState;								};
    void 			SetHoodTolerance(double dTolerance);
    bool			IsHoodAtSetpoint();
    double			GetHoodActual()														{	return m_pHoodEncoder->GetDistance();				};
    double 			GetHoodSetpoint()													{	return m_dHoodSetpoint;								};
    double			GetHoodTolerance()													{	return m_dHoodTolerance;							};
    HoodState		GetHoodState()														{	return m_nHoodState;								};

private:
    // Object pointers.
    Servo*					m_pHoodServo;
    rev::CANSparkMax*		m_pLeftShooter;
    rev::CANSparkMax*		m_pRightShooter;
    rev::CANPIDController*	m_pShooterPID;
    DigitalOutput*          m_pVisionSwitch;
    Encoder*				m_pHoodEncoder;
    frc2::PIDController*	m_pHoodPID;
    Timer*					m_pTimer;

    // Declare variables.
    bool			m_bIsReady;
    bool			m_bVisionTracking;
    bool			m_bMotionMagic;

    // Shooter variables.
    double 			m_dShooterProportional;
    double 			m_dShooterIntegral;
    double 			m_dShooterDerivative;
    double 			m_dShooterFeedForward;
    double 			m_dShooterTolerance;
    double 			m_dShooterSetpoint;
    double			m_dShooterActual;
    double			m_dShooterMaxFindingTime;
    double			m_dShooterFindingStartTime;
    ShooterState	m_nShooterState;
    bool			m_bShooterIsReady;

    // Hood variables.
    double 			m_dHoodProportional;
    double 			m_dHoodTrackingP;
    double 			m_dHoodIntegral;
    double 			m_dHoodDerivative;
    double 			m_dHoodTolerance;
    double 			m_dHoodSetpoint;
    double			m_dHoodActual;
    double			m_dHoodMaxFindingTime;
    double			m_dHoodFindingStartTime;
    HoodState		m_nHoodState;
    bool			m_bHoodIsReady;
};
/////////////////////////////////////////////////////////////////////////////
#endif