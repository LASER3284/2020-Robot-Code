/****************************************************************************
	Description:	Defines the CSparkMotion control class.
	Classes:		CSparkMotion
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 FIRST Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef SparkMotion_H
#define SparkMotion_H

#include <rev/CANSparkMax.h>
#include <frc/Timer.h>
#include "IOMap.h"

using namespace rev;
using namespace frc;

// Default Constants set for drive motors.
const int	 	nDefaultSparkMotionPulsesPerRev					=  	    42;		// Encoder Pulses Per Revolution (Integrated).
const double 	dDefaultSparkMotionRevsPerUnit		    		=    (1.000 / (5.875* 3.1415));	// Revolutions per unit of measure. (1 revs(Encoder)/(5.875 in * PI))
const double	dDefaultSparkMotionTimeUnitInterval				=	10.000;		//TODO: Spark MAX velocity unknown, but Falcon returns 100ms so we'll keep this
const double 	dDefaultSparkMotionFwdHomeSpeed					=    0.000;		// Homing forward speed (set to zero because drive motors don't home)
const double 	dDefaultSparkMotionRevHomeSpeed					=    0.000;		// Homing reverse speed (set to zero because drive motors don't home)
const double 	dDefaultSparkMotionProportional					=    0.500; 	// Default proportional value.
const double 	dDefaultSparkMotionIntegral						=    0.000;		// Default integral value.
const double 	dDefaultSparkMotionDerivative		    		=    0.000;		// Default derivative value.
const double	dDefaultSparkMotionFeedForward		    		=	 0.000;		// Default feed forward value.
const double 	dDefaultSparkMotionVoltageRampRate	    		=    0.250;		// Default voltage ramp rate. This is in seconds from neutral to full output.
const double 	dDefaultSparkMotionTolerance		    		=    0.250;		// Default tolerance in desired units.
const double 	dDefaultSparkMotionLowerPositionSoftLimit	    = -250.000; 	// Default lower  position soft limit. This is in desired units.
const double 	dDefaultSparkMotionUpperPositionSoftLimit	    =  250.000; 	// Default upper position soft limit. This is in desired units.
const double	dDefaultSparkMotionLowerVelocitySoftLimit		=  -10.000;		// Default lower velocity soft limit. This is in desired units.
const double	dDefaultSparkMotionUpperVelocitySoftLimit		= 	10.000;		// Default upper velocity soft limit. This is in desired units.
const double 	dDefaultSparkMotionIZone			    		=    5.000;		// Default IZone value. This is in the desired units.
const double 	dDefaultSparkMotionMaxHomingTime	    		=    0.000;		// Default Maximum allowable time to home. Zero to disable timeout. This is in seconds.
const double 	dDefaultSparkMotionMaxFindingTime	    		=    0.000;		// Default Maximum allowable time to move to position. Zero to disable timeout. This is in seconds.
const double	dDefualtSparkMotionManualFwdSpeed 	    		=	 0.500;
const double	dDefualtSparkMotionManualRevSpeed	    		=	-0.500;
///////////////////////////////////////////////////////////////////////////////


class CSparkMotion
{
public:
    // Method Prototypes.
    CSparkMotion(int nDeviceID);
    ~CSparkMotion();

	void	ClearStickyFaults();
	void	ConfigLimitSwitches(bool bFwdLimitNormallyOpen, bool bRevLimitNormallyOpen);
	double	GetActual();
    double	GetSetpoint();
    double  GetTolerance();
    bool    IsAtSetpoint();
	bool	IsFwdLimitSwitchPressed();
	bool	IsRevLimitSwitchPressed();
	void	ResetEncoderPosition();
    void	SetAcceleration(double dRPS);
	void	SetAccumIZone(double dIZone);
	void	SetClosedLoopRampRate(double dClosedLoopRampRate);
    void	SetCruiseRPM(double dRPM);
	void	SetHomeSpeeds(double dFwdSpeed, double dRevSpeed);
	void	SetManualSpeed(double dForward, double dReverse);
	void	SetMotorInverted(bool bInverted);
	void	SetMotorNeutralMode(int nMode);
	void	SetNominalOutputVoltage(double dNominalFwdOutput, double dNominalRevOutput);
	void	SetOpenLoopRampRate(double dOpenLoopRampRate);
	void	SetPeakOutputPercent(double dMaxFwdOutput, double dMaxRevOutput);
	void	SetPIDValues(double dProportional, double dIntegral, double dDerivative, double dFeedForward = 0.000);
	void	SetPulsesPerRev(int nPPR);
	void	SetRevsPerUnit(double dRPU);
	void	SetSetpoint(double dSetpoint, bool bUsePosition);
	void	SetPositionSoftLimits(double dMinValue, double dMaxValue);
	void	SetVelocitySoftLimits(double dMinValue, double dMaxValue);
	void	SetTolerance(double dValue);
    void	StartHoming();
	void	Stop();
    void	Tick();

    // One-line Methods.
    CANSparkMax*	GetMotorPointer()			    	{ return m_pMotor;														};
	bool	IsReady()									{ return m_bReady;														};
	bool	IsHomingComplete()							{ return m_bHomingComplete;												};
	void	SetMaxHomingTime(double dMaxHomingTime)		{ m_dMaxHomingTime = dMaxHomingTime;									};
	void	SetMaxFindingTime(double dMaxFindingTime)	{ m_dMaxFindingTime = dMaxFindingTime;									};
	State	GetState()									{ return m_nCurrentState;												};
	void	SetState(State nNewState)					{ m_nCurrentState = nNewState;											};
	double	GetMotorCurrent()							{ return m_pMotor->GetOutputCurrent();									};
	double	GetMotorVoltage()							{ return m_pMotor->GetBusVoltage(); 							};
	double	GetRevsPerUnit()							{ return m_dRevsPerUnit;												};
	int		GetPulsesPerRev()							{ return m_nPulsesPerRev;												};
//	int		GetRawEncoderCounts()						{ return m_pMotor->GetSelectedSensorPosition();	                        };
	void	BackOffHome(bool bBackOff)					{ m_bBackOffHome = bBackOff;											};
	void	UseMotionMagic(bool bEnabled)				{ m_bMotionMagic = bEnabled;											};

private:
	// Object Pointers.
    CANSparkMax*            m_pMotor;
	Timer*                  m_pTimer;

	// Member Variables.
	bool					m_bFwdLimitSwitchNormallyOpen;
	bool					m_bRevLimitSwitchNormallyOpen;
	bool					m_bHomingComplete;
	bool					m_bReady;
	bool					m_bBackOffHome;
	bool					m_bMotionMagic;
	bool					m_bUsePosition;
	int						m_nPulsesPerRev;
	int						m_nDeviceID;
	double					m_dSetpoint;
	double					m_dTimeUnitInterval; //TODO: Still unknown. Needs testing.
	double					m_dRevsPerUnit;
	double					m_dFwdMoveSpeed;
	double					m_dRevMoveSpeed;
	double					m_dFwdHomeSpeed;
	double					m_dRevHomeSpeed;
	double					m_dTolerance;
	double					m_dLowerPositionSoftLimit;
	double					m_dUpperPositionSoftLimit;
	double					m_dLowerVelocitySoftLimit;
	double					m_dUpperVelocitySoftLimit;
	double					m_dIZone;
	double					m_dMaxHomingTime;
	double					m_dMaxFindingTime;
	double					m_dHomingStartTime;
	double					m_dFindingStartTime;
	State					m_nCurrentState;
};
#endif