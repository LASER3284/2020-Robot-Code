/****************************************************************************
	Description:	Defines the CFalconMotion control class.
	Classes:		CFalconMotion
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 FIRST Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef FalconMotion_H
#define FalconMotion_H

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

using namespace ctre::phoenix::motorcontrol::can;
using namespace frc;

// Default Constants set for drive motors.
const int	 	nDefaultFalconMotionPulsesPerRev				=  	 43008;		// Encoder Pulses Per Revolution (84/8 * 2048).
const double 	dDefaultFalconMotionRevsPerUnit		    		=    (1.000 / (5.875 * 3.1415));	// Revolutions per unit of measure. (1 revs(Encoder)/(5.875 in * PI))
const double	dDefaultFalconMotionTimeUnitInterval			=	10.000;		// Falcon velocity returns rotations/100ms. (x10 for seconds)
const double 	dDefaultFalconMotionFwdHomeSpeed				=    0.000;		// Homing forward speed (set to zero because drive motors don't home)
const double 	dDefaultFalconMotionRevHomeSpeed				=    0.000;		// Homing reverse speed (set to zero because drive motors don't home)
const double 	dDefaultFalconMotionProportional				=    0.080; 	// Default proportional value.
const double 	dDefaultFalconMotionIntegral					=    0.000;		// Default integral value.
const double 	dDefaultFalconMotionDerivative		   		 	=    0.000;		// Default derivative value.
const double	dDefaultFalconMotionFeedForward		    		=	 0.105;		// Default feed forward value.
const double 	dDefaultFalconMotionVoltageRampRate	    		=    0.250;		// Default voltage ramp rate. This is in seconds from neutral to full output.
const double 	dDefaultFalconMotionTolerance		    		=    0.250;		// Default tolerance in desired units.
const double 	dDefaultFalconMotionLowerPositionSoftLimit	    = -250.000; 	// Default lower position soft limit. This is in desired units.
const double 	dDefaultFalconMotionUpperPositionSoftLimit	    =  250.000; 	// Default upper position soft limit. This is in desired units.
const double	dDefaultFalconMotionLowerVelocitySoftLimit		=  -10.000;		// Default lower velocity soft limit. This is in desired units.
const double	dDefaultFalconMotionUpperVelocitySoftLimit		= 	10.000;		// Default upper velocity soft limit. This is in desired units.
const double 	dDefaultFalconMotionIZone			    		=    5.000;		// Default IZone value. This is in the desired units.
const double 	dDefaultFalconMotionMaxHomingTime	    		=    0.000;		// Default Maximum allowable time to home. Zero to disable timeout. This is in seconds.
const double 	dDefaultFalconMotionMaxFindingTime	    		=    0.000;		// Default Maximum allowable time to move to position. Zero to disable timeout. This is in seconds.
const double	dDefualtFalconMotionManualFwdSpeed 	    		=	 0.500;
const double	dDefualtFalconMotionManualRevSpeed	    		=	-0.500;
enum State {eIdle, eHomingReverse, eHomingForward, eFinding, eManualForward, eManualReverse};
///////////////////////////////////////////////////////////////////////////////


class CFalconMotion
{
public:
    // Method Prototypes.
    CFalconMotion(int nDeviceID);
    ~CFalconMotion();

	void	ClearStickyFaults();
	void	ConfigLimitSwitches(bool bFwdLimitNormallyOpen, bool bRevLimitNormallyOpen);
	double	GetActual(bool bUsePosition);
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
	void	SetSensorInverted(bool bInverted);
	void	SetSetpoint(double dSetpoint, bool bUsePosition);
	void	SetPositionSoftLimits(double dMinValue, double dMaxValue);
	void	SetVelocitySoftLimits(double dMinValue, double dMaxValue);
	void	SetTolerance(double dValue);
    void	StartHoming();
	void	Stop();
    void	Tick();

    // One-line Methods.
    WPI_TalonFX*	GetMotorPointer()					{ return m_pMotor;														};
	bool	IsReady()									{ return m_bReady;														};
	bool	IsHomingComplete()							{ return m_bHomingComplete;												};
	void	SetMaxHomingTime(double dMaxHomingTime)		{ m_dMaxHomingTime = dMaxHomingTime;									};
	void	SetMaxFindingTime(double dMaxFindingTime)	{ m_dMaxFindingTime = dMaxFindingTime;									};
	State	GetState()									{ return m_nCurrentState;												};
	void	SetState(State nNewState)					{ m_nCurrentState = nNewState;											};
	double	GetMotorCurrent()							{ return m_pMotor->GetOutputCurrent();									};
	double	GetMotorVoltage()							{ return m_pMotor->GetMotorOutputVoltage(); 							};
	double	GetRevsPerUnit()							{ return m_dRevsPerUnit;												};
	int		GetPulsesPerRev()							{ return m_nPulsesPerRev;												};
	int		GetRawEncoderCounts()						{ return m_pMotor->GetSelectedSensorPosition();	                        };
	void	BackOffHome(bool bBackOff)					{ m_bBackOffHome = bBackOff;											};
	void	UseMotionMagic(bool bEnabled)				{ m_bMotionMagic = bEnabled;											};

private:
	// Object Pointers.
    WPI_TalonFX*            m_pMotor;
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
	double					m_dRevsPerUnit;
	double					m_dTimeUnitInterval;
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