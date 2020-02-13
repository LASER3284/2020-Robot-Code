/****************************************************************************
    Description:	Defines the 2020 Infinite Recharge Robot I/O map.
    Classes:		None
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef IOMap_h
#define IOMap_h
/////////////////////////////////////////////////////////////////////////////

// Solenoid Channels.
const int nIntakeSolenoid				=	0;		// Intake Solenoid Channel Number

// CAN Device IDs.
const int nLeftDriveMotor1		  		=  	1;		// Falcon ID for left drive motor 1
const int nLeftDriveMotor2		  		=  	2;		// Falcon ID for left drive motor 2 						
const int nRightDriveMotor1		  		=   3;		// Falcon ID for right drive motor 1
const int nRightDriveMotor2		  		=   4;		// Falcon ID for right drive motor 2						
const int nIntakeMotor					=	5;		// Talon SRX ID for intake motor
const int nTurretMotor					=	6;		// Talon SRX ID for turret motor
const int nShooterLeft					=	7;		// Spark MAX ID for left shooter motor
const int nShooterRight					= 	8;		// Spark MAX ID for right shooter motor
const int nShooterPreload				=	9;		// Spark MAX ID for shooter preloading
const int nColorWheelMotor				=  10;		// Spark MAX ID for Color Wheel motor
const int nClimberMotorLeft				=  11;		// Spark MAX ID for left climber winch
const int nClimberMotorRight			=  12;		// Spark MAX ID for right climber winch
const int nHopperBeltMotor				=  13;		// Talon SRX ID for belts in the hopper
const int nGondolaMotor					=  14;		// Talon SRX ID for the Gondola motor
const int nIntakeRetentionMotor         =  15;      // Spark MAX ID for retention in intake


// PWM Channels.
const int nBlinkinID					=	0;		// PWM channel for Blinkin LED driver
const int nHoodServo					= 	1;		// PWM channel for Hood actuation servo

// Relay Channels.

// Analog Channels.

// Digital Channels.
const int nHoodEncoderChannelA			=	0;		// Encoder clock A signal for hood servo.
const int nHoodEncoderChannelB			=	1;		// Encoder clock B signal for hood servo.

// Xbox Controller Button Assignments.
enum XboxButtons 		{eButtonA = 1, eButtonB, eButtonX, eButtonY, eButtonLB, eButtonRB, eBack, eStart, eButtonLS, eButtonRS};
// Xbox Controller Axis Assignments.
enum XboxAxis			{eLeftAxisX = 0, eLeftAxisY, eLeftTrigger, eRightTrigger, eRightAxisX, eRightAxisY};
// Logitech Flight Stick Button Assignments.
enum LogButtons	 		{eButtonTrigger = 1, eButton2, eButton3, eButton4, eButton5, eButton6, eButton7, eButton8, eButton9, eButton10, eButton11, eButton12};
// Shared Robot states for Motion.
enum State {eIdle, eHomingReverse, eHomingForward, eFinding, eManualForward, eManualReverse};
/////////////////////////////////////////////////////////////////////////////
#endif