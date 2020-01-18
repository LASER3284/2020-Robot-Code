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

// CAN Device IDs.
const int nLeftDriveMotor1		  		=  	1;		// Falcon ID for left drive motor 1
const int nLeftDriveMotor2		  		=  	2;		// Falcon ID for left drive motor 2 						
const int nRightDriveMotor1		  		=   3;		// Falcon ID for right drive motor 1
const int nRightDriveMotor2		  		=   4;		// Falcon ID for right drive motor 2						

// PWM Channels.

// Relay Channels.

// Analog Channels.

// Digital Channels.

// Xbox Controller Button Assignments.
enum XboxButtons 		{eButtonA = 1, eButtonB, eButtonX, eButtonY, eButtonLB, eButtonRB, eBack, eStart, eButtonLS, eButtonRS};
// Xbox Controller Axis Assignments.
enum XboxAxis			{eLeftAxisX = 1, eLeftAxisY, eLeftTrigger, eRightTrigger, eRightAxisX, eRightAxisY};
// Logitech Flight Stick Button Assignments.
enum LogButtons	 		{eButtonTrigger = 1, eButton2, eButton3, eButton4, eButton5, eButton6, eButton7, eButton8, eButton9, eButton10, eButton11, eButton12};
// Shared Robot states for Motion.
enum State {eIdle, eHomingReverse, eHomingForward, eFinding, eManualForward, eManualReverse};
/////////////////////////////////////////////////////////////////////////////
#endif