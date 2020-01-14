/****************************************************************************
	Description:	Defines the CDrive control class.
	Classes:		CDrive
	Project:		2020 Infinite Recharge Robot Code.
	Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Drive_H
#define Drive_H

#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <ctre/Phoenix.h>
#include "RobotMain.h"

using namespace frc;
/////////////////////////////////////////////////////////////////////////////
class CDrive
{
public:
    CDrive(Joystick *pDriveController);
    ~CDrive();
    void Init();
    void Tick();
    void Stop();

private:
    Joystick            *m_pDriveController;
    WPI_TalonFX         *m_pLeftMotor1;
    WPI_TalonFX         *m_pLeftMotor2;
    WPI_TalonFX         *m_pRightMotor1;
    WPI_TalonFX         *m_pRightMotor2;
    DifferentialDrive   *m_pRobotDrive;
};
/////////////////////////////////////////////////////////////////////////////
#endif