/****************************************************************************
    Description:	Defines the CColorWheel control class.
    Classes:		CColorWheel
    Project:	2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef ColorWheel_h
#define ColorWheel_h

#include <rev/CANSparkMax.h>
#include <rev/ColorSensorV3.h>
#include <IOMap.h>

using namespace frc;
using namespace rev;

// Default constants for the ColorWheel class.
const double dColorWheelTargetVelocity  =  60.0;
const double dColorWheelConversionRatio =  142.2;

// ColorWheel enum.
enum TargetColor {eColorStopped, eColorVelocity, eColorRed, eColorYellow, eColorBlue, eColorGreen};
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CColorWheel class definition.
    Arguments:		None
    Derived From:	Nothing
******************************************************************************/
class CColorWheel
{
public:
    CColorWheel();
    ~CColorWheel();
    void Init();
    void Tick();
    void MoveToColor(int nColor);
    void MoveToSetpoint(double dSetpoint);
    bool IsAtSetpoint();

private:
    // Object Pointers.
    CANSparkMax*    m_pSpinner;
    ColorSensorV3*  m_pSensor;

    // Member variables.
    double  m_dSetpoint;
    double  m_dVelocityActual;
    double  m_dPositionActual;
    int     m_nState;
};
/////////////////////////////////////////////////////////////////////////////
#endif