/****************************************************************************
    Description:	Implements the CColorWheel control class.
    Classes:		CColorWheel
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include "ColorWheel.h"

using namespace frc;
/////////////////////////////////////////////////////////////////////////////

/****************************************************************************
    Description:	CColorWheel Constructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CColorWheel::CColorWheel()
{
    // Create object pointers.
    m_pSpinner = new CANSparkMax(nColorWheelMotor, CANSparkMax::MotorType::kBrushless);
    m_pSensor  = new ColorSensorV3(I2C::kOnboard);

    // Initialize member variables.
    m_dSetpoint         = 0.0;
    m_dPositionActual   = 0.0;
    m_dVelocityActual   = 0.0;
    m_nState            = eColorStopped;
}

/****************************************************************************
    Description:	CColorWheel Destructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CColorWheel::~CColorWheel()
{
    // Delete objects.
    delete m_pSpinner;
    delete m_pSensor;

    // Set objects to nullptrs.
    m_pSpinner  = nullptr;
    m_pSensor   = nullptr;
}

/****************************************************************************
    Description:	Init - Ran once to initialize the Color Wheel
    Arguments:		None
    Returns:		Nothing
****************************************************************************/
void CColorWheel::Init()
{
    m_dPositionActual = m_pSpinner->GetEncoder().GetPosition();
    m_dVelocityActual = m_pSpinner->GetEncoder().GetVelocity();
}

/****************************************************************************
    Description:	Tick - Ran every 20ms to update the Color Wheel
    Arguments:		None
    Returns:		Nothing
****************************************************************************/
void CColorWheel::Tick()
{
    switch(m_nState)
    {
        case eColorStopped :
            break;

        case eColorVelocity :
            break;

        case eColorRed :
            break;

        case eColorYellow :
            break;

        case eColorBlue :
            break;

        case eColorGreen :
            break;
    }
}

/****************************************************************************
    Description:	MoveToColor - Moves to a given color using the state machine
    Arguments:		TargetColor color
    Returns:		Nothing
****************************************************************************/
void CColorWheel::MoveToColor(int nColor)
{

}

/****************************************************************************
    Description:	MoveToSetpoint - Moves to a velocity setpoint
    Arguments:		double dSetpoint
    Returns:		Nothing
****************************************************************************/
void CColorWheel::MoveToSetpoint(double dSetpoint)
{

}

/****************************************************************************
    Description:	IsAtSetpoint - Returns if at the velocity setpoint
    Arguments:		None
    Returns:		bool - Is at velocity setpoint
****************************************************************************/
bool CColorWheel::IsAtSetpoint()
{

}
///////////////////////////////////////////////////////////////////////////////