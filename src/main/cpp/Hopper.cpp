/****************************************************************************
    Description:	Implements the CHopper control class.
    Classes:		CHopper
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include "Hopper.h"

using namespace frc;
using namespace rev;
using namespace ctre;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
    Description:	CHopper Constructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CHopper::CHopper()
{
    // Create Object Pointers.
    m_pMainBelt			= new WPI_TalonSRX(nHopperBeltMotor);
    m_pShooterFeeder	= new CANSparkMax(nShooterPreload, CANSparkMax::MotorType::kBrushless);
}

/****************************************************************************
    Description:	CHopper Destructor.
    Arguments:		None
    Derived From:	Nothing
****************************************************************************/
CHopper::~CHopper()
{
    // Delete objects.
    delete m_pMainBelt;
    delete m_pShooterFeeder;

    // Set objects to nullptrs.
    m_pMainBelt			= nullptr;
    m_pShooterFeeder	= nullptr;
}

/****************************************************************************
    Description:	Initialize Hopper parameters.
    Arguments: 		None
    Returns: 		Nothing
****************************************************************************/
void CHopper::Init()
{
    // Turn off main hopper motor.
    m_pMainBelt->Set(ControlMode::PercentOutput, 0.00);
    // Turn off shooter preload.
    m_pShooterFeeder->Set(0.00);
}

/****************************************************************************
    Description:	Start feeding Energy through the hopper.
    Arugments: 		bool - True for start, false for stop.
    Returns: 		Nothing
****************************************************************************/
void CHopper::Feed(bool bEnabled)
{
    if (bEnabled)
    {
        // Start feeding the hopper.
        m_pMainBelt->Set(ControlMode::PercentOutput, dHopperMainSpeed);
    }
    else
    {
        // Stop the hopper.
        m_pMainBelt->Set(ControlMode::PercentOutput, 0.00);
    }
}

/****************************************************************************
    Description:	Start feeding Energy through the hopper.
    Arugments: 		bool - True for start, false for stop.
    Returns: 		Nothing
****************************************************************************/
void CHopper::Preload(bool bEnabled)
{
    if (bEnabled)
    {
        // Start preloading into the shooter.
        m_pShooterFeeder->Set(dHopperPreloadSpeed);
    }
    else
    {
        // Stop the preloader.
        m_pShooterFeeder->Set(0.00);
    }
}

/****************************************************************************
    Description:	Reverse the belts for a short amount of time.
    Arugments: 		bool - True for start, false for stop.
    Returns: 		Nothing
****************************************************************************/
// void CHopper::Unjam(bool bEnabled)
// {
//     if (bEnabled)
//     {
//         m_pMainBelt->Set(0.6);
//     }
//     else
//     {
//         m_pMainBelt->Set(0.0);
//     }
// }
/////////////////////////////////////////////////////////////////////////////
