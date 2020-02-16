/****************************************************************************
    Description:	Defines the CHopper control class.
    Classes:		CHopper
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Hopper_h
#define Hopper_h

#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include "IOMap.h"

using namespace frc;
using namespace rev;
using namespace ctre;

// Hopper Contants.
const double dHopperMainSpeed       = -0.9;
const double dHopperPreloadSpeed  	= 0.5;
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CHopper class definition.
    Arguments:		None
    Derived From:	Nothing
******************************************************************************/
class CHopper
{
public:
    CHopper();
    ~CHopper();

    // Public Methods.
    void Init();
    void Feed(bool bEnabled = true);
    void Preload(bool bEnabled = true);

private:
    // Object pointers.
    WPI_TalonSRX*		m_pMainBelt;
    CANSparkMax*        m_pShooterFeeder;

    // Declare variables.
};
/////////////////////////////////////////////////////////////////////////////
#endif
