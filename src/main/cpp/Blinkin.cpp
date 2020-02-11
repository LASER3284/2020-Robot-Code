/****************************************************************************
    Description:	Implements the Blinkin Class for LED Control.

    Classes:		Blinkin

    Project:		Blinkin LED Driver
****************************************************************************/
#include "Blinkin.h"

using namespace frc;
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	Blinkin Constructor.
    Arguments:		nPort - PWM port
    Derived From:	Nothing
******************************************************************************/
Blinkin::Blinkin(int nPort)
{
    // Instantiate objects.
    m_pBlinkin = new Spark(nPort);
    nID = eSinelon;
    dPWMOutput = (nID - 50.5) / 50.0;

    m_pBlinkin->SetSafetyEnabled(false);
}

/******************************************************************************
    Description:	Blinkin Destructor.
    Arguments:		None
    Derived From:	Nothing
******************************************************************************/
Blinkin::~Blinkin()
{
    // Delete Spark, set to nullptr.
    delete m_pBlinkin;
    m_pBlinkin = nullptr;
}

/******************************************************************************
    Description:	Sets the state of the Blinkin driver.
    Arguments:		nStateID - enum of the state selected.
    Returns:		Nothing
******************************************************************************/
void Blinkin::SetState(int nStateID)
{
    dPWMOutput = (nStateID - 50.5) / 50.0;
    m_pBlinkin->SetSpeed(dPWMOutput);
}

/******************************************************************************
    Description:	Sets the state of the Blinkin driver.
    Arguments:		None
    Returns:		int nState - Enum of the state of the Blinkin driver.
******************************************************************************/
int Blinkin::GetState()
{
    return nID;
}
/////////////////////////////////////////////////////////////////////////////