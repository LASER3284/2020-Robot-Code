/****************************************************************************
    Description:	Defines the Blinkin Class for LED Control.

    Classes:		Blinkin

    Project:		Blinkin LED Driver
****************************************************************************/
#ifndef Blinkin_h
#define Blinkin_h

#include <frc/Spark.h>
using namespace frc;
/////////////////////////////////////////////////////////////////////////////
class Blinkin
{
public:
    // Construction
    Blinkin(int nPort);
    ~Blinkin();

    // Getter/Setter methods.
    void SetState(int nStateID);
    int  GetState();

    // Huge list of states.
    enum BlinkinState
    {
        eRainbowRainbowPalette = 1,
        eRainbowPartyPalette,
        eRainbowOceanPalette,
        eRainbowLavaPalette,
        eRainbowForestPalette,
        eRainbowGlitter,
        eConfetti,
        eShotRed,
        eShotBlue,
        eShotWhite,
        eSinelonRainbowPalette,
        eSinelonPartyPalette,
        eSinelonOceanPalette,
        eSinelonLavaPalette,
        eSinelonForestPalette,
        eBeatsPerMinRainbowPalette,
        eBeatsPerMinPartyPalette,
        eBeatsPerMinOceanPalette,
        eBeatsPerMinLavaPalette,
        eBeatsPerMinForestPalette,
        eFireMedium,
        eFireLarge,
        eTwinkleRainbowPalette,
        eTwinklePartyPalette,
        eTwinkleOceanPalette,
        eTwinkleLavaPalette,
        eTwinkleForestPalette,
        eColorWaveRainbowPalette,
        eColorWavePartyPalette,
        eColorWaveOceanPalette,
        eColorWaveLavaPalette,
        eColorWaveForestPalette,
        eLarsonScanRed,
        eLarsonScanGray,
        eLightChaseRed,
        eLightChaseBlue,
        eLightChaseGray,
        eHeartbeatRed,
        eHeartbeatBlue,
        eHeartbeatWhite,
        eHeartbeatGray,
        eBreathRed,
        eBreathBlue,
        eBreathGray,
        eStrobeRed,
        eStrobeBlue,
        eStrobeGold,
        eStrobeWhite,
        eEndToEndBlend1,
        eLarsonScanner1,
        eLightChase1,
        eHeartbeatSlow1,
        eHeartbeatMedium1,
        eHeartbeatFast1,
        eBreathSlow1,
        eBreathFast1,
        eShot1,
        eStrobe1,
        eEndToEndBlend2,
        eLarsonScanner2,
        eLightChase2,
        eHeartbeatSlow2,
        eHeartbeatMedium2,
        eHeartbeatFast2,
        eBreathSlow2,
        eBreathFast2,
        eShot2,
        eStrobe2,
        eSparkle1on2,
        eSparkle2on1,
        eColorGradient,
        eBeatsPerMin,
        eEndToEndBlend1to2,
        eEndToEndBlend,
        eColor1and2,
        eTwinkle,
        eColorWave,
        eSinelon,
        eHotPink,
        eDarkRed,
        eRed,
        eRedOrange,
        eOrange,
        eGold,
        eYellow,
        eLawnGreen,
        eLime,
        eDarkGreen,
        eGreen,
        eBlueGreen,
        eAqua,
        eSkyBlue,
        eDarkBlue,
        eBlue,
        eBlueViolet,
        eViolet,
        eWhite,
        eGray,
        eDarkGray,
        eBlack
    };

private:
    int nID;
    double dPWMOutput;
    Spark*  m_pBlinkin;
};
/////////////////////////////////////////////////////////////////////////////
#endif