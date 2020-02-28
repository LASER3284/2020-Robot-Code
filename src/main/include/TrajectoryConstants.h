/****************************************************************************
    Description:	Defines the Poses used for autonomous.
    Classes:		CTrajectoryConstants
    Project:		2020 Infinite Recharge Robot Code.
    Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef TrajectoryConstants_h
#define TrajectoryConstants_h

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/Trajectory.h>

using namespace frc;
using namespace units;
using namespace std;

enum TrajectoryList 
{
    eAllianceTrench1 = 1,
    eAllianceTrench2, 
    eFrontShieldGenerator1,
    eFrontShieldGenerator2,
    eSideShieldGenerator1, 
    eSideShieldGenerator2,
    eOpposingTrench1,
    eOpposingTrench2, 
    ePowerPort1, 
    eTakePowerCells1, 
    eDoNothing, 
    eTestPath1
};
/////////////////////////////////////////////////////////////////////////////

class CTrajectoryConstants
{
public:
    void SelectTrajectory(int nTrajectory)
    {
        // Retrieve the correct trajectory.
        switch(nTrajectory)
        {
            case eAllianceTrench1 :
                m_StartPoint =
                {
                    10.0_ft,				// X starting position on field in feet.
                    21.0_ft,			    // Y starting position on field in feet.
                    Rotation2d(0_deg)		// Starting rotation on field in degrees.
                };

                m_Waypoints = 
                {
                    m_StartPoint,
                    Pose2d
                    {
                        17.5_ft,				// X of point 1 on field in feet.
                        24.5_ft,				// Y of point 1 on field in feet.
                        Rotation2d(0_deg)       // Rotation at point 1 in degrees.
                    },
                    Pose2d
                    {
                        26.0_ft,				// X of point 2 on field in feet.
                        24.5_ft,				// Y of point 2 on field in feet.
                        Rotation2d(0_deg)       // Rotation at point 2 in degrees.
                    }
                };

                m_bIsReversed = false;
                break;

            case eAllianceTrench2 :
                m_StartPoint = 
                {
                    Pose2d
                    {
                        26.0_ft,				// X of point 2 on field in feet.
                        24.5_ft,				// Y of point 2 on field in feet.
                        Rotation2d(0_deg)       // Rotation at point 2 in degrees.
                    }
                };

                m_Waypoints = 
                {
                    m_StartPoint,
                    Pose2d
                    {
                        27.0_ft,				// X of point 3 on field in feet.
                        24.5_ft,				// Y of point 3 on field in feet.
                        Rotation2d(0_deg)       // Rotation at point 3 in degrees.
                    }
                };

                m_bIsReversed = true;
                break;

            case eFrontShieldGenerator1 :
                m_StartPoint =
                {
                    10.0_ft,				// X starting position on field in feet.
                    15.5_ft,					// Y starting position on field in feet.
                    Rotation2d(0_deg)		// Starting rotation on field in degrees.
                };

                m_Waypoints =
                {
                    m_StartPoint,
                    Pose2d
                    {
                        18.5_ft,				    // X of point 1 on field in feet.
                        13.5_ft,				    // Y of point 1 on field in feet.
                        Rotation2d(10_deg)     // Rotation at point 1 in degrees.
                    }
                };

                m_bIsReversed = false;
                break;

            case eFrontShieldGenerator2 :
                m_StartPoint = 
                {
                    Pose2d
                    {
                        18.5_ft,				    // X of point 1 on field in feet.
                        13.5_ft,				    // Y of point 1 on field in feet.
                        Rotation2d(0_deg)     // Rotation at point 1 in degrees.
                    }
                };

                m_Waypoints = 
                {
                    m_StartPoint,
                    Pose2d
                    {
                        10.0_ft,				    // X ending position on field in feet.
                        15.5_ft,				// Y ending position on field in feet.
                        Rotation2d(-20_deg)		// Ending rotation on field in degrees.
                    }
                };

                m_bIsReversed = true;
                break;

            case eSideShieldGenerator1 :
                m_StartPoint =
                {
                    10.0_ft,				// X starting position on field in feet.
                    15.5_ft,			    // Y starting position on field in feet.
                    Rotation2d(0_deg)		// Starting rotation on field in degrees.
                };

                m_Waypoints = 
                {
                    m_StartPoint,
                    Pose2d
                    {
                        16.5_ft,				// X of point 1 on field in feet.
                        20.0_ft,				// Y of point 1 on field in feet.
                        Rotation2d(8_deg)       // Rotation at point 1 in degrees.
                    },
                    Pose2d
                    {
                        20.5_ft,				// X of point 2 on field in feet.
                        18.0_ft,				// Y of point 2 on field in feet.
                        Rotation2d(-70_deg)     // Rotation at point 2 in degrees.
                    }
                };

                m_bIsReversed = false;
                break;

            case eSideShieldGenerator2 :
                m_StartPoint = 
                {
                    20.5_ft,				// X of point 2 on field in feet.
                    18.0_ft,				// Y of point 2 on field in feet.
                    Rotation2d(-70_deg)     // Rotation at point 2 in degrees.
                };

                m_Waypoints = 
                {
                    m_StartPoint,
                    Pose2d
                    {
                        14.0_ft,				// X of point 3 on field in feet.
                        19.5_ft,				// Y of point 3 on field in feet.
                        Rotation2d(8_deg)       // Rotation at point 3 in degrees.
                    },
                    Pose2d
                    {
                        8.0_ft,				    // X ending position on field in feet.
                        19.0_ft,				// Y ending position on field in feet.
                        Rotation2d(0_deg)		// Ending rotation on field in degrees.
                    }
                };

                m_bIsReversed = true;
                break;

            case eOpposingTrench1 :
                m_StartPoint =
                {
                    10.0_ft,				// X starting position on field in feet.
                    6.5_ft,					// Y starting position on field in feet.
                    Rotation2d(0_deg)		// Starting rotation on field in degrees.
                };

                m_Waypoints = 
                {
                    m_StartPoint,
                    Pose2d
                    {
                        16.0_ft,				// X of point 1 on field in feet.
                        6.5_ft,				    // Y of point 1 on field in feet.
                        Rotation2d(0_deg)     // Rotation at point 1 in degrees.
                    },
                    Pose2d
                    {
                        20.5_ft,				// X of point 1 on field in feet.
                        5.5_ft,				    // Y of point 1 on field in feet.
                        Rotation2d(-30_deg)     // Rotation at point 1 in degrees.
                    }
                };

                m_bIsReversed = false;
                break;

            case eOpposingTrench2 :
                m_StartPoint = 
                {
                    Pose2d
                    {
                        20.5_ft,				// X of point 1 on field in feet.
                        5.5_ft,				    // Y of point 1 on field in feet.
                        Rotation2d(0_deg)     // Rotation at point 1 in degrees.
                    },
                };

                m_Waypoints = 
                {
                    m_StartPoint,
                    Pose2d
                    {
                        10.5_ft,				    // X of point 2 on field in feet.
                        10.5_ft,				// Y of point 2 on field in feet.
                        Rotation2d(-60_deg)      // Rotation at point 2 in degrees.
                    },
                    Pose2d
                    {
                        6.5_ft,				    // X ending position on field in feet.
                        16.5_ft,				// Y ending position on field in feet.
                        Rotation2d(-40_deg)		// Ending rotation on field in degrees.
                    }
                };
                
                m_bIsReversed = true;
                break;

            case ePowerPort1 :
                m_StartPoint =
                {
                    10.0_ft,				// X starting position on field in feet.
                    3.5_ft,					// Y starting position on field in feet.
                    Rotation2d(0_deg)		// Starting rotation on field in degrees.
                };

                m_Waypoints = 
                {
                    m_StartPoint,
                    Pose2d
                    {
                        5.0_ft,				    // X of point 1 on field in feet.
                        8.0_ft,				    // Y of point 1 on field in feet.
                        Rotation2d(-90_deg)     // Rotation at point 1 in degrees.
                    },
                    Pose2d
                    {
                        5.0_ft,				    // X ending position on field in feet.
                        17.5_ft,				// Y ending position on field in feet.
                        Rotation2d(-40_deg)		// Ending rotation on field in degrees.
                    }
                };

                m_bIsReversed = false;
                break;

            case eTakePowerCells1 :
                m_StartPoint =
                {
                    0.0_ft,					// X starting position on field in feet.
                    0.0_ft,					// Y starting position on field in feet.
                    Rotation2d(0_deg)		// Starting rotation on field in degrees.
                };

                m_Waypoints = 
                {
                    m_StartPoint,
                    Pose2d
                    {
                        10.5_ft,				// X of point 1 on field in feet.
                        10.5_ft,				// Y of point 1 on field in feet.
                        Rotation2d(-5_deg)      // Rotation at point 1 in degrees.
                    },
                    Pose2d
                    {
                        5.5_ft,				    // X ending position on field in feet.
                        19.0_ft,				// Y ending position on field in feet.
                        Rotation2d(-90_deg)		// Ending rotation on field in degrees.
                    }
                };

                m_bIsReversed = false;
                break;

            case eDoNothing :
                m_StartPoint =
                {
                    10.0_ft,				// X starting position on field in feet.
                    3.5_ft,					// Y starting position on field in feet.
                    Rotation2d(0_deg)		// Starting rotation on field in degrees.
                };

                m_Waypoints =
                {
                    m_StartPoint,
                    Pose2d
                    {
                        10.0_ft,				// X ending position on field in feet.
                        3.5_ft,					// Y ending position on field in feet.
                        Rotation2d(0_deg)		// Ending rotation on field in degrees.
                    }
                };

                m_bIsReversed = false;
                break;

            case eTestPath1 :
                m_StartPoint =
                {
                    0.0_ft,					// X starting position on field in feet.
                    1.0_ft,					// Y starting position on field in feet.
                    Rotation2d(0_deg)		// Starting rotation on field in degrees.
                };

                m_Waypoints =
                {
                    m_StartPoint,
                    Pose2d
                    {
                        2.0_ft,					// X of point 1 on field in feet.
                        1.0_ft,					// Y of point 1 on field in feet.
                        Rotation2d(0_deg)
                    },
                    Pose2d
                    {
                        10.0_ft,				// X of point 2 on field in feet.
                        1.0_ft,					// Y of point 2 on field in feet.
                        Rotation2d(0_deg)
                    },
                };

                m_bIsReversed = false;
                break;

            default :
                m_StartPoint =
                {
                    0.0_ft,					// X starting position on field in feet.
                    0.0_ft,					// Y starting position on field in feet.
                    Rotation2d(0_deg)		// Starting rotation on field in degrees.
                };

                m_Waypoints =
                {
                    m_StartPoint,
                    Pose2d
                    {
                        0.1_ft,				    // X ending position on field in feet.
                        0.1_ft,					// Y ending position on field in feet.
                        Rotation2d(0_deg)		// Ending rotation on field in degrees.
                    }
                };

                m_bIsReversed = false;
                break;
        }
    }

    // One-line methods.
    Pose2d GetSelectedTrajectoryStartPoint()    {   return m_StartPoint;    };
    vector<Pose2d> GetSelectedTrajectory()      {   return m_Waypoints;     };
    bool GetIsTrajectoryReversed()              {   return m_bIsReversed;   };

    // Configure trajectory properties.
    const meters_per_second_t kMaxSpeed = 2_mps;
    const meters_per_second_squared_t kMaxAcceleration = 3_mps_sq;

private:
    bool m_bIsReversed = false;

    Pose2d m_StartPoint
    {
        0.0_ft,
        0.0_ft,
        Rotation2d(0_deg)
    };

    vector<Pose2d> m_Waypoints
    {
        Pose2d
        {
            0.1_ft,
            0.1_ft,
            Rotation2d(0_deg)
        },
        Pose2d
        {
            0.2_ft,
            0.2_ft,
            Rotation2d(0_deg)
        }
    };
};
/////////////////////////////////////////////////////////////////////////////
#endif