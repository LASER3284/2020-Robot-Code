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
/////////////////////////////////////////////////////////////////////////////

class CTrajectoryConstants
{
public:
    void SelectTrajectory(int nTrajectory)
    {
        // Retrieve the correct trajectory.
        switch(nTrajectory)
        {
            case eAllianceTrench :
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
                        34.5_ft,				// X of point 2 on field in feet.
                        24.5_ft,				// Y of point 2 on field in feet.
                        Rotation2d(0_deg)       // Rotation at point 2 in degrees.
                    },
                    Pose2d
                    {
                        27.0_ft,				// X of point 3 on field in feet.
                        24.5_ft,				// Y of point 3 on field in feet.
                        Rotation2d(0_deg)       // Rotation at point 3 in degrees.
                    },
                    Pose2d
                    {
                        7.0_ft,				    // X ending position on field in feet.
                        19.0_ft,				// Y ending position on field in feet.
                        Rotation2d(0_deg)		// Ending rotation on field in degrees.
                    }
                };
                break;

            case eFrontShieldGenerator :
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
                        Rotation2d(20_deg)     // Rotation at point 1 in degrees.
                    },
                    Pose2d
                    {
                        8.0_ft,				    // X ending position on field in feet.
                        19.0_ft,				// Y ending position on field in feet.
                        Rotation2d(0_deg)		// Ending rotation on field in degrees.
                    }
                };
                break;

            case eSideShieldGenerator :
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
                    },
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
                break;

            case eOpposingTrench :
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
                        21.0_ft,				// X of point 1 on field in feet.
                        2.5_ft,				    // Y of point 1 on field in feet.
                        Rotation2d(-90_deg)     // Rotation at point 1 in degrees.
                    },
                    Pose2d
                    {
                        8.5_ft,				    // X of point 2 on field in feet.
                        10.5_ft,				// Y of point 2 on field in feet.
                        Rotation2d(90_deg)      // Rotation at point 2 in degrees.
                    },
                    Pose2d
                    {
                        6.5_ft,				    // X ending position on field in feet.
                        17.0_ft,				// Y ending position on field in feet.
                        Rotation2d(140_deg)		// Ending rotation on field in degrees.
                    }
                };
                break;

            case ePowerPort :
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
                break;

            case eTakePowerCells :
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
                break;

            case eTestPath :
                m_StartPoint =
                {
                    1.0_ft,					// X starting position on field in feet.
                    1.0_ft,					// Y starting position on field in feet.
                    Rotation2d(0_deg)		// Starting rotation on field in degrees.
                };

                m_Waypoints =
                {
                    m_StartPoint,
                    Pose2d
                    {
                        7.5_ft,					// X of point 1 on field in feet.
                        7.0_ft,					// Y of point 1 on field in feet.
                        Rotation2d(0_deg)
                    },
                    Pose2d
                    {
                        11.0_ft,				// X of point 2 on field in feet.
                        4.5_ft,					// Y of point 2 on field in feet.
                        Rotation2d(-90_deg)
                    },
                    Pose2d
                    {
                        8.0_ft,				    // X of point 2 on field in feet.
                        2.0_ft,					// Y of point 2 on field in feet.
                        Rotation2d(-180_deg)
                    },
                    Pose2d
                    {
                        1.0_ft,				    // X ending position on field in feet.
                        1.0_ft,					// Y ending position on field in feet.
                        Rotation2d(-180_deg)    // Ending rotation on field in degrees.
                    }
                };
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
                break;
        }
    }

    // One-line methods.
    vector<Pose2d> GetSelectedTrajectory()      {   return m_Waypoints;     };
    Pose2d GetSelectedTrajectoryStartPoint()    {   return m_StartPoint;    };

    // Configure trajectory properties.
    const meters_per_second_t kMaxSpeed = 1.5_mps;
    const meters_per_second_squared_t kMaxAcceleration = 1_mps_sq;

    enum TrajectoryList {eAllianceTrench = 1, eFrontShieldGenerator, eSideShieldGenerator, eOpposingTrench, ePowerPort, eTakePowerCells, eDoNothing, eTestPath};

private:
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
            0.0_ft,
            0.0_ft,
            Rotation2d(0_deg)
        },
        Pose2d
        {
            0.0_ft,
            0.0_ft,
            Rotation2d(0_deg)
        }
    };
};
/////////////////////////////////////////////////////////////////////////////
#endif