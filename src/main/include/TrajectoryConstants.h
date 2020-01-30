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

    // Start pose for robot field position.
    Pose2d m_StartPoint
    {
        0.0_ft,					// X starting position on field in feet.
        0.0_ft,					// Y starting position on field in feet.
        Rotation2d(0_deg)		// Starting rotation on field in degrees.
    };

    // End pose.
    Pose2d m_EndPoint
    {
        14.0_ft,				// X ending position on field in feet.
        7.0_ft,					// Y ending position on field in feet.
        Rotation2d(105_deg)		// Ending rotation on field in degrees.
    };
    
    // Vector including all poses and waypoints.
    vector<Pose2d>  m_InteriorWaypoints
    {
        m_StartPoint,
        Pose2d
        {
            6.0_ft,					// X of point 1 on field in feet.
            7.5_ft,					// Y of point 1 on field in feet.
            Rotation2d(0_deg)
        },
        Pose2d
        {
            11.0_ft,				// X of point 2 on field in feet.
            2.5_ft,					// Y of point 2 on field in feet.
            Rotation2d(0_deg)
        },
        m_EndPoint
    };

    // Configure trajectory properties.
    TrajectoryConfig m_Config
    {
        3_fps,					// Max desirable velocity.
        1_fps_sq				// Max desirable acceleration.
    };
};
/////////////////////////////////////////////////////////////////////////////
#endif