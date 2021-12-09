package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import java.util.Arrays;
/*
              warehouses
                  /\x
                   |
                   |
 blue side y<-----------> -y red side
                   |
                   |
                  \/-x
*/

public class AutoPaths extends AutoBasic1
{
    //////////// RedLower Auto ///////////////
    public static  Trajectory RT_traj1 = BuildTrajectory(new Pose2d(0,0, Math.toRadians(270)))
            .lineToSplineHeading(new Pose2d(0, 10, Math.toRadians(270))
            )
            .build();
    public static  Trajectory RT_traj2 = BuildTrajectory(RT_traj1.end())
            .lineToSplineHeading(new Pose2d(0, 24, Math.toRadians(225))
            )
            .build();
    public static  Trajectory RT_traj3 = BuildTrajectory(RT_traj2.end())
            .lineToSplineHeading(new Pose2d(-35, 4, Math.toRadians(100))
            )
            .build();
    public static  Trajectory RT_traj4 = BuildTrajectory(RT_traj3.end())
            .lineToSplineHeading(new Pose2d(-35, 24, Math.toRadians(100))
            )
            .build();
    public static  Trajectory RT_traj5 = BuildTrajectory(RT_traj4.end())
            .lineToSplineHeading(new Pose2d(-40, 24, Math.toRadians(100))
            )
            .build();


    ////////// BlueLower Auto ////////
    public static  Trajectory RT_traj11 = BuildTrajectory(new Pose2d(0,0, Math.toRadians(90)))
            .lineToSplineHeading(new Pose2d(0, -10, Math.toRadians(90))
            )
            .build();
    public static  Trajectory RT_traj12 = BuildTrajectory(RT_traj11.end())
            .lineToSplineHeading(new Pose2d(0, -24, Math.toRadians(135))
            )
            .build();
    public static  Trajectory RT_traj13 = BuildTrajectory(RT_traj12.end())
            .lineToSplineHeading(new Pose2d(-35, -4, Math.toRadians(0))
            )
            .build();
    public static  Trajectory RT_traj14 = BuildTrajectory(RT_traj13.end())
            .lineToSplineHeading(new Pose2d(-35, -28, Math.toRadians(0))
            )
            .build();
    public static  Trajectory RT_traj15 = BuildTrajectory(RT_traj14.end())
            .lineToSplineHeading(new Pose2d(-45, -28, Math.toRadians(0))
            )
            .build();


    ////////// RedUpper Auto ////////
    public static  Trajectory RT_traj21 = BuildTrajectory(new Pose2d(0,0, Math.toRadians(270)))
            .lineToSplineHeading(new Pose2d(0, 10, Math.toRadians(270))
            )
            .build();
    public static  Trajectory RT_traj22 = BuildTrajectory(RT_traj21.end())
            .lineToSplineHeading(new Pose2d(0, 22, Math.toRadians(315))
            )
            .build();
    public static  Trajectory RT_traj23 = BuildTrajectory(RT_traj22.end())
            .lineToSplineHeading(new Pose2d(5, -10, Math.toRadians(360))
            )
            .build();
    public static  Trajectory RT_traj24 = BuildTrajectory(RT_traj23.end())
            .lineToSplineHeading(new Pose2d(40, -15, Math.toRadians(360))
            )
            .build();
    public static  Trajectory RT_traj25 = BuildTrajectory(RT_traj24.end())
            .lineToSplineHeading(new Pose2d(0, -15, Math.toRadians(360))
            )
            .build();
    public static  Trajectory RT_traj26 = BuildTrajectory(RT_traj25.end())
            .lineToSplineHeading(new Pose2d(0, 12, Math.toRadians(315))
            )
            .build();
    public static  Trajectory RT_traj27 = BuildTrajectory(RT_traj26.end())
            .lineToSplineHeading(new Pose2d(0, -15, Math.toRadians(360))
            )
            .build();
    public static  Trajectory RT_traj28 = BuildTrajectory(RT_traj27.end())
            .lineToSplineHeading(new Pose2d(42, -20, Math.toRadians(360))
            )
            .build();
    public static  Trajectory RT_traj29 = BuildTrajectory(RT_traj28.end())
            .lineToSplineHeading(new Pose2d(0, -20, Math.toRadians(360))
            )
            .build();
    public static  Trajectory RT_traj210 = BuildTrajectory(RT_traj29.end())
            .lineToSplineHeading(new Pose2d(0, 10, Math.toRadians(315))
            )
            .build();
    public static  Trajectory RT_traj211 = BuildTrajectory(RT_traj210.end())
            .lineToSplineHeading(new Pose2d(0, -20, Math.toRadians(360))
            )
            .build();
    public static  Trajectory RT_traj212 = BuildTrajectory(RT_traj211.end())
            .lineToSplineHeading(new Pose2d(45, -25, Math.toRadians(360))
            )
            .build();


    ////////// BlueUpper Auto ////////
    public static  Trajectory RT_traj31 = BuildTrajectory(new Pose2d(0,0, Math.toRadians(90)))
            .lineToSplineHeading(new Pose2d(0, -10, Math.toRadians(90))
            )
            .build();
    public static  Trajectory RT_traj32 = BuildTrajectory(RT_traj31.end())
            .lineToSplineHeading(new Pose2d(0, -22, Math.toRadians(45))
            )
            .build();
    public static  Trajectory RT_traj33 = BuildTrajectory(RT_traj32.end())
            .lineToSplineHeading(new Pose2d(5, 10, Math.toRadians(0))
            )
            .build();
    public static  Trajectory RT_traj34 = BuildTrajectory(RT_traj33.end())
            .lineToSplineHeading(new Pose2d(45, 10, Math.toRadians(0))
            )
            .build();
    public static  Trajectory RT_traj35 = BuildTrajectory(RT_traj34.end())
            .lineToSplineHeading(new Pose2d(0, 10, Math.toRadians(0))
            )
            .build();
    public static  Trajectory RT_traj36 = BuildTrajectory(RT_traj35.end())
            .lineToSplineHeading(new Pose2d(0, -12, Math.toRadians(45))
            )
            .build();
    public static  Trajectory RT_traj37 = BuildTrajectory(RT_traj36.end())
            .lineToSplineHeading(new Pose2d(5, 10, Math.toRadians(0))
            )
            .build();
    public static  Trajectory RT_traj38 = BuildTrajectory(RT_traj37.end())
            .lineToSplineHeading(new Pose2d(47, 10, Math.toRadians(0))
            )
            .build();
    public static  Trajectory RT_traj39 = BuildTrajectory(RT_traj38.end())
            .lineToSplineHeading(new Pose2d(0, 10, Math.toRadians(0))
            )
            .build();
    public static  Trajectory RT_traj310 = BuildTrajectory(RT_traj39.end())
            .lineToSplineHeading(new Pose2d(0, -12, Math.toRadians(45))
            )
            .build();
    public static  Trajectory RT_traj311 = BuildTrajectory(RT_traj310.end())
            .lineToSplineHeading(new Pose2d(5, 1, Math.toRadians(0))
            )
            .build();
    public static  Trajectory RT_traj312 = BuildTrajectory(RT_traj311.end())
            .lineToSplineHeading(new Pose2d(47, 10, Math.toRadians(0))
            )
            .build();



    public static TrajectoryBuilder BuildTrajectory(Pose2d startPose)
    {
        return new TrajectoryBuilder(startPose, new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        )), new ProfileAccelerationConstraint(MAX_ACCEL));
    }
}
