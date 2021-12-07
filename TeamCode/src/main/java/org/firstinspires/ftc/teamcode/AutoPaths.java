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



          |
          |
    <----------->






*/

public class AutoPaths extends AutoBasic1
{
    public static  Trajectory RT_traj1 = BuildTrajectory(new Pose2d(0,0, Math.toRadians(270)))
            .lineToSplineHeading(new Pose2d(0, 10, Math.toRadians(270))
            )
            .build();
    public static  Trajectory RT_traj2 = BuildTrajectory(new Pose2d(0,10, Math.toRadians(270)))
            .lineToSplineHeading(new Pose2d(0, 26, Math.toRadians(225))
            )
            .build();
    public static  Trajectory RT_traj3 = BuildTrajectory(new Pose2d(0,26, Math.toRadians(225)))
            .lineToSplineHeading(new Pose2d(-35, 4, Math.toRadians(100))
            )
            .build();
    public static  Trajectory RT_traj4 = BuildTrajectory(new Pose2d(-35,4, Math.toRadians(100)))
            .lineToSplineHeading(new Pose2d(-35, 24, Math.toRadians(100))
            )
            .build();
    public static  Trajectory RT_traj5 = BuildTrajectory(new Pose2d(-35,24, Math.toRadians(100)))
            .lineToSplineHeading(new Pose2d(-40, 24, Math.toRadians(100))
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
