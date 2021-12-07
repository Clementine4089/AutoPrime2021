package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "RedLowerAuto")
public class RedLowerAuto extends AutoBasic1
{

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(270)));
        waitForStart();

        while (opModeIsActive() && !isStopRequested())
        {
            initTeam(true);

            setupOpMode();

            GrabberMove(true);
            drive.followTrajectory(AutoPaths.RT_traj1);
            ArmMoveTo(3100);
            drive.followTrajectory(AutoPaths.RT_traj2);
            sleep(1500);
            GrabberMove(false);
            IntakeMove(1);
            sleep(500);
            IntakeMove(0);
            ArmMoveTo(0);
            drive.followTrajectory(AutoPaths.RT_traj3);
            DuckyWheelMoveTo(5, 10000);
            drive.followTrajectory(AutoPaths.RT_traj4);
            drive.followTrajectory(AutoPaths.RT_traj5);


            break;
        }


        requestOpModeStop();
    }
}