package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "BlueUpperAuto")
public class BlueUpperAuto extends AutoBasic1
{

    SampleMecanumDrive drive;
    CapstonePipeline.CapstonePosition capstonePosition;
    CapstoneDetectionCamera capstoneDetectionCamera;

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(255)));

        capstoneDetectionCamera = new CapstoneDetectionCamera(hardwareMap);

        while (!isStarted())
        {
            capstonePosition = capstoneDetectionCamera.getPosition();

            telemetry.addData("Current Position", capstonePosition);
            telemetry.addData("Left Analysis", capstoneDetectionCamera.getAnalysis()[0]);
            telemetry.addData("Center Analysis", capstoneDetectionCamera.getAnalysis()[1]);
            telemetry.addData("Right Analysis", capstoneDetectionCamera.getAnalysis()[2]);
            telemetry.update();
        }

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

        setupOpMode();

        waitForStart();

        while (opModeIsActive() && !isStopRequested())
        {
            int aP = 0;
            if (capstonePosition == CapstonePipeline.CapstonePosition.LEFT)
            {
                aP = initLevel(3); //low
            }

            else if (capstonePosition == CapstonePipeline.CapstonePosition.CENTER)
            {
                aP = initLevel(2); //middle
            }

            else if (capstonePosition == CapstonePipeline.CapstonePosition.RIGHT)
            {
                aP = initLevel(1); //high
            }
            while (opModeIsActive() && !isStopRequested())
            {
                initTeam(true);
                initArm();
                GrabberMove(true, true);

                /// step 1 ///

                drive.followTrajectory(AutoPaths.RT_traj31); //moves slightly towards the goal
                ArmMoveTo(aP);
                sleep(2000);
                drive.followTrajectory(AutoPaths.RT_traj32); //moves to the goal ready to place the element
                sleep(1000);
                GrabberMove(false, false);
                sleep(400);
                drive.followTrajectory(AutoPaths.RT_traj33); //drives to the wall
                ArmMoveTo(0);
                sleep(500);
                IntakeMove(1);
                sleep(200);
                drive.followTrajectory(AutoPaths.RT_traj34); //drives to the hub to pick up an element
                sleep(1000);
                IntakeMove(0);
                GrabberMove(true, false);

                /// step 2 ///
                drive.followTrajectory(AutoPaths.RT_traj35); //drives backwards out of the hub
                ArmMoveTo(3050);
                drive.followTrajectory(AutoPaths.RT_traj36); //drives to the goal ready to place the element
                sleep(800);
                GrabberMove(false, false);
                //sleep(300);
                ArmMoveTo(0);
                drive.followTrajectory(AutoPaths.RT_traj37); //drives to the wall
                IntakeMove(1);
                drive.followTrajectory(AutoPaths.RT_traj38); //drives to the hub ready to pick up an element
                sleep(2000);
                IntakeMove(0);
                GrabberMove(true, false);

                /// step 3 ///
                drive.followTrajectory(AutoPaths.RT_traj39); // drives backwards out of the hub
                ArmMoveTo(3050);
                drive.followTrajectory(AutoPaths.RT_traj310); // drives to the goal ready to place the element
                sleep(800);
                GrabberMove(false, false);
                //sleep(300);
                ArmMoveTo(0);
                drive.followTrajectory(AutoPaths.RT_traj311); //drives to the wall
                IntakeMove(1);
                drive.followTrajectory(AutoPaths.RT_traj312); //drives into the hub and parks.
                sleep(10000);
                break;
            }
            requestOpModeStop();
        }
    }
}