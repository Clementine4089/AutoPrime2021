package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "RedUpperAuto")
public class RedUpperAuto extends AutoBasic1
{

    SampleMecanumDrive drive;
    CapstonePipeline.CapstonePosition capstonePosition;
    CapstoneDetectionCamera capstoneDetectionCamera;

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(255)));

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

        waitForStart();

        while (opModeIsActive() && !isStopRequested())
        {
            int aP = 0;
            if(capstonePosition == CapstonePipeline.CapstonePosition.LEFT){
                aP = initLevel(3); //low
            }

            else if(capstonePosition == CapstonePipeline.CapstonePosition.CENTER){
                aP= initLevel(2); //middle
            }

            else if(capstonePosition == CapstonePipeline.CapstonePosition.RIGHT){
                aP= initLevel(1); //high
            }

            initTeam(true);
            setupOpMode();
            initArm();

            /// step 1 ///
            GrabberMove(true);
            drive.followTrajectory(AutoPaths.RT_traj21); //moves slightly towards the goal
            ArmMoveTo(aP); //3150
            drive.followTrajectory(AutoPaths.RT_traj22); //moves to the goal ready to place the element
            sleep(1700);
            GrabberMove(false);
            IntakeMove(-1);
            sleep(500);
            IntakeMove(0);
            ArmMoveTo(0);
            drive.followTrajectory(AutoPaths.RT_traj23); //drives to the wall
            IntakeMove(1);
            sleep(300);
            drive.followTrajectory(AutoPaths.RT_traj24); //drives to the hub to pick up an element
            sleep(1800);
            IntakeMove(0);
            GrabberMove(true);

            /// step 2 ///
            drive.followTrajectory(AutoPaths.RT_traj25); //drives backwards out of the hub
            ArmMoveTo(3150);
            drive.followTrajectory(AutoPaths.RT_traj26); //drives to the goal ready to place the element
            sleep(1200);
            GrabberMove(false);
            IntakeMove(-1);
            sleep(500);
            IntakeMove(0);
            ArmMoveTo(0);
            drive.followTrajectory(AutoPaths.RT_traj27); //drives to the wall
            IntakeMove(1);
            drive.followTrajectory(AutoPaths.RT_traj28); //drives to the hub ready to pick up an element
            drive.followTrajectory(AutoPaths.RT_traj213); //drives slower to be ready to pick up element
            sleep(2000);
            IntakeMove(0);
            GrabberMove(true);

            /// step 3 ///
            drive.followTrajectory(AutoPaths.RT_traj29); // drives backwards out of the hub
            ArmMoveTo(3150);
            drive.followTrajectory(AutoPaths.RT_traj210); // drives to the goal ready to place the element
            sleep(1200);
            GrabberMove(false);
            IntakeMove(-1);
            sleep(500);
            IntakeMove(0);
            ArmMoveTo(0);
            drive.followTrajectory(AutoPaths.RT_traj211); //drives to the wall
            IntakeMove(1);
            drive.followTrajectory(AutoPaths.RT_traj212); //drives into the hub and parks.

            break;
        }


        requestOpModeStop();
    }
}
