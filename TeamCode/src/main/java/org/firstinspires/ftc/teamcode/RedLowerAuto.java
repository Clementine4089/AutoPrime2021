package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "RedLowerAuto")
public class RedLowerAuto extends AutoBasic1
{

    SampleMecanumDrive drive;

    CapstonePipeline.CapstonePosition capstonePosition;
    CapstoneDetectionCamera capstoneDetectionCamera;

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(270)));

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

            GrabberMove(true);
            drive.followTrajectory(AutoPaths.RT_traj1);
            ArmMoveTo(aP);
            drive.followTrajectory(AutoPaths.RT_traj2);
            sleep(2000);
            GrabberMove(false);
            IntakeMove(-1);
            sleep(500);
            IntakeMove(0);
            ArmMoveTo(0);
            drive.followTrajectory(AutoPaths.RT_traj3);
            DuckyWheelMoveTo(-0.35, -1200);
            drive.followTrajectory(AutoPaths.RT_traj4);
            drive.followTrajectory(AutoPaths.RT_traj5);


            break;
        }


        requestOpModeStop();
    }
}