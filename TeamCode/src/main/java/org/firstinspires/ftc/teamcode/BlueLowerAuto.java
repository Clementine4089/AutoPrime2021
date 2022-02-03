package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "BlueLowerAuto")
public class BlueLowerAuto extends AutoBasic1
{

    SampleMecanumDrive drive;
    CapstonePipeline.CapstonePosition capstonePosition;
    CapstoneDetectionCamera capstoneDetectionCamera;
    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(90)));

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
            GrabberMove(true, true);
            initArm();

            drive.followTrajectory(AutoPaths.RT_traj11);// drives little ways forward
            ArmMoveTo(aP);
            sleep(2500);
            drive.followTrajectory(AutoPaths.RT_traj12);// drive to goal
            sleep(800);
            GrabberMove(false, false);
            sleep(400);
            drive.followTrajectory(AutoPaths.RT_traj13);// drives away from the goal
            ArmMoveTo(0);
            drive.followTrajectory(AutoPaths.RT_traj16);//drives to the ducky wheel
            drive.followTrajectory(AutoPaths.RT_traj116);
            DuckyWheelMoveTo(0.35, 1200);
            IntakeMove(0.63);
            drive.followTrajectory(AutoPaths.RT_traj17);
            drive.followTrajectory(AutoPaths.RT_traj18);
            drive.followTrajectory(AutoPaths.RT_traj115);
            drive.followTrajectory(AutoPaths.RT_traj19);
            drive.followTrajectory(AutoPaths.RT_traj111);
            drive.followTrajectory(AutoPaths.RT_traj112);
            drive.followTrajectory(AutoPaths.RT_traj113);
            DuckyGrab(true);
            sleep(500);
            IntakeMove(0);
            ArmMoveTo(2900);
            drive.followTrajectory(AutoPaths.RT_traj114);
            sleep(200);
            GrabberMove(false, false);
            sleep(100);
            ArmMoveTo(0);
            drive.followTrajectory(AutoPaths.RT_traj14); // drives to park



            break;
        }


        requestOpModeStop();
    }
}
