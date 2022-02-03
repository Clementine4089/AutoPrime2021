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
            GrabberMove(true, true);

            drive.followTrajectory(AutoPaths.RT_traj1);// drives little ways off the wall
            ArmMoveTo(aP);
            sleep(1000);
            drive.followTrajectory(AutoPaths.RT_traj2); // drives to the goal ready to drop off an element
            sleep(400);
            GrabberMove(false, false);
            sleep(300);
            drive.followTrajectory(AutoPaths.RT_traj3);// moves little ways then moves towards teh ducky wheel
            ArmMove();
            drive.followTrajectory(AutoPaths.RT_traj6); // moves up to the ducky wheel
            DuckyWheelMoveTo(-0.35, -1200);
            IntakeMove(0.63);
            drive.followTrajectory(AutoPaths.RT_traj06);
            drive.followTrajectory(AutoPaths.RT_traj7); //spins around, ready to pick up the duck
            drive.followTrajectory(AutoPaths.RT_traj8);// drives up
            drive.followTrajectory(AutoPaths.RT_traj04); // moves toward wall
            drive.followTrajectory(AutoPaths.RT_traj9); //drives down
            drive.followTrajectory(AutoPaths.RT_traj05); //moves toward wall
            drive.followTrajectory(AutoPaths.RT_traj01); //drives up closer to wall, hopefully has a duck
            IntakeMove(0.9);
            sleep(1200);
            DuckyGrab(true);
            //sleep(400);
            IntakeMove(0);
            ArmMoveTo(2900);
            drive.followTrajectory(AutoPaths.RT_traj02);// drives up to the goal to drop of duck
            sleep(400);
            GrabberMove(false, false);
            sleep(300);
            drive.followTrajectory(AutoPaths.RT_traj03);// drives to park
            ArmMove();
            drive.followTrajectory(AutoPaths.RT_traj4); // drives to the wall
           // drive.followTrajectory(AutoPaths.RT_traj5); //drives against the wall to line up field centric


            break;
        }


        requestOpModeStop();
    }
}