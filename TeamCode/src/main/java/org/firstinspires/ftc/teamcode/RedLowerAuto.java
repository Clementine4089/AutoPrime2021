package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name=" RedLower Auto", group="Linear OpMode")
public class RedLowerAuto extends AutoBasic1 {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initTeam(true); // Red = true

        setupOpMode();

        driveForTicks(0.4, 0.0, 0, 300); //- left +right
        driveForTicks(0, -0.5, 0, -1500); //- left +right
        driveForTicks(-0.4, 0.0, 0, -300); //- left +right
        driveForTicks(-0.1, 0.0, 0, -100); //- left +right
        DuckyWheelMoveTo(5, 10000);
        driveForTicks(0.5, 0.0, 0, 650); //- left +right
        //driveForTicks(0.0, 0.2, 0, 100); //- left +right
    }


}