package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueLower Auto", group="Linear Opmode")
public class BlueLowerAuto extends AutoBasic1 {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initTeam(false); // blue = false

        setupOpMode();

        driveForTicks(0, 0.3, 0, 300); //- left +right
        driveForTicks(-0.3, 0, 0, -900); //- left +right
        DuckyWheelMoveTo(5, -5000);
        driveForTicks(0, 0.3, 0, 1000); //- left +right
        driveForTicks(-0.3, 0.0, 0, -300); //- left +right
    }


}