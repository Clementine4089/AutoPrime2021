package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueHigh Auto", group="Linear Opmode")
public class BlueHighAuto extends AutoBasic1 {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initTeam(false); // blue = false

        setupOpMode();

        driveForTicks(0.5, 0.0, 0, 1500); //- left +right

    }


}