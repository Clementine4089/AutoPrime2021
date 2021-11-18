package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedHigh Auto", group="Linear Opmode")
public class RedHighAuto extends AutoBasic1 {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initTeam(true); // Red = true

        setupOpMode();

        driveForTicks(0.5, 0.0, 0, 1500); //- left +right

    }


}
