


/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.config.Config;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.DigitalChannel;
        import com.qualcomm.robotcore.hardware.PIDFCoefficients;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.Servo;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BasicAuto", group="Linear OpMode")

@Config
public class AutoBasic1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorArm = null;
    private DcMotor motorDuckyWheel = null;
    private DcMotor motorIntake = null;
    private Servo servoGrabber = null;
    private Servo servoRetainer = null;
    private DigitalChannel limitSwitch = null;

    private BNO055IMU imu = null;
    private final boolean fieldCentric = true;

    private boolean Team = true; // false = blue, true = red
    //public static int level = 0; // 1 = high goal, 2 = middle goal, 3 = high goal
    //public static int testPos = 3400;

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        motorArm = hardwareMap.dcMotor.get("ArmMotor");
        motorDuckyWheel = hardwareMap.dcMotor.get("DuckyWheelMotor");
        motorIntake = hardwareMap.dcMotor.get("IntakeMotor");
        servoGrabber = hardwareMap.get(Servo.class, "GrabberServo");
        servoRetainer = hardwareMap.get(Servo.class, "GrabberServoNew");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "LimitSwitch0");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDuckyWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParms = new BNO055IMU.Parameters();
        imuParms.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(imuParms);

        waitForStart();
    }
    public void initTeam(boolean t){
        Team = t;
    }
    public void setupOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        motorArm = hardwareMap.dcMotor.get("ArmMotor");
        motorDuckyWheel = hardwareMap.dcMotor.get("DuckyWheelMotor");
        motorIntake = hardwareMap.dcMotor.get("IntakeMotor");
        servoGrabber = hardwareMap.get(Servo.class, "GrabberServo");
        servoRetainer = hardwareMap.get(Servo.class, "GrabberServoNew");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "LimitSwitch0");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        motorDuckyWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

    }
    public void ArmMoveTo(int position)
    {
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setTargetPosition(position);
        motorArm.setPower(0.8);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void ArmMove()
    {
        while (limitSwitch.getState())
        {
                motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorArm.setPower(-0.8);
        }
        motorArm.setPower(0);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
}

    public void DuckyWheelMoveTo(double speed, double ticks)
    {
        motorDuckyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean positiveDir = ticks > 0;
        double endTicks = motorDuckyWheel.getCurrentPosition() + ticks;
        while (opModeIsActive() &&
                ((positiveDir == true && motorDuckyWheel.getCurrentPosition() < endTicks) ||
                        (positiveDir == false && motorDuckyWheel.getCurrentPosition() > endTicks))) {
            ////////////////////// Ducky Wheel ///////////////////////////
                motorDuckyWheel.setPower(speed);

        }
        motorDuckyWheel.setPower(0);

    }
    void GrabberMove(boolean oc,boolean start)
    {
        ///////////////// Grabber ////////////////////
        if (oc == false)//grabber ungrab
        {
            servoGrabber.setPosition(0.63);

        }
        if (oc == true) //grapper grap and reverse intake
        {
            servoGrabber.setPosition(0.8);
            servoRetainer.setPosition(0);
            if(!start)
            {
                SpitOut();
            }
        }
    }
    void DuckyGrab(boolean grab ){
        if (grab == false)//grabber ungrab
        {
            servoGrabber.setPosition(0.63);

        }
        if (grab == true) //grapper grap and reverse intake
        {
            servoGrabber.setPosition(1);
            servoRetainer.setPosition(0);
        }
    }
    void SpitOut(){
        double ticks = 400;
        double speed = 1;
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean positiveDir = ticks > 0;
        double endTicks = motorIntake.getCurrentPosition() + ticks;
        while (opModeIsActive() &&
                ((positiveDir == true && motorIntake.getCurrentPosition() < endTicks) ||
                        (positiveDir == false && motorIntake.getCurrentPosition() > endTicks))) {
            ////////////////////// Ducky Wheel ///////////////////////////
            FtcDashboard.getInstance().getTelemetry().addData("Current Ticks", motorIntake.getCurrentPosition());
            FtcDashboard.getInstance().getTelemetry().addData("Target Ticks", endTicks);
            FtcDashboard.getInstance().getTelemetry().update();

            motorIntake.setPower(speed);

        }
        motorIntake.setPower(0);
    }
    void IntakeMove(double speed)
    {
        ////////////////////intake//////////////////////
        if (speed > 0) //Makes intake go forward
        {
            motorIntake.setPower(-speed);
            servoRetainer.setPosition(0.5);
        }
        else if (speed < 0) //Makes intake go rewerse
        {
            motorIntake.setPower(-speed);
        }
        else// stops intake
        {
            motorIntake.setPower(0);
        }
    }

    void initArm() //for initializing the arm encouders at the beginning of each auto
    {
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    int initLevel(int level)
    {
        int armPos = 0;
        if (level == 3){
            armPos = 3750;
        }
        else if (level == 2){
            armPos = 3450;
        }
        else if (level == 1){
            armPos = 2900;
        }
        else {
            //return testPos;
        }
        return armPos;
    }
}