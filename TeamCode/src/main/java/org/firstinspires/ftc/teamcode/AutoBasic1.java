


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


public class AutoBasic1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackRight = null;
    private DcMotor motorArm = null;
    private DcMotor motorDuckyWheel = null;
    private CRServo servoIntake = null;
    private Servo servoGrabber = null;
    private DigitalChannel limitSwitch = null;

    private BNO055IMU imu = null;
    private final boolean fieldCentric = true;

    private boolean Team = true; // false = blue, true = red

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFrontLeft = hardwareMap.dcMotor.get("LeftFrontMotor");
        motorBackLeft = hardwareMap.dcMotor.get("LeftBackMotor");
        motorFrontRight = hardwareMap.dcMotor.get("RightFrontMotor");
        motorBackRight = hardwareMap.dcMotor.get("RightBackMotor");
        motorArm = hardwareMap.dcMotor.get("ArmMotor");
        motorDuckyWheel = hardwareMap.dcMotor.get("DuckyWheelMotor");
        servoIntake = hardwareMap.get(CRServo.class, "IntakeServo");
        servoGrabber = hardwareMap.get(Servo.class, "GrabberServo");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "LimitSwitch0");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDuckyWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParms = new BNO055IMU.Parameters();
        imuParms.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(imuParms);

        waitForStart();
//            // InitArm();
//            driveForTicks(0.3, 0.0, 0, 300); //- left +right
//            driveForTicks(0, -0.3, 0, -1500); //- left +right
//            driveForTicks(-0.3, 0.0, 0, -200); //- left +right
//            DuckyWheelMoveTo(5, 5000);
//            driveForTicks(0.3, 0.0, 0, 700); //- left +right
//            //movebackforw()
            //moveTo
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
        motorFrontLeft = hardwareMap.dcMotor.get("LeftFrontMotor");
        motorBackLeft = hardwareMap.dcMotor.get("LeftBackMotor");
        motorFrontRight = hardwareMap.dcMotor.get("RightFrontMotor");
        motorBackRight = hardwareMap.dcMotor.get("RightBackMotor");
        motorArm = hardwareMap.dcMotor.get("ArmMotor");
        motorDuckyWheel = hardwareMap.dcMotor.get("DuckyWheelMotor");
        servoIntake = hardwareMap.get(CRServo.class, "IntakeServo");
        servoGrabber = hardwareMap.get(Servo.class, "GrabberServo");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "LimitSwitch0");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDuckyWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParms = new BNO055IMU.Parameters();
        imuParms.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(imuParms);

        waitForStart();
//

    }
    public void driveForTime(double y, double x, double rx, double time)
    {
        runtime.reset();

        while(opModeIsActive() && runtime.seconds() < time)
        {
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontDrivePower = (y + x + rx) / denominator;
            double leftRearDrivePower = (y - x + rx) / denominator;
            double rightFrontDrivePower = (y + x - rx) / denominator;
            double rightRearDrivePower = (y - x - rx) / denominator;

            motorFrontLeft.setPower(leftFrontDrivePower);
            motorBackLeft.setPower(leftRearDrivePower);
            motorFrontRight.setPower(rightFrontDrivePower);
            motorBackRight.setPower(rightRearDrivePower);
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    public void driveForTicks(double y, double x, double rx, double ticks) {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean positiveDir = ticks > 0;
        double endTicks = motorFrontLeft.getCurrentPosition() + ticks;

        while (opModeIsActive() &&
                ((positiveDir == true && motorFrontLeft.getCurrentPosition() < endTicks) ||
                        (positiveDir == false && motorFrontLeft.getCurrentPosition() > endTicks))) {
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontDrivePower = (y + x + rx) / denominator;
            double leftRearDrivePower = (y - x + rx) / denominator;
            double rightFrontDrivePower = (y + x - rx) / denominator;
            double rightRearDrivePower = (y - x - rx) / denominator;

            motorFrontLeft.setPower(leftFrontDrivePower);
            motorBackLeft.setPower(leftRearDrivePower);
            motorFrontRight.setPower(rightFrontDrivePower);
            motorBackRight.setPower(rightRearDrivePower);
            telemetry.addData("Drive", "Motor Pos, Ticks: %d, %f", motorFrontLeft.getCurrentPosition(), endTicks);
            telemetry.update();
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
    public void ArmMoveTo(int position)
    {
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setTargetPosition(position);
        motorArm.setPower(0.5);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void InitArm()
    {
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArm.setPower(-1);

        while (limitSwitch.getState() && !opModeIsActive())
        {
            telemetry.addData("Init arm", "Switch value: %s",limitSwitch.getState() );
            telemetry.addData("Init arm ", "Position: %d", motorArm.getCurrentPosition());
            telemetry.addData("Init arm ", "Time: %f", getRuntime());
            telemetry.update();
        }
        //hi
        //temp
        motorArm.setPower(0);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void Arm(boolean a, boolean b, boolean x, boolean stop1, boolean stop2, boolean manualUp, boolean manualDn)
    {
        ///////////////// Arm //////////////////////////
        if (a)// arm moves to the high level
        {
            ArmMoveTo(2000);
        }
        if (b)// arm moves to the middle level
        {
            ArmMoveTo(1000);
        }
//        if (x)//arm moves back to start
//        {
//            motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            if(limitSwitch.getState())
//            {
//                motorArm.setPower(-0.5);
//                motorArm.setMode(DcMotor.RunMode.RESET_ENCODERS);
//
//            }
//
//            motorArm.setPower(0);
//            motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
        if (manualUp)
        {
            motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorArm.setPower(0.5);
        }
        else if (manualDn  && limitSwitch.getState())
        {
            motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorArm.setPower(-0.5);
        }
        else if (!limitSwitch.getState())
        {
            motorArm.setPower(0);
            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (stop1 || stop2)
        {
            motorArm.setPower(0);
        }

        telemetry.addData("Arm ", "Position: %d", motorArm.getCurrentPosition());

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
            if (Team)
                motorDuckyWheel.setPower(speed);
            else
                motorDuckyWheel.setPower(speed * -1);
        }
        motorDuckyWheel.setPower(0);

    }
    private void MecanumDrive(double y, double x, double rx, boolean spdUp, boolean spdDn)
    {
        ////////////////Mecanum Drive ///////////////////////////
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        float speedMultiplier = 0.5f;
        if (spdUp)
        {
            speedMultiplier = 1;
        }
        else if (spdDn)
        {
            speedMultiplier = 0.25f;
        }
        if (fieldCentric) {
            double angle = -imu.getAngularOrientation().firstAngle;
            telemetry.addData("Heading", "%f", angle);

            // From https://www.ctrlaltftc.com/practical-examples/drivetrain-control
            double x_rotated = x * Math.cos(angle) - y * Math.sin(angle);
            double y_rotated = x * Math.sin(angle) + y * Math.cos(angle);
            x = x_rotated;
            y = y_rotated;
        }
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator)*speedMultiplier;
        double backLeftPower = ((y - x + rx) / denominator)*speedMultiplier;
        double frontRightPower = ((y + x - rx) / denominator)*speedMultiplier;
        double backRightPower = ((y - x - rx) / denominator)*speedMultiplier;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);

        double pos1 = servoGrabber.getPosition();
        telemetry.addData("Test","1, position: 1-%f", pos1);
    }
    private void Grabber(boolean right_bumper)
    {
        ///////////////// Grabber ////////////////////
        if (right_bumper)//grabber ungrab
        {
            servoGrabber.setPosition(0);
        }
        else //grabber grab
        {
            servoGrabber.setPosition(0.2);
        }
    }

    private void Intake(float right_trigger, float left_trigger)
    {
        ////////////////////intake//////////////////////
        if (right_trigger >= 0.1) //Makes intake go forward
        {
            servoIntake.setPower(1);
        }
        else if (left_trigger >= 0.1) //Makes intake go rewerse
        {
            servoIntake.setPower(-1);
        }
        else
        {
            servoIntake.setPower(0);
        }
    }

}