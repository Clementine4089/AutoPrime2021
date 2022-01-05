package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "AutoPrime Mecanum TeleOp", group = "Linear Opmode")

public class MecanumTeleOp extends LinearOpMode
{
    public char team = '-';

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackRight = null;
    private DcMotor motorArm = null;
    private DcMotor motorDuckyWheel = null;
    private DcMotor motorIntake = null;
    private Servo servoGrabber = null;
    private DigitalChannel limitSwitch = null;

    boolean teamSelected = false;

    private boolean a1;
    private boolean b1;
    private boolean x1;
    private boolean y1;
    private boolean right_bumper1;
    private float right_trigger1;
    private boolean left_bumper1;
    private float left_trigger1;
    private double left_stick_y1; // Remember, this is reversed!
    private double left_stick_x1; // Counteract imperfect strafing
    private double right_stick_x1;

    private boolean a2;
    private boolean b2;
    private boolean x2;
    private boolean y2;
    private boolean right_bumper2;
    private float right_trigger2;
    private boolean left_bumper2;
    private float left_trigger2;
    private double left_stick_y2; // Remember, this is reversed!
    private double left_stick_x2; // Counteract imperfect strafing
    private double right_stick_x2;
    private boolean dpad_up;
    private boolean dpad_down;

    private BNO055IMU imu = null;
    private final boolean fieldCentric = true;

    public void HandleInput()
    {
        a1 = gamepad1.a;
        b1 = gamepad1.b;
        x1 = gamepad1.x;
        y1 = gamepad1.y;
        right_bumper1 = gamepad1.right_bumper;
        right_trigger1 = gamepad1.right_trigger;
        left_bumper1 = gamepad1.left_bumper;
        left_trigger1 = gamepad1.left_trigger;
        left_stick_y1 = -gamepad1.left_stick_y; // Remember, this is reversed!
        left_stick_x1 = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        right_stick_x1 = gamepad1.right_stick_x;
        dpad_up = gamepad2.dpad_up;
        dpad_down = gamepad2.dpad_down;

        a2 = gamepad2.a;
        b2 = gamepad2.b;
        x2 = gamepad2.x;
        y2 = gamepad2.y;
        right_bumper2 = gamepad2.right_bumper;
        right_trigger2 = gamepad2.right_trigger;
        left_bumper2 = gamepad2.left_bumper;
        left_trigger2 = gamepad2.left_trigger;
        left_stick_y2 = -gamepad2.left_stick_y;
        left_stick_x2 = gamepad2.left_stick_x;
        right_stick_x2 = gamepad2.right_stick_x;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = hardwareMap.dcMotor.get("LeftFrontMotor");
        motorBackLeft = hardwareMap.dcMotor.get("LeftBackMotor");
        motorFrontRight = hardwareMap.dcMotor.get("RightFrontMotor");
        motorBackRight = hardwareMap.dcMotor.get("RightBackMotor");
        motorArm = hardwareMap.dcMotor.get("ArmMotor");
        motorDuckyWheel = hardwareMap.dcMotor.get("DuckyWheelMotor");
        motorIntake = hardwareMap.dcMotor.get("IntakeMotor");
        servoGrabber = hardwareMap.get(Servo.class, "GrabberServo");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "LimitSwitch0");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
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

        InitArm(); //limit switch potentially broken
        //motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (!teamSelected && !isStopRequested())
        {
            HandleInput();
            SetTeam(x1, b1);
        }
        if (isStopRequested()) return;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParms = new BNO055IMU.Parameters();
        imuParms.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParms);

        waitForStart();
        runtime.reset();



        while (opModeIsActive())
        {
            HandleInput();
            MecanumDrive(left_stick_y1, left_stick_x1, right_stick_x1, right_bumper1, left_bumper1);
            Arm(a2, b2, x2, left_bumper2, right_bumper2, dpad_up, dpad_down);
            Grabber(right_bumper2,left_bumper2);
            Intake(right_trigger2, left_trigger2);
            DuckyWheel(left_stick_y2);
            SpinOneDuck();
            //OutTake(y2);
            telemetry.update();

        }
    }

    public void ArmMoveTo(int position)
    {
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setTargetPosition(position);
        motorArm.setPower(0.8);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void Arm(boolean a, boolean b, boolean x, boolean stop1, boolean stop2, boolean manualUp, boolean manualDn)
    {
        ///////////////// Arm //////////////////////////
        if (a)// arm moves to the high level
        {
            ArmMoveTo(3100);
        }
        if (b)// arm moves to the Low Level
        {
            ArmMoveTo(3800);
        }
        if (manualUp)
        {
            motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorArm.setPower(0.8);
        }
        else if (manualDn  && limitSwitch.getState())
        {
            motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorArm.setPower(-0.8);
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

    private void Grabber(boolean right_bumper, boolean left_bumper)
    {
        ///////////////// Grabber ////////////////////
        if (right_bumper)//grabber ungrab
        {
            servoGrabber.setPosition(0.55);
        }
        if (left_bumper)
        {
            servoGrabber.setPosition(0.65);
        }
    }

    private void Intake(float right_trigger, float left_trigger)
    {
        ////////////////////intake//////////////////////
        if (right_trigger >= 0.1) //Makes intake go forward
        {
            motorIntake.setPower(1);
        }
        else if (left_trigger >= 0.1) //Makes intake go rewerse
        {
            motorIntake.setPower(-1);
        }
        else
        {
            motorIntake.setPower(0);
        }
    }

    private void DuckyWheel(double input)
    {
        float multiplier = 5;
        ////////////////////// Ducky Wheel ///////////////////////////
        if (team == 'b')
            motorDuckyWheel.setPower(input * multiplier);
        else
            motorDuckyWheel.setPower(input * multiplier * -1);

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
            FtcDashboard.getInstance().getTelemetry().addData("Current Ticks", motorDuckyWheel.getCurrentPosition());
            FtcDashboard.getInstance().getTelemetry().addData("Target Ticks", endTicks);
            FtcDashboard.getInstance().getTelemetry().update();
            if (team == 'b')
                motorDuckyWheel.setPower(speed);
            else
                motorDuckyWheel.setPower(speed * -1);
        }
        motorDuckyWheel.setPower(0);

    }
    public void SpinOneDuck(){

        if(gamepad2.y){
            if (team == 'r')
            {
                DuckyWheelMoveTo(0.6, -675); // one rot =103.6
                DuckyWheelMoveTo(1, -325);
            }
            else{
                DuckyWheelMoveTo(0.6, 675); // one rot =103.6
                DuckyWheelMoveTo(1, 325);
            }
        }


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
        if (fieldCentric && team == 'b') {
            double angle = -imu.getAngularOrientation().firstAngle-Math.PI/2;
            telemetry.addData("Heading", "%f", angle);

            // From https://www.ctrlaltftc.com/practical-examples/drivetrain-control
            double x_rotated = x * Math.cos(angle) - y * Math.sin(angle);
            double y_rotated = x * Math.sin(angle) + y * Math.cos(angle);
            x = x_rotated;
            y = y_rotated;
        }
        else if (fieldCentric){
            double angle = -imu.getAngularOrientation().firstAngle;
            telemetry.addData("Heading", "%f", angle);

            // From https://www.ctrlaltftc.com/practical-examples/drivetrain-control
            double x_rotated = x * Math.cos(angle) - y * Math.sin(angle);
            double y_rotated = x * Math.sin(angle) + y * Math.cos(angle);
            x = x_rotated;
            y = y_rotated;
        }
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator) * speedMultiplier;
        double backLeftPower = ((y - x + rx) / denominator) * speedMultiplier;
        double frontRightPower = ((y + x - rx) / denominator) * speedMultiplier;
        double backRightPower = ((y - x - rx) / denominator) * speedMultiplier;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);

        double pos1 = servoGrabber.getPosition();
        telemetry.addData("Test","1, position: 1-%f", pos1);
        telemetry.update();
    }
    private void SetTeam(boolean x, boolean b)
    {
        telemetry.addData("Driver, please select a team. Current team: ", "%c", team);
        telemetry.update();
        if (x)
        {
            team = 'b';
            telemetry.addData("Team Selected. Current team: ", "%c", team);
            telemetry.update();
            teamSelected = true;
        }
        else if (b)
        {
            team = 'r';
            telemetry.addData("Team Selected. Current team: ", "%c", team);
            telemetry.update();
            teamSelected = true;
        }


    }


}

