package org.firstinspires.ftc.teamcode.hardware;

import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Onbot_HardwareITD {

    // DEFINE OPMORE MEMBERS
    private LinearOpMode myOpMode;

    // ACCESS INSxTRUMENTS OF HUB
    public IMU imu;
    Orientation angle;
    double botHeading;

    // SENSORS

    // MOTOR DECLARATIONS - MOVEMENT
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    // MOTOR DECLARATIONS - SUBSYSTEMS
    public DcMotorEx lift1 = null;
    public DcMotorEx intake = null;
    public DcMotorEx horizontal = null;

    // SERVOS
    public Servo claw = null;
    public Servo arm1 = null;
    public Servo arm2 = null;
    public Servo intakeFlip1 = null;
    public Servo intakeFlip2 = null;


    // MOTOR POWERS
    public static double MAX_POWER = 1;
    public static double STRAFE_GAIN = 1.5;
    public static double COUNTS_PER_MOTOR_REV = 537.7; // 28 for REV ;
    public static double DRIVE_GEAR_REDUCTION = 1.0; //   12 for REV;
    public static double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static double ROBOT_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // SLIDES POSITIONS
    public int lift1_up = 1500;
    public int lift1_down = 0;
    public int lift1Current = 0;

    public final int top = 8300;
    public final int mid = 2500;
    public int vertCurrent = 0;
    public final int horiMax = 5000;
    public int horiCurrent = 0;

    // SERVO POSITIONS
    public final double claw_open = 0.66;
    public final double claw_closed = 0.3;

    public final double arm1_out = 0.33;
    public final double arm1_in = 0.496;

    public final double arm2_out = 0.67;
    public final double arm2_in = 0.5;

    public double arm1_current = arm1_in;
    public double arm2_current = arm2_in;

    public final double intakeFlip1_down = 0.416;

    public final double intakeFlip1_up = 0.53;

    public final double kp = 0.03;
    public final double errorMargin = 5;


    //TOGGLES
    private String intakeMode = "forward";
    private boolean intakeModeToggle = false;
    private boolean intakeToggle = false;
    private boolean intakeOn = false;
    private boolean armToggle = false;
    private boolean clawToggle = false;
    private boolean liftToggle = false;
    private boolean intakeDownToggle = false;

    public void HardwareTestbot() {
    }

    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {
        myOpMode = opMode;

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //motors
        frontLeft = myOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = myOpMode.hardwareMap.get(DcMotor.class, "backRight");
        lift1 = myOpMode.hardwareMap.get(DcMotorEx.class, "lift");
        intake = myOpMode.hardwareMap.get(DcMotorEx.class, "intake");
        horizontal = myOpMode.hardwareMap.get(DcMotorEx.class, "horizontal");


        //servos
        arm1 = myOpMode.hardwareMap.get(Servo.class, "arm1");
        arm2 = myOpMode.hardwareMap.get(Servo.class, "arm2");
        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        intakeFlip1 = myOpMode.hardwareMap.get(Servo.class, "intakeFlip1");
        intakeFlip2 = myOpMode.hardwareMap.get(Servo.class, "intakeFlip2");
        encoderState("reset");
        encoderState("off");

        // BRAKES THE MOTORS
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        lift1.setDirection(DcMotor.Direction.FORWARD);
        horizontal.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intakeFlip2.setDirection(Servo.Direction.REVERSE);

        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setTargetPosition(0);
        horizontal.setTargetPosition(0);


        // MOTOR POWERS
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // SERVO INITIALIZE *********************************************************************************************************
        arm1.setPosition(arm1_in);
        arm2.setPosition(arm2_in);
        claw.setPosition(claw_open);
        intakeFlip1.setPosition(intakeFlip1_up);
        intakeFlip2.setPosition(intakeFlip1_up);


    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void roboCentric(double forw, double side, double spin) {
        encoderState("off");
        double denominator = Math.max(Math.abs(forw) + Math.abs(side) + Math.abs(spin), 1);
        double FLPow = (forw + side + spin) / denominator;
        double FRPow = (forw - side - spin) / denominator;
        double BLPow = (forw - side + spin) / denominator;
        double BRPow = (forw + side - spin) / denominator;

        // Set drive motor power levels.
        frontLeft.setPower(FLPow * MAX_POWER);
        frontRight.setPower(FRPow * MAX_POWER);
        backLeft.setPower(BLPow * MAX_POWER / 2); //front two motors geared 1:2
        backRight.setPower(BRPow * MAX_POWER / 2);

        while (frontLeft.isBusy() || backLeft.isBusy() || frontRight.isBusy() || backRight.isBusy()) {
        }

    }

    public void fieldCentric(double y, double x, double rx) {

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(botHeading) - y * sin(botHeading);
        double rotY = x * sin(botHeading) + y * Math.cos(botHeading);

        rotX = rotX * STRAFE_GAIN;  // Counteract imperfect strafing


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (-rotY + rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX - rx) / denominator;
        double frontRightPower = (-rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX + rx) / denominator;

        // Set drive motor power levels.
        frontLeft.setPower(frontLeftPower * MAX_POWER);
        frontRight.setPower(frontRightPower * MAX_POWER);
        backLeft.setPower(backLeftPower * MAX_POWER);
        backRight.setPower(backRightPower * MAX_POWER);
    }

    public void distanceDrive(double forMovement, double latMovement, double turn, double speed) {
        int y = (int) (forMovement * COUNTS_PER_INCH);
        int x = (int) (latMovement * COUNTS_PER_INCH);
        turn = ROBOT_DIAMETER_INCHES * Math.PI * (turn / 180.0) * Math.PI / 6.5;
        int rx = (int) (turn * COUNTS_PER_INCH);

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


        rotX = rotX * STRAFE_GAIN;  // Counteract imperfect strafing


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        int frontleftTargetPos = (int) (rotY - rotX - rx);
        int backleftTargetPos = (int) (-rotY + rotX - rx);
        int frontrightTargetPos = (int) (rotY + rotX + rx);
        int backrightTargetPos = (int) (-rotY - rotX + rx);

        frontLeft.setTargetPosition(frontleftTargetPos);
        frontRight.setTargetPosition(frontrightTargetPos);
        backLeft.setTargetPosition(backleftTargetPos);
        backRight.setTargetPosition(backrightTargetPos);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

    }

    public void mechEN(double forMovement, double latMovement, double turn, double speed) {
        int forwardSteps = (int) (forMovement * COUNTS_PER_INCH);
        int sideSteps = (int) (latMovement * COUNTS_PER_INCH);
        turn = ROBOT_DIAMETER_INCHES * Math.PI * (turn / 180.0) * Math.PI / 6.5;
        int turnSteps = (int) (turn * COUNTS_PER_INCH);


        int frontleftTargetPos = frontLeft.getCurrentPosition() + (int) (forwardSteps - sideSteps - turnSteps);
        int frontrightTargetPos = frontRight.getCurrentPosition() + (int) (forwardSteps + sideSteps + turnSteps);
        int backleftTargetPos = backLeft.getCurrentPosition() + (int) (-forwardSteps + sideSteps - turnSteps);
        int backrightTargetPos = backRight.getCurrentPosition() + (int) (-forwardSteps - sideSteps + turnSteps);

        frontLeft.setTargetPosition(frontleftTargetPos);
        frontRight.setTargetPosition(frontrightTargetPos);
        backLeft.setTargetPosition(backleftTargetPos);
        backRight.setTargetPosition(backrightTargetPos);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnToAngle(double angle) {
        //skibidi toilet - andrew
        encoderState("off");
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = botHeading - angle;
        while (Math.abs(error) > errorMargin && myOpMode.opModeIsActive()) {
            fieldCentric(0, 0, error * kp);
            error = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - angle;
        }
        encoderState("reset");
        encoderState("run");
    }

    public void encoderState(String a) {
        if (a.equals("reset")) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        } else if (a.equals("run")) {
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (a.equals("position")) {
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (a.equals("off")) {
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void intakeState(boolean a, boolean b) {
        if (a && !intakeModeToggle) {
            if (intakeMode.equals("forward")) {
                intakeMode = "reverse";
            } else {
                intakeMode = "forward";
            }
            intakeModeToggle = true;
        } else if (!a) {
            intakeModeToggle = false;
        }

        if (b && !intakeToggle) {
            if (intakeOn) {
                intakeOn = false;
            } else {
                intakeOn = true;
            }
            intakeToggle = true;
        } else if (!b) {
            intakeToggle = false;
        }

        if (intakeOn) {
            if (intakeMode.equals("forward")) {
                intake.setPower(1);
            } else {
                intake.setPower(-1);
            }
        } else {
            intake.setPower(0);
        }
    }

    public void intakeState(String a, boolean b) {
        if (b) {
            if (a.equals("forward")) {
                intakeMode = "forward";
                intake.setPower(1);
            } else {
                intakeMode = "reverse";
                intake.setPower(-1);
            }
        } else {
            intake.setPower(0);
        }
    }

    public void horizontalSys(double slide, boolean intakeUp) {
        if (slide > 0.1 && horizontal.getTargetPosition() + 15 < horiMax && Math.abs(horizontal.getTargetPosition() - horizontal.getCurrentPosition()) < 1000) {
            horiCurrent += 15;
            horizontal.setTargetPosition(horiCurrent);
            horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            horizontal.setPower(1);
        } else if (slide < -0.1 && horizontal.getTargetPosition() - 15 > 0 && Math.abs(horizontal.getTargetPosition() - horizontal.getCurrentPosition()) < 1000) {
            horiCurrent -= 15;
            horizontal.setTargetPosition(horiCurrent);
            horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            horizontal.setPower(1);
        }
        /*
        else if((slide<0.1 && slide>-0.1) &&lift1.getTargetPosition()>0 && lift1.getTargetPosition()<105){
            horiCurrent -= 75;
            horizontal.setTargetPosition(horiCurrent);
            horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            horizontal.setPower(1);
        }
        */
        if (intakeUp && !intakeDownToggle) {
            if (Math.abs(intakeFlip1.getPosition() - intakeFlip1_down) < 0.05) {
                intakeFlip1.setPosition(intakeFlip1_up);
                    intakeFlip2.setPosition(intakeFlip1_up);
            } else {
                intakeFlip1.setPosition(intakeFlip1_down);
                intakeFlip2.setPosition(intakeFlip1_down);
            }
            intakeDownToggle = true;
        } else if (!intakeUp) {
            intakeDownToggle = false;
        }
    }

    public void horizontalSys(String pos) {

        if (pos.equals("up")) {
            intakeFlip1.setPosition(intakeFlip1_up);
            intakeFlip2.setPosition(intakeFlip1_up);
        } else if (pos.equals("down")) {
            intakeFlip1.setPosition(intakeFlip1_down);
            intakeFlip2.setPosition(intakeFlip1_down);
        }
    }

    public int liftSimple(boolean up, boolean down){
        if(up){
            vertCurrent += 10;
        } else if (down) {
            vertCurrent -= 10;
        }
        lift1.setTargetPosition(vertCurrent);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift1.setPower(0.8);
        return vertCurrent;
    }

    public void lift(double slide, boolean top, boolean mid) {

        if (slide > 0.1 && lift1.getTargetPosition() + 50 < this.top && Math.abs(lift1.getTargetPosition() - lift1.getCurrentPosition()) < 350) {
            vertCurrent += 50;
            lift1.setTargetPosition(vertCurrent);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(1);
        } else if (slide < -0.1 && lift1.getTargetPosition() - 25 > 0 && Math.abs(lift1.getTargetPosition() - lift1.getCurrentPosition()) < 350) {
            vertCurrent -= 50;
            lift1.setTargetPosition(vertCurrent);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(1);
        } else if ((slide < 0.1 && slide > -0.1) && lift1.getTargetPosition() > 0 && lift1.getTargetPosition() < 105) {
            vertCurrent -= 50;
            lift1.setTargetPosition(vertCurrent);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(1);
        }

        if (top && !liftToggle) {
            if (lift1.getCurrentPosition() > this.mid - 100) {
                lift1.setTargetPosition(this.top);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
            } else {
                lift1.setTargetPosition(this.mid);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
            }
            liftToggle = true;
        } else if (!top) {
            liftToggle = false;
        }
        if (mid) {
            lift1.setTargetPosition(0);
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setPower(1);
        }
    }

    public void arms(boolean armPos, boolean clawButton) {
        //METHOD FOR MANUAL (toggles)
        if (armPos && !armToggle) {
            if (arm1.getPosition() == arm1_out) {
                arm1.setPosition(arm1_in);
                arm2.setPosition(arm2_in);
                arm1_current = arm1_in;
                arm2_current = arm2_in;
            } else {
                arm1.setPosition(arm1_out);
                arm2.setPosition(arm2_out);
                arm1_current = arm1_out;
                arm2_current = arm2_out;
            }
            armToggle = true;
        } else if (!armPos) {
            armToggle = false;
        }
        if (clawButton && !clawToggle) {
            if (claw.getPosition() == claw_closed) {
                claw.setPosition(claw_open);
            } else {
                claw.setPosition(claw_closed);
            }
            clawToggle = true;
        } else if (!clawButton) {
            clawToggle = false;
        }
    }

    public void arms(String armPos, String clawPos) {
        //METHOD FOR SET VALUES
        if (armPos.equals("in")) {
            arm1.setPosition(arm1_in);
            arm2.setPosition(arm2_in);
            arm1_current = arm1_in;
            arm2_current = arm2_in;
        } else if (armPos.equals("out")) {
            arm1.setPosition(arm1_out);
            arm2.setPosition(arm2_out);
            arm1_current = arm1_out;
            arm2_current = arm2_out;
        }
        if (clawPos.equals("open")) {
            claw.setPosition(claw_open);
        } else if (clawPos.equals("closed")) {
            claw.setPosition(claw_closed);
        }

    }
}
