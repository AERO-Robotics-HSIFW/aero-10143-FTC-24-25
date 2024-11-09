package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.statemachines.StateMachines;


public class Onbot_HardwareITD {

    // DEFINE OPMORE MEMBERS
    private LinearOpMode myOpMode;

    // ACCESS INSTRUMENTS OF HUB
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

    public ColorSensor color = null;
    // MOTOR POWERS
    public static double MAX_POWER = 1;
    public static double STRAFE_GAIN = 1.5;
    public static double COUNTS_PER_MOTOR_REV = 537.7; // 28 for REV ;
    public static double DRIVE_GEAR_REDUCTION = 1.0; //   12 for REV;
    public static double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static double ROBOT_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // MOTOR POSITIONS

    public final int top = 3800;
    public final int mid = 2000;

    public final int horiMax = 5000;
    public int horiMin = 0;
    public final int horiInSub = 2000; //some tick number to where the intake would be considered inside of the submersible

    public double intakePower =0;

    // SERVO POSITIONS
    public final double claw_open = 0.66;
    public final double claw_closed = 0.3;
    public final double arms_out = 0.629;
    public final double arms_in = 0.5;
    public final double intakeFlip_down = 0.338;
    public final double intakeFlip_up = 0.433;
    public final double intakeFlip2_offset = 0.095;

    // CURRENT POSITIONS
    public int vertCurrent = 0;
    public int horiCurrent = 0;
    public double intakeFlip_current = intakeFlip_up;
    public double arms_current = arms_in;
    public double claw_current = claw_open;

    public final double kp = 0.03;
    public final double errorMargin = 5;

    //TOGGLES
    private boolean intToggle = false;
    private boolean doubleToggle = false;
    private boolean stringToggle = false;
    private String intakeMode = "forward";




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
        lift1 = myOpMode.hardwareMap.get(DcMotorEx.class, "lift1");
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

        color = myOpMode.hardwareMap.get(ColorSensor.class, "color");
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

        arm1.setDirection(Servo.Direction.REVERSE);
        intakeFlip2.setDirection(Servo.Direction.REVERSE);

        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setTargetPosition(0);
        horizontal.setTargetPosition(0);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // MOTOR POWERS
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // SERVO INITIALIZE *********************************************************************************************************
        arm1.setPosition(arms_in);
        arm2.setPosition(arms_in);
        claw.setPosition(claw_open);
        intakeFlip1.setPosition(intakeFlip_up);
        intakeFlip2.setPosition(intakeFlip_up + intakeFlip2_offset);

        color.enableLed(true);

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

    public int toggleIntegers(boolean input, int value1, int value2, int current){
        int returnValue = current;
        if (input && !intToggle) {
            if (current >= value1) {
                returnValue = value2;
            } else {
                returnValue =value1;
            }
            intToggle = true;
        } else if (!input) {
            intToggle = false;
        }
        return returnValue;
    }
    public double toggleDoubles(boolean input, double value1, double value2, double current){
        double returnValue = current;
        if (input && !doubleToggle) {
            if (value1 == current) {
                returnValue = value2;
            } else {
                returnValue = value1;
            }
            doubleToggle = true;
        } else if (!input) {
            doubleToggle = false;
        }
        return returnValue;
    }

    public String toggleStrings(boolean input, String value1, String value2, String current){
        String returnValue = current;
        if (input && !doubleToggle) {
            if (value1.equals(current)) {
                returnValue = value2;
            } else {
                returnValue = value1;
            }
            stringToggle = true;
        } else if (!input) {
            stringToggle = false;
        }
        return returnValue;
    }

    public void actions(){
        intake.setPower(intakePower);

        horizontal.setTargetPosition(horiCurrent);
        lift1.setTargetPosition(vertCurrent);

        arm1.setPosition(arms_current);
        arm2.setPosition(arms_current);

        intakeFlip1.setPosition(intakeFlip_current);
        intakeFlip1.setPosition(intakeFlip_current + intakeFlip2_offset);

        horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontal.setPower(1);
        lift1.setPower(1);
    }
    //Manual use of intake
    public void intakePowerManual(boolean direction, boolean on) {
        intakeMode = toggleStrings(direction, "forward","reverse",intakeMode);
        intakePower = toggleDoubles(on, intakeMode.equals("forward")?1:-1,0,intakePower);
        intakePosSet(intakePower==0);

    }
    //Set directions for intake
    public double intakePowerSet(boolean on, boolean inRobot) {
        if(on){
            intakePower = inRobot?1:-1; //inRobot signifies whether intake will be bringing samples into the robot
        }
        else{
            intakePower = 0;
        }
        return intakePower;
    }
    public double intakePosSet(boolean up){
        if(up){
            intakeFlip_current = intakeFlip_up;
        }
        else{
            intakeFlip_current = intakeFlip_down;
        }
        return intakeFlip_current;
    }
    public int horiSlidesMan(double slide) {
        if (slide > 0.1 && horizontal.getTargetPosition() + 15 < horiMax && Math.abs(horizontal.getTargetPosition() - horizontal.getCurrentPosition()) < 1000) {
            horiCurrent += (int)(15 * slide);
        } else if (slide < -0.1 && horizontal.getTargetPosition() - 15 > 0 && Math.abs(horizontal.getTargetPosition() - horizontal.getCurrentPosition()) < 1000) {
            horiCurrent += (int)(15 * slide);
        }
        else if((slide<0.1 && slide>-0.1) &&lift1.getCurrentPosition()>0 && lift1.getCurrentPosition()<105) {
            horiCurrent = 0;
        }
        return horiCurrent;
    }

    public int horiSlidesSet(int pos) {
        horiCurrent = pos;
        return horiCurrent;
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

    public int liftMan(double slide, boolean top, boolean reset) {

        if (slide > 0.1 && lift1.getTargetPosition() + 50 < this.top && Math.abs(lift1.getTargetPosition() - lift1.getCurrentPosition()) < 350) {
            vertCurrent += 50;
        } else if (slide < -0.1 && lift1.getTargetPosition() - 25 > 0 && Math.abs(lift1.getTargetPosition() - lift1.getCurrentPosition()) < 350) {
            vertCurrent -= 50;
        } else if ((slide < 0.1 && slide > -0.1) && lift1.getTargetPosition() > 0 && lift1.getTargetPosition() < 105) {
            vertCurrent = 0;
        }
        vertCurrent = toggleIntegers(top, this.mid - 100, this.top, vertCurrent);

        if (reset) {
            vertCurrent = 0;
        }
        return vertCurrent;
    }
    public int vertSlidesSet(int pos) {
        vertCurrent = pos;
        return vertCurrent;
    }

    //MANUAL
    public void armsMan(boolean armPos, boolean clawButton) {
        arms_current = toggleDoubles(armPos, arms_in, arms_out, arms_current);
        claw_current = toggleDoubles(clawButton, claw_open, claw_closed, claw_current);
    }
    //SET POSITIONS
    public double armsPos(String armPos) {
        //METHOD FOR SET VALUES
        if (armPos.equals("in")) {
            arms_current = arms_in;
        } else if (armPos.equals("out")) {
            arms_current = arms_out;
        }
        return arms_current;
    }
    public double clawPos(String clawPos){
        if (clawPos.equals("open")) {
            claw_current = claw_open;
        } else if (clawPos.equals("closed")) {
            claw_current = claw_closed;
        }
        return claw_current;
    }
    public char largestColor(){
        char returnValue = ' ';
        if(color.green()> color.red() && (color.green()>color.blue())){
            returnValue= 'y';
        }
        else if(color.red()>color.green() && (color.red() > color.blue())){
            returnValue = 'r';
        }
        else{
            returnValue= 'b';
        }
        return returnValue;
    }
    public boolean colorThreshold(){
        return color.red() > 200 || color.green() > 200 || color.blue() > 200;
    }
    public double intake(char largestColor, String teamColor){
        double returnValue;
        if(colorThreshold()){
            if(teamColor.equals("red")){
                if(largestColor == 'r' || largestColor == 'y'){
                    returnValue = intakePowerSet(false,true);
                }
                else{
                    returnValue = intakePowerSet(true, false);
                }
            }
            else{
                if(largestColor == 'b' || largestColor == 'y'){
                    returnValue = intakePowerSet(false,true);
                }
                else{
                    returnValue = intakePowerSet(true, false);
                }
            }
        }
        else{
            returnValue = 1;
        }
        return returnValue;
    }
}
