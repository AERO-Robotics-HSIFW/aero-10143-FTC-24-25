package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class HardwareITD {

    // DEFINE OPMORE MEMBERS
    private LinearOpMode myOpMode;

    // ACCESS INSTRUMENTS OF HUB
    public IMU imu;
    Orientation angle;

    // SENSORS

    // MOTOR DECLARATIONS - MOVEMENT
    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;

    // MOTOR DECLARATIONS - SUBSYSTEMS
    public DcMotorEx lift1 = null;
    public DcMotorEx lift2 = null;
    public DcMotorEx horizontal = null;
    public DcMotor intake = null;

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
    public static double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    public static double ROBOT_DIAMETER_INCHES = 20.0;
    public static double ROBOT_CIRCUMFERENCE = ROBOT_DIAMETER_INCHES * Math.PI;
    public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            WHEEL_CIRCUMFERENCE;
    public final double inRobot = 1; //testing required for this

    // MOTOR POSITIONS

    public final int top = 3000;
    public final int mid = 1750;

    public final int horiMax = 1500;
    public final int horiInSub = 500; //some tick number to where the intake would be considered inside of the submersible

    public double intakePower = 0;

    // SERVO POSITIONS
    public final double claw_open = 0.51; // Value might need changing
    public final double claw_closed = 0.415; // Value might need changing
    public final double arms_out_SAMP = 0.8; // Value might need changing
    public final double arms_in = 0.16; // Value might need changing
    public final double arms_out_SPEC = 1;
    public final double intakeFlip_down = 0.53; // Value will 100% need changing
    public final double intakeFlip_up = 0.43; // Value will 100% need changing
    public final double intakeFlip2_offset = 0; // Value will 100% need changing

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
        frontLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = myOpMode.hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = myOpMode.hardwareMap.get(DcMotorEx.class, "backRight");
        lift1 = myOpMode.hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = myOpMode.hardwareMap.get(DcMotorEx.class,"lift2");
        horizontal = myOpMode.hardwareMap.get(DcMotorEx.class, "horizontal");
        intake = myOpMode.hardwareMap.get(DcMotor.class, "intake");


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
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        lift1.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontal.setDirection(DcMotor.Direction.FORWARD);

        arm2.setDirection(Servo.Direction.REVERSE);
        intakeFlip2.setDirection(Servo.Direction.REVERSE);

        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setTargetPosition(0);
        lift2.setTargetPosition(0);
        horizontal.setTargetPosition(0);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        intakeFlip2.setPosition(intakeFlip_up);


        color.enableLed(true);

    }

    public void autoMovement(int forw, double side, int spin, double power) {
        int forwardCount = (int) (forw*COUNTS_PER_INCH);
        int sideCount = (int) (side*COUNTS_PER_INCH);
        spin = (int) ((spin%360)/180.0*ROBOT_CIRCUMFERENCE * COUNTS_PER_INCH*1.375);
        int turnCount = spin;


        int frontLeftTarget = frontLeft.getCurrentPosition() + (forwardCount + sideCount + turnCount);
        int frontRightTarget = frontRight.getCurrentPosition()+ (forwardCount - sideCount - turnCount);
        int backLeftTarget = backLeft.getCurrentPosition()+ (forwardCount - sideCount + turnCount);
        int backRightTarget = backRight.getCurrentPosition()+ (forwardCount + sideCount - turnCount);

       frontRight.setTargetPosition(frontRightTarget);
       frontLeft.setTargetPosition(frontLeftTarget);
       backRight.setTargetPosition(backRightTarget);
       backLeft.setTargetPosition(backLeftTarget);


        encoderState("position");
        double isTurn = spin!=0?0.5:1;
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power*isTurn);
        backLeft.setPower(power*isTurn);

        while(frontRight.isBusy() && frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy());
        encoderState("run");

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

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
        backLeft.setPower(BLPow * MAX_POWER);
        backRight.setPower(BRPow * MAX_POWER);

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
                returnValue = value1;
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
        horizontal.setTargetPosition(horiCurrent);
        lift1.setTargetPosition(vertCurrent);

        arm1.setPosition(arms_current);
        arm2.setPosition(arms_current);

        intakeFlip1.setPosition(intakeFlip_current);
        intakeFlip2.setPosition(intakeFlip_current + intakeFlip2_offset);
        claw.setPosition(claw_current);
        horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontal.setPower(1);
        lift1.setPower(1);
        lift2.setPower(2);
        intake.setPower(intakePower);
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
            intakePower = inRobot?this.inRobot:-this.inRobot; //inRobot signifies whether intake will be bringing samples into the robot
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
            horiCurrent += (int)(50 * slide);
        } else if (slide < -0.1 && horizontal.getTargetPosition() - 15 > 0 && Math.abs(horizontal.getTargetPosition() - horizontal.getCurrentPosition()) < 1000) {
            horiCurrent += (int)(50 * slide);
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
        arms_current = toggleDoubles(armPos, arms_in, arms_out_SAMP, arms_current);
        claw_current = toggleDoubles(clawButton, claw_open, claw_closed, claw_current);
    }
    public double armsIncrement(double newPos){
        arms_current = newPos;
        return arms_current;
    }
    //SET POSITIONS
    public double armsPos(String armPos) {
        //METHOD FOR SET VALUES
        if (armPos.equals("in")) {
            arms_current = arms_in;
        } else if (armPos.equals("out_samp")) {
            arms_current = arms_out_SAMP;
        }
        else if(armPos.equals("out_spec")){
            arms_current = arms_out_SPEC;
        }
        return arms_current;
    }
    public double armsPos(double newPos){
        arms_current = newPos;
        return arms_current;
    }
    public double clawState(boolean open){
        if (open) {
            claw_current = claw_open;
        } else{
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
        int threshold = 600;
        return color.red() > threshold || color.green() > threshold || color.blue() > threshold;
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
