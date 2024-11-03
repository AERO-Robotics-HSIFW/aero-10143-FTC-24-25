package org.firstinspires.ftc.teamcode.SAMPLE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    OpMode myOpMode;
    // Motor constants for target positions
    public static final int LIFT_POSITION_STATE_ONE = 500;
    public static final int LIFT_POSITION_STATE_TWO = 1000;
    public static final int LIFT_POSITION_STATE_THREE = 1500;
    public static final int LIFT_POSITION_STATE_FOUR = 2000;

    public static final int SLIDE_POSITION_STATE_ONE = 300;
    public static final int SLIDE_POSITION_STATE_TWO = 600;
    public static final int SLIDE_POSITION_STATE_THREE = 900;
    public static final int SLIDE_POSITION_STATE_FOUR = 1200;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor liftMotor, horizontalSlideMotor;

    public Hardware(OpMode opMode) {
        // Initialize drive motors
        myOpMode = opMode;

        frontLeft = myOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = myOpMode.hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize lift and slide motors
        liftMotor = myOpMode.hardwareMap.get(DcMotor.class, "liftMotor");
        horizontalSlideMotor = myOpMode.hardwareMap.get(DcMotor.class, "horizontalSlideMotor");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setLiftTargetPosition(int position) {
        liftMotor.setTargetPosition(position);
    }

    public void setHorizontalSlideTargetPosition(int position) {
        horizontalSlideMotor.setTargetPosition(position);
    }

    public void applyTargetPositions() {
        liftMotor.setPower(1.0);
        horizontalSlideMotor.setPower(1.0);
    }

    public void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        liftMotor.setPower(0);
        horizontalSlideMotor.setPower(0);
    }

    public void mecanumDrive(double forward, double strafe, double rotate) {
        double flPower = forward + strafe + rotate;
        double frPower = forward - strafe - rotate;
        double blPower = forward - strafe + rotate;
        double brPower = forward + strafe - rotate;

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }
}
