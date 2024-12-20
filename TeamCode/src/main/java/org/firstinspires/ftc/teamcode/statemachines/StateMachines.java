package org.firstinspires.ftc.teamcode.statemachines;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Onbot_HardwareITD;

public class StateMachines {
    public enum robotStates {
        IDLE,
        LIFT_EXTEND,
        LIFT_GRAB,
        LIFT_DUMP,
        LIFT_RETRACT,
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_CHECK,
        INTAKE_IN,
        INTAKE_OUT,
        INTAKE_GRAB,
        INTAKE_RETRACT,
        INTAKE_TRANSFER,
        ARMS_OUT,
        MANUAL
    }

    /* Note to self: The term "reversing" means it is going into the robot
    the term "forward" means that it is going out of the robot
    (That is for intake when looking in the hardware class)
    */
    Onbot_HardwareITD hardware = null;
    public robotStates robotState = robotStates.INTAKE_START;
    ElapsedTime runtime;
    public int vertTarget;

    public double armsTarget;
    public double clawTarget;

    public double intakeFlipTarget;

    public double intakePower;

    public int horiTarget;
    private robotStates prevState = robotState;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private boolean top = false;
    public String teamColor = "red";
    public char largestColor = ' ';
    // Constructor initializing
    public StateMachines(Onbot_HardwareITD robot) {
        hardware = robot;
        runtime = new ElapsedTime();
        armsTarget = hardware.arms_in;
        horiTarget = 0;
        vertTarget = 0;
        clawTarget = hardware.clawState("open");
        intakeFlipTarget = hardware.intakeFlip_up;
    }

    // All possible states the robot can be in


    public void stateAction() {
        // Applying when to set the target position for encoder and servos
        hardware.lift1.setTargetPosition(vertTarget);
        //this is correct
        hardware.horizontal.setTargetPosition(horiTarget);

        hardware.arm1.setPosition(armsTarget);
        hardware.arm2.setPosition(armsTarget);

        hardware.claw.setPosition(clawTarget);

        hardware.intakeFlip1.setPosition(intakeFlipTarget);
        hardware.intakeFlip2.setPosition(intakeFlipTarget + hardware.intakeFlip2_offset);

        // Assigning power
        hardware.intake.setPower(intakePower);

        hardware.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.lift1.setPower(1);
        hardware.horizontal.setPower(1);
    }

    // Taking the inputs from the teleop file, and translating the binds to their perspective states
    public void inputTranslation(Gamepad input1, Gamepad input2) {
        gamepad1 = input1;
        gamepad2 = input2;
        boolean manual = (gamepad2.left_stick_y >0.1 || gamepad2.left_stick_y<-0.1 || gamepad2.dpad_right || gamepad2.dpad_left);

        if (gamepad2.x) {
            robotState = robotStates.INTAKE_START;
        }else if(gamepad2.left_bumper){
            robotState = robotStates.INTAKE_IN;
        }else if(gamepad2.right_bumper){
            robotState = robotStates.INTAKE_RETRACT;
        }
        else if (gamepad1.dpad_up && robotState.equals(robotStates.LIFT_GRAB)) {
            vertTarget = 3800;
            top = true;
            robotState = robotStates.LIFT_EXTEND;

        } else if (gamepad1.dpad_down && robotState.equals(robotStates.LIFT_GRAB)) {
            vertTarget = 2000;
            top = false;
            robotState = robotStates.LIFT_EXTEND;
        } else if (gamepad1.a) {
            //robotState = robotStates.LIFT_RETRACT;
        } else if (gamepad1.left_bumper) {
            robotState = robotStates.LIFT_DUMP;
        } else if(gamepad2.right_trigger > 0.9){
            runtime.reset();
            robotState = robotStates.IDLE;
        }

        if(manual){
            prevState = robotState; //if it runs again prevState will never be set to manual
            robotState =  robotStates.MANUAL;
        }

    }

    // The actual state machine logic
    public void stateMachineLogic() {
        switch (robotState) {
            case INTAKE_START: // see input translation -> gp2 left bump signifies starting state machine
                intakePower = hardware.intakePowerSet(false,false);
                clawTarget = hardware.clawState("open");
                break;
            case INTAKE_EXTEND:
                intakeFlipTarget = hardware.intakePosSet(horiTarget < hardware.horiInSub);
                if(hardware.colorThreshold()){
                    runtime.reset();
                    intakeFlipTarget = hardware.intakePosSet(true);
                    intakePower = hardware.intakePowerSet(false, false);
                    robotState = robotStates.INTAKE_CHECK;
                }
                break;
            case INTAKE_CHECK:
                if(runtime.milliseconds() > 250){
                    largestColor = hardware.largestColor();
                    intakePower = hardware.intake(largestColor, teamColor);
                    if(intakePower == 1){
                        robotState = robotStates.INTAKE_IN;
                    }
                    else if(intakePower == -1){
                        runtime.reset();
                        robotState = robotStates.INTAKE_OUT;
                    }
                    else{
                        horiTarget = hardware.horiSlidesSet(0);
                        intakeFlipTarget = hardware.intakePosSet(true);
                        robotState = robotStates.INTAKE_GRAB;
                    }
                }
                break;
            case INTAKE_IN:
                intakePower = hardware.intakePowerSet(true,true);
                robotState = robotStates.INTAKE_EXTEND;
                break;
            case INTAKE_OUT:
                intakeFlipTarget = hardware.intakePosSet(true);
                if(runtime.milliseconds() > 500){
                    intakePower = hardware.intakePowerSet(true,true);
                    robotState = robotStates.INTAKE_EXTEND;
                }
                break;
            case INTAKE_GRAB:
                intakePower = hardware.intakePowerSet(false,false);
                if(hardware.horizontal.getCurrentPosition() < 100){
                    robotState = robotStates.INTAKE_RETRACT;
                }
                break;

            case INTAKE_RETRACT:
                intakePower = hardware.intakePowerSet(true,true);
                if(runtime.milliseconds() > 1500){
                    runtime.reset();
                    robotState = robotStates.INTAKE_TRANSFER;
                }
                break;

            case INTAKE_TRANSFER:
                if(runtime.milliseconds() > 500){
                    intakePower = hardware.intakePowerSet(false,false);
                    armsTarget = hardware.armsPos("in");
                    clawTarget = hardware.clawState("closed");
                    runtime.reset();
                    robotState = robotStates.LIFT_GRAB;
                }
                break;

            case LIFT_GRAB:
                //dpad_up top, dpad_down mid

                break;

            case LIFT_EXTEND:
                if((hardware.lift1.getCurrentPosition() > 1500 && !top) || (hardware.lift1.getCurrentPosition() > 3300 && top)) {
                    armsTarget = hardware.armsPos("out");
                    if(gamepad1.right_bumper){
                        runtime.reset();
                        robotState = robotStates.ARMS_OUT;
                    }
                }
                break;

            case ARMS_OUT:
                if (runtime.milliseconds() > 1000) {
                    clawTarget = hardware.clawState("open");
                    runtime.reset();
                    robotState = robotStates.LIFT_DUMP;
                }
                break;

            case LIFT_DUMP:
                if (runtime.milliseconds() > 1000) {
                    armsTarget = hardware.armsPos("in");
                    runtime.reset();
                    robotState = robotStates.LIFT_RETRACT;

                }
                break;
            case LIFT_RETRACT:
                if(runtime.milliseconds() > 500){
                    vertTarget = hardware.vertSlidesSet(0);
                    robotState = robotStates.INTAKE_START;
                }
                break;
            case MANUAL:
                horiTarget = hardware.horiSlidesMan(-gamepad2.left_stick_y);
                if(gamepad2.dpad_right){
                    intakeFlipTarget = hardware.intakePosSet(true);
                    intakePower = hardware.intakePowerSet(true, false);
                }
                else if(gamepad2.dpad_left){
                    intakePower = hardware.intakePowerSet(true,true);
                }
                robotState = prevState;
                break;
            case IDLE:
                clawTarget = hardware.clawState("open");
                armsTarget = hardware.armsPos("in");
                intakeFlipTarget = hardware.intakePosSet(true);
                intakePower = hardware.intakePowerSet(true, false);
                if(runtime.milliseconds() > 2000){
                    vertTarget = 0;
                    horiTarget = 0;
                    robotState = robotStates.INTAKE_START;
                }
                break;
            default:
                //default state needs to be initialization position
                robotState = robotStates.INTAKE_START;
                break;

        }
    }
}