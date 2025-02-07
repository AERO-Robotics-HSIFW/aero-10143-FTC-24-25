package org.firstinspires.ftc.teamcode.statemachines;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.HardwareITD;

public class StateMachines {
    public enum robotStates {
        IDLE,
        SPECIMEN_INIT,
        SPECIMEN_GRAB,
        SPECIMEN_RELEASE,
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
    HardwareITD hardware = null;
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
    public StateMachines(HardwareITD robot) {
        hardware = robot;
        runtime = new ElapsedTime();
        armsTarget = hardware.arms_in;
        horiTarget = 0;
        vertTarget = 0;
        intakeFlipTarget = hardware.intakeFlip_up;
    }

    // All possible states the robot can be in


    public void stateAction() {
        // Applying when to set the target position for encoder and servos
        hardware.lift1.setTargetPosition(vertTarget);
        hardware.lift2.setTargetPosition(vertTarget);
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
        hardware.lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.lift1.setPower(1);
        hardware.lift2.setPower(1);
        hardware.horizontal.setPower(0.75);
    }

    // Taking the inputs from the teleop file, and translating the binds to their perspective states
    public void inputTranslation(Gamepad input1, Gamepad input2) {
        gamepad1 = input1;
        gamepad2 = input2;
        boolean armsReady = (robotState.equals(robotStates.ARMS_OUT) || robotState.equals(robotStates.INTAKE_START) || robotState.equals(robotStates.LIFT_GRAB));
        //GAMEPAD1 INPUTS
            //StateTranslations

        if (gamepad1.a && robotState.equals(robotStates.LIFT_GRAB)) {
            vertTarget = hardware.top;
            top = true;
            robotState = robotStates.LIFT_EXTEND;
        }
        if(gamepad1.right_bumper && robotState.equals(robotStates.ARMS_OUT)) {
            clawTarget = hardware.clawState(true);
            runtime.reset();
            robotState = robotStates.LIFT_DUMP;
        }
        if(gamepad1.right_trigger > 0.5&& armsReady){
            armsTarget = hardware.armsPos(armsTarget+0.001);
        }
        else if(gamepad1.left_trigger > 0.5 && armsReady){
            armsTarget = hardware.armsPos(armsTarget-0.001);
        }
        if(armsReady){
            clawTarget = hardware.clawState(hardware.toggleBooleans(gamepad1.right_bumper, true, false, clawTarget == hardware.claw_open));
        }
        if(gamepad1.dpad_up&&armsReady){
            vertTarget = hardware.liftMan(0.5, false,false);
        }
        else if(gamepad1.dpad_down&&armsReady){
            vertTarget = hardware.liftMan(-0.5,false,false);
        }


        //GAMEPAD2 INPUTS
            //StateTranslations
        if (gamepad2.dpad_down) {
            robotState = robotStates.INTAKE_START;
        }
        if(gamepad2.dpad_right){ //dpad_right OUT robot
            intakeFlipTarget = hardware.intakePosSet(true);
            intakePower = hardware.intakePowerSet(true, false);
        }
        if(gamepad2.dpad_left){ //dpad_left INTO robot
            intakeFlipTarget = hardware.intakePosSet(false);
            intakePower = hardware.intakePowerSet(true,true);
        }
        else if(gamepad2.dpad_down){//dpad_down intake OFF + UP
            intakeFlipTarget = hardware.intakePosSet(true);
            intakePower = hardware.intakePowerSet(false,false);
        }
        horiTarget = hardware.horiSlidesMan(-gamepad2.left_stick_y);

    }

    // The actual state machine logic
    public void stateMachineLogic() {
        switch (robotState) {
            case INTAKE_START: // see input translation -> gp2 left bump signifies starting state machine
                intakePower = hardware.intakePowerSet(false,false);
                clawTarget = hardware.claw_open;
                break;
            case INTAKE_EXTEND:
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
                hardware.intakePower = 0;
                clawTarget = hardware.clawState(true);
                intakePower = hardware.intakePower;
                robotState = robotStates.INTAKE_EXTEND;
                break;
            case INTAKE_OUT:
                intakeFlipTarget = hardware.intakePosSet(true);
                if(runtime.milliseconds() > 1500){
                    intakePower = hardware.intakePowerSet(true,true);
                    robotState = robotStates.INTAKE_EXTEND;
                }
                break;
            case INTAKE_GRAB:
                intakePower = hardware.intakePowerSet(false,false);
                if(hardware.horizontal.getCurrentPosition() < 100){

                    intakeFlipTarget = 0.4756;
                    robotState = robotStates.INTAKE_RETRACT;
                }
                break;

            case INTAKE_RETRACT:
                intakePower = hardware.intakePowerSet(true,true);
                if(runtime.milliseconds() > 250){
                    runtime.reset();
                    robotState = robotStates.INTAKE_TRANSFER;
                }
                break;

            case INTAKE_TRANSFER:
                if(runtime.milliseconds() > 500){
                    clawTarget = hardware.clawState(false);
                    intakePower = hardware.intakePowerSet(false,false);
                    armsTarget = hardware.armsPos("in"); //just in case um
                    runtime.reset();
                    robotState = robotStates.LIFT_GRAB;
                }
                break;

            case SPECIMEN_INIT:
                vertTarget = hardware.vertSlidesSet(900);
                if(runtime.milliseconds() > 500){
                    armsTarget = hardware.armsPos(0.6725);
                    if(runtime.milliseconds() > 1500) {
                        vertTarget = hardware.vertSlidesSet(0);
                    }
                }
                break;
            case SPECIMEN_GRAB:
                clawTarget = hardware.clawState(false);
                if(runtime.milliseconds() > 200){
                    horiTarget = hardware.horiSlidesSet(200);
                    if(runtime.milliseconds()>500){
                        armsTarget = hardware.armsPos(0.6);
                        robotState = robotStates.ARMS_OUT;
                    }
                }
                break;
            case LIFT_GRAB:
                //dpad_up top, dpad_down mid

                break;

            case LIFT_EXTEND:
                if((hardware.lift1.getCurrentPosition() > hardware.mid-200 && !top) || (hardware.lift1.getCurrentPosition() > hardware.top-200 && top)) {
                    armsTarget = hardware.armsPos("out_samp");
                    runtime.reset();
                    robotState=robotStates.ARMS_OUT;
                }
                break;

            case ARMS_OUT:
                //look at input translation
                break;

            case SPECIMEN_RELEASE:
                armsTarget = hardware.armsPos(0.6725);
                if(runtime.milliseconds() > 500){
                    clawTarget = hardware.clawState(true);
                    vertTarget = hardware.vertSlidesSet(1200);
                    armsTarget = hardware.armsPos("in");
                    runtime.reset();
                    robotState = robotStates.LIFT_RETRACT;

                }
                break;
            case LIFT_DUMP:
                if (runtime.milliseconds() > 500) {
                    armsTarget = hardware.armsPos("in");
                    runtime.reset();
                    robotState = robotStates.LIFT_RETRACT;

                }
                break;
            case LIFT_RETRACT:

                if(runtime.milliseconds() > 750){
                    clawTarget = hardware.clawState(false);
                    vertTarget = hardware.vertSlidesSet(0);
                    robotState = robotStates.INTAKE_START;
                }
                break;
            case MANUAL:


                /*GAMEPAD 1 MANUAL
                boolean armsReady = (prevState.equals(robotStates.ARMS_OUT) || prevState.equals(robotStates.INTAKE_START) || prevState.equals(robotStates.LIFT_GRAB));

                if(gamepad1.right_trigger > 0.5&& armsReady){
                    armsTarget = hardware.armsPos(armsTarget+0.001);
                }
                else if(gamepad1.left_trigger > 0.5 && armsReady){
                    armsTarget = hardware.armsPos(armsTarget-0.001);
                }
                if(armsReady && gamepad1.b){
                    clawTarget = hardware.clawState(false);
                }
                else if(armsReady && gamepad1.a){
                    clawTarget = hardware.clawState(true);
                }

                if(gamepad1.dpad_left&&armsReady){
                    vertTarget = hardware.liftMan(0.5, false,false);
                }
                else if(gamepad1.dpad_right&&armsReady){
                    vertTarget = hardware.liftMan(-0.5,false,false);
                }

                //GAMEPAD2 INPUTS
                if(gamepad2.dpad_right){
                    intakeFlipTarget = hardware.intakePosSet(true);
                    intakePower = hardware.intakePowerSet(true, false);
                }
                else if(gamepad2.dpad_left){
                    intakeFlipTarget = hardware.intakePosSet(false);
                    intakePower = hardware.intakePowerSet(true,true)/2;
                }
                else if(gamepad2.dpad_up){
                    intakeFlipTarget = hardware.intakePosSet(true);
                    intakePower = hardware.intakePowerSet(false,false);
                }
                horiTarget = hardware.horiSlidesMan(-gamepad2.left_stick_y);
                // program manual slide control + claaw
                 */
                robotState = prevState;
                break;

            case IDLE:
                clawTarget = hardware.clawState(true);
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
