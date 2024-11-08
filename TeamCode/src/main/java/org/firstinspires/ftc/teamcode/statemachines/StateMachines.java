package org.firstinspires.ftc.teamcode.statemachines;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Onbot_HardwareITD;

public class StateMachines {
    public enum robotStates {
        IDLE,
        LIFT_START,
        LIFT_EXTEND,
        LIFT_GRAB,
        LIFT_DUMP,
        LIFT_RETRACT,
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_GRAB,
        INTAKE_RETRACT,
        INTAKE_TRANSFER,
        ARMS_OUT,
        ARMS_IN,
        MANUAL
    }

    /* Note to self: The term "reversing" means it is going into the robot
    the term "forward" means that it is going out of the robot
    (That is for intake when looking in the hardware class)
    */
    Onbot_HardwareITD hardware = null;
    robotStates robotState = robotStates.INTAKE_EXTEND;
    ElapsedTime runtime;
    public int vertTarget;

    public double armsTarget;

    public double intakeFlipTarget;

    public int intakePower;

    public int horiTarget;
    private robotStates prevState = robotState;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    // Constructor initializing
    public StateMachines() {
        hardware = new Onbot_HardwareITD();
        runtime = new ElapsedTime();
        armsTarget = hardware.arms_in;
        horiTarget = 0;
        vertTarget =0;
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
        boolean manual = (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.right_bumper);

        if (gamepad2.left_bumper) {
            robotState = robotStates.INTAKE_EXTEND;
        } else if(gamepad2.b) { // B is supposed to reverse the intake, but how would that be written in a state machine?
            // just set the intake power to be something
            robotState = robotStates.INTAKE_EXTEND;
        } else if (gamepad1.dpad_up) {
            robotState = robotStates.LIFT_EXTEND;
        } else if (gamepad1.dpad_down) {
            robotState = robotStates.LIFT_RETRACT;
        } else {
            robotState = robotStates.IDLE;
        }

        if(manual){
            prevState = (robotState == robotStates.MANUAL?prevState:robotState); //if it runs again prevState will never be set to manual
            robotState =  robotState.MANUAL;
        }

    }

    // The actual state machine logic
    public void stateMachineLogic() {
        switch (robotState) {
            case INTAKE_START: // see input translation -> gp2 left bump signifies starting state machine
                break;
            case INTAKE_EXTEND:
                hardware.intakePosSet(horiTarget < hardware.horiInSub);
                break;
            case INTAKE_GRAB:

                break;
                 //I think this implementation is wrong. You should not be doing manual control here.
                /* here is something better:
                //if robot picks up something colored then start retracting
                if(colorSensor.blue() > some number){
                    hardware.horizontal.setTargetPosition(0);
                    intake needs to be pivoted up to avoid crashing into barrier
                    runtime.reset();
                    robotState = robotStates.INTAKE_RETRACT;
                }
                 */
            case INTAKE_RETRACT:
                //in your retract state you need something to tell the intake pivot servos to lift immediately
                //hardware.intakeState("forward",true); *********
                if(runtime.milliseconds() > 1500){
                    robotState = robotStates.INTAKE_TRANSFER;
                    runtime.reset();
                }
                break;

            case INTAKE_TRANSFER:
                hardware.arms(false,true);
                if(runtime.milliseconds() > 500){
                    robotState = robotStates.LIFT_GRAB;
                }
                break;

            case LIFT_GRAB:
                // Ralph said he'd do this one
                runtime.reset();
                robotState = robotStates.LIFT_EXTEND;
                break;

            case LIFT_EXTEND:
                hardware.arms("out","ur mom"/*this changes absolutely nothing - ralph (this is targeted at you kevin, in case u ever see this :)*/);
                if(runtime.milliseconds() > 500) {
                    robotState = robotStates.ARMS_OUT;
                }
                break;

            case ARMS_OUT:
                // Sit on it
                robotState = robotStates.LIFT_DUMP;
                break;

            case LIFT_DUMP:
               hardware.arms("in","our mom" /* Again, does nothing, dw Kevin - Ritvik */);
               if(runtime.milliseconds() > 500) {
                   robotState = robotStates.ARMS_IN;
               }
               break;

           case ARMS_IN:

               robotState = robotStates.LIFT_RETRACT;
               break;

            case LIFT_RETRACT:
                //this should not be manaual
                horiTarget = 0;
                break;
            case MANUAL:
                break;
            default:
                //default state needs to be initialization position
                robotState = robotStates.INTAKE_START;
                break;

        }
    }

}