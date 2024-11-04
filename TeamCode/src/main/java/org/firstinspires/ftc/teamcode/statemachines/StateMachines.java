package org.firstinspires.ftc.teamcode.statemachines;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.*;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.Onbot_HardwareITD;

public class StateMachines {
    /* Note to self: The term "reversing" means it is going into the robot
    the term "forward" means that it is going out of the robot
    (That is for intake when looking in the hardware class)
    */
    Onbot_HardwareITD hardware = null;
    ElapsedTime runtime;
    public int curLiftTargetPos;

    // Constructor initializing
    public StateMachines(){
        hardware = new Onbot_HardwareITD();
        runtime = new ElapsedTime();
        curLiftTargetPos = 0;
    }

    // All possible states the robot can be in
    public enum robotStates {
        IDLE,
        LIFT_START,
        LIFT_EXTEND,
        LIFT_GRAB,
        LIFT_DUMP,
        LIFT_RETRACT,
        INTAKE_EXTEND,
        INTAKE_RETRACT,
        INTAKE_TRANSFER,
        ARMS_OUT,
        ARMS_IN,
        MANUAL
    }
    robotStates robotState = robotStates.INTAKE_EXTEND;

    public void stateAction() {
        // Need assistance
    }

    // Taking the inputs from the teleop file, and translating the binds to their perspective states
    public void inputTranslation(Gamepad input1, Gamepad input2) {
        gamepad1 = input1;
        gamepad2 = input2;

        if (gamepad2.a) {
            robotState = robotStates.INTAKE_EXTEND;
        } else if(gamepad2.b) { // B is supposed to reverse the intake, but how would that be written in a state machine?
            // Ask about
        } else if (gamepad1.dpad_up) {
            robotState = robotStates.LIFT_EXTEND;
        } else if (gamepad1.dpad_down) {
            robotState = robotStates.LIFT_RETRACT;
        } else {
            robotState = robotStates.IDLE;
        }
    }

    // The actual state machine logic
    public void stateMachineLogic() {
        switch (robotState) {
            case INTAKE_EXTEND:
                hardware.intakeState(gamepad2.b,gamepad2.a);
                hardware.horizontalSys(gamepad2.dpad_up?0.5:(gamepad2.dpad_down?-0.5:0), gamepad2.b);

                if(gamepad2.a) {
                    hardware.horizontalSys(-0.5, false);
                    hardware.horizontalSys("up");
                    if (hardware.horizontal.getCurrentPosition() < 150) {
                        hardware.horizontal.setTargetPosition(0);
                        runtime.reset();
                        robotState = robotStates.INTAKE_RETRACT;
                    }
                }
                break;

            case INTAKE_RETRACT:
                hardware.intakeState("forward",true);
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

            case LIFT_EXTEND:
                hardware.arms("out","ur mom"/*this changes absolutely nothing - ralph (this is targeted at you kevin, in case u ever see this :)*/);
                if(runtime.milliseconds() > 500) {
                    robotState = robotStates.ARMS_OUT;
                }

            case ARMS_OUT:
                // Sit on it

            case LIFT_DUMP:
               hardware.arms("in","our mom" /* Again, does nothing, dw Kevin - Ritvik */);
               if(runtime.milliseconds() > 500) {
                   robotState = robotStates.ARMS_IN;
               }

           case ARMS_IN:


            case LIFT_RETRACT:
                if (gamepad1.dpad_down) {
                    hardware.lift(gamepad1.dpad_up?0.5:(gamepad1.dpad_down?-0.5:0),false,false);
                    robotState = robotStates.LIFT_GRAB;
                }

            default:
                robotState = robotStates.INTAKE_EXTEND;
        }
    }
}