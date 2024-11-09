package org.firstinspires.ftc.teamcode.SAMPLE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Robot TeleOp", group="TeleOp")
public class TeleOp extends OpMode {

    private Hardware robot;
    private StateMachine stateMachine;

    @Override
    public void init() {
        robot = new Hardware(this);
        stateMachine = new StateMachine(robot);
    }

    @Override
    public void loop() {
        stateMachine.inputTranslation(gamepad1, gamepad2);
        stateMachine.runStateMachine();
        stateMachine.stateAction();
    }
}
