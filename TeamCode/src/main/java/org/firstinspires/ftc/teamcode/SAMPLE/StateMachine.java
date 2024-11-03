package org.firstinspires.ftc.teamcode.SAMPLE;

import com.qualcomm.robotcore.hardware.Gamepad;

public class StateMachine {

    private Hardware hardware;
    private RobotState currentState;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    // Variables to store the current target positions
    private int currentLiftTargetPosition;
    private int currentSlideTargetPosition;

    // Increment values for manual control
    private static final int LIFT_INCREMENT = 50;
    private static final int SLIDE_INCREMENT = 50;

    public enum RobotState {
        IDLE,
        STATE_ONE,
        STATE_TWO,
        STATE_THREE,
        STATE_FOUR,
        MANUAL // New manual state
    }

    public StateMachine(Hardware hardware) {
        this.hardware = hardware;
        this.currentState = RobotState.IDLE; // initial state
        this.currentLiftTargetPosition = 0;
        this.currentSlideTargetPosition = 0;
    }

    public void inputTranslation(Gamepad input1, Gamepad input2) {
        gamepad1 = input1;
        gamepad2 = input2;
        // Check gamepad inputs and set the current state
        if (gamepad1.a) {
            currentState = RobotState.STATE_ONE;
        } else if (gamepad1.b) {
            currentState = RobotState.STATE_TWO;
        } else if (gamepad1.x) {
            currentState = RobotState.STATE_THREE;
        } else if (gamepad1.y) {
            currentState = RobotState.STATE_FOUR;
        } else if (gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_down) {
            currentState = RobotState.MANUAL;
        } else {
            currentState = RobotState.IDLE;
        }
    }

    public void runStateMachine() {
        // Update target positions based on the current state
        switch (currentState) {
            case STATE_ONE:
                // Set target positions for state one
                currentLiftTargetPosition = Hardware.LIFT_POSITION_STATE_ONE;
                currentSlideTargetPosition = Hardware.SLIDE_POSITION_STATE_ONE;
                break;

            case STATE_TWO:
                // Set target positions for state two
                currentLiftTargetPosition = Hardware.LIFT_POSITION_STATE_TWO;
                currentSlideTargetPosition = Hardware.SLIDE_POSITION_STATE_TWO;
                break;

            case STATE_THREE:
                // Set target positions for state three
                currentLiftTargetPosition = Hardware.LIFT_POSITION_STATE_THREE;
                currentSlideTargetPosition = Hardware.SLIDE_POSITION_STATE_THREE;
                break;

            case STATE_FOUR:
                // Set target positions for state four
                currentLiftTargetPosition = Hardware.LIFT_POSITION_STATE_FOUR;
                currentSlideTargetPosition = Hardware.SLIDE_POSITION_STATE_FOUR;
                break;

            case MANUAL:
                // Manual adjustments based on D-Pad input
                if (gamepad1.dpad_up) {
                    currentLiftTargetPosition += LIFT_INCREMENT;
                } else if (gamepad1.dpad_down) {
                    currentLiftTargetPosition -= LIFT_INCREMENT;
                }

                if (gamepad1.dpad_right) {
                    currentSlideTargetPosition += SLIDE_INCREMENT;
                } else if (gamepad1.dpad_left) {
                    currentSlideTargetPosition -= SLIDE_INCREMENT;
                }
                break;

            case IDLE:
            default:
                // If idle, reset target positions
                currentLiftTargetPosition = 0;
                currentSlideTargetPosition = 0;
                break;
        }
    }

    public void stateAction() {
        // Apply the target positions to the hardware
        hardware.setLiftTargetPosition(currentLiftTargetPosition);
        hardware.setHorizontalSlideTargetPosition(currentSlideTargetPosition);
        hardware.applyTargetPositions();
    }
}
