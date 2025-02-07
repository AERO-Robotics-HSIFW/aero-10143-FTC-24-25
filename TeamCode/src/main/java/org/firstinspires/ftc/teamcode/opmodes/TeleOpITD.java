/*
Copyright 2024 FIRST Tech Challenge Team 10143

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.HardwareITD;
import org.firstinspires.ftc.teamcode.statemachines.StateMachines;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this OpMode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp(name="MainCode", group= "Linear OpMode")
public class TeleOpITD extends LinearOpMode {


    ElapsedTime runtime = new ElapsedTime();
    HardwareITD robot = new HardwareITD();
    StateMachines states = new StateMachines(robot);

    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.initDrive(this);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        int num = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            states.inputTranslation(gamepad1,gamepad2);
            states.stateMachineLogic();
            states.stateAction();
            if(gamepad1.x){
                robot.roboCentric(-gamepad1.left_stick_y/2, gamepad1.left_stick_x/2, gamepad1.right_stick_x/2);
            }
            else{
                robot.roboCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            states.teamColor = robot.toggleStrings(gamepad2.left_trigger > 0.75, "red", "blue", states.teamColor);
            telemetry.addData("Team Color:", states.teamColor.toUpperCase());
            telemetry.addData("Horizontal Ticks:", robot.horizontal.getTargetPosition());
            telemetry.addData("Vertical Ticks:", robot.lift1.getTargetPosition());
            telemetry.addData("CURRENT STATE:", states.robotState);
            telemetry.addData("RED", robot.color.red());
            telemetry.addData("RED", robot.color.green());
            telemetry.addData("RED", robot.color.blue());
            telemetry.addData("CLAW POS D", robot.claw_current);
            telemetry.addData("CLAW POS A", robot.claw.getPosition());
            telemetry.update();
        }
    }
}