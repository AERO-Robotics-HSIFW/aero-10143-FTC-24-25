// This file is the code for when the robot is on the RIGHT side of the field

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Onbot_HardwareITD;
import org.firstinspires.ftc.teamcode.statemachines.StateMachines;

@Autonomous(name="AutonRight", group= "LinearOpMode")
public class AutonRight extends LinearOpMode {
    Onbot_HardwareITD robot = null;
    StateMachines robotState = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Onbot_HardwareITD();
        robotState = new StateMachines(robot);

        robot.initDrive(this);
        waitForStart();

        while (opModeIsActive()) {
            robot.autoMovement(500,0);
        }
    }
}
