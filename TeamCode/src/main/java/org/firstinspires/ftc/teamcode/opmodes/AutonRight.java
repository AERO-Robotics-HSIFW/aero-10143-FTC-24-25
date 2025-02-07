// This file is the code for when the robot is on the RIGHT side of the field

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.teamcode.hardware.HardwareITD;
import org.firstinspires.ftc.teamcode.statemachines.StateMachines;

@Autonomous(name="AutonRight", group= "LinearOpMode")
public class AutonRight extends LinearOpMode {
    HardwareITD robot = null;
    StateMachines robotState = null;
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new HardwareITD();
        robotState = new StateMachines(robot);
        robot.initDrive(this);
        waitForStart();
    }
}
