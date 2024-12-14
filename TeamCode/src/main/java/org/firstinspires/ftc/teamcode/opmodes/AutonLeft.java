// This file is the code for when the robot is on the LEFT side of the field

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.*;
import org.firstinspires.ftc.teamcode.hardware.Onbot_HardwareITD;
import org.firstinspires.ftc.teamcode.statemachines.StateMachines;


@Autonomous(name="AutonLeft", group= "Autonomous")
public class AutonLeft extends LinearOpMode {
    Onbot_HardwareITD robot = null;
    StateMachines robotState = null;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new Onbot_HardwareITD();
        robotState = new StateMachines(robot);
        robot.initDrive(this);
        waitForStart();
        if(opModeIsActive()){
            robot.autoMovement(20,0,0,1);
            sleep(100);
            robot.autoMovement(0,-23.5,0,0.5); //go to net zone
            sleep(100);
            robot.autoMovement(0,0,20,0.5); // orient to bucket + first sampple
            sleep(100);
            //robotState
            robot.autoMovement(6,0,0,0.5);
            sleep(100);
            runtime.reset();
            robot.autoMovement(-6,0,0,0.5);
            sleep(100);
            robot.autoMovement(0,0,-18.5,0.5);
            sleep(100);
            robot.autoMovement(6,0,0,0.5);
            sleep(100);
            robot.autoMovement(-6,0,0,0.5);
            sleep(100);
            robot.autoMovement(0,0,18.5,0.5);
            sleep(100);

        }
    }
}
