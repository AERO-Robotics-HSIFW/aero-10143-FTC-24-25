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
            robot.autoMovement(0,-23.5,0,0.5);
            sleep(100);
            robot.autoMovement(0,0,20,0.5);
            robot.vertSlidesSet(robot.top);
            while(robot.lift1.getCurrentPosition() < robot.top-500);
            robot.autoMovement(6,0,0,0.5);
            robot.horiSlidesSet(robot.horiMax);
            sleep(100);
            runtime.reset();
            while(robot.horiCurrent < robot.horiMax -1000 );
            while(opModeIsActive() && runtime.milliseconds() < 5000 && !robot.colorThreshold()){
                robot.intakePowerSet(true,true);
                robot.intakePosSet(false);
                robot.actions();
            }
            robot.intakePowerSet(false,true);
            robot.intakePosSet(true);
            robot.horiSlidesSet(0);
            robot.autoMovement(-6,0,0,0.5);
            while(robot.horizontal.getCurrentPosition() > 100);
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
