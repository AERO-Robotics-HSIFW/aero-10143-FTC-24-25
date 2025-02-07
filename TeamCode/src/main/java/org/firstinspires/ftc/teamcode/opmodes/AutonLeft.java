// This file is the code for when the robot is on the LEFT side of the field

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.*;
import org.firstinspires.ftc.teamcode.hardware.HardwareITD;
import org.firstinspires.ftc.teamcode.statemachines.StateMachines;


@Autonomous(name="AutonLeft", group= "Autonomous")
public class AutonLeft extends LinearOpMode {
    HardwareITD robot = null;
    StateMachines robotState = null;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new HardwareITD();
        robotState = new StateMachines(robot);
        robot.initDrive(this);
        waitForStart();
        if(opModeIsActive()){
            robot.clawState("closed");
            robot.actions();
            robot.autoMovement(20,0,0,1);
            sleep(100);
            robot.autoMovement(0,-28,0,0.5); //go to net zone
            sleep(100);
            robot.autoMovement(0,0,32,0.5); // orient to bucket + first sampple
            sleep(100);
            robot.vertSlidesSet(robot.top);
            robot.actions();
            while(robot.lift1.getCurrentPosition() < robot.top-10);
            sleep(250);
            robot.armsPos("out");
            robot.actions();
            sleep(1500);
            robot.clawState("open");
            robot.actions();
            sleep(500);
            robot.armsPos("in");
            robot.actions();
            sleep(500);
            robot.vertSlidesSet(0);
            robot.actions();
            /*
            robot.horiSlidesSet(robot.horiMax);
            robot.actions();
            sleep(250);
            while(runtime.seconds()>7.5&&!robot.colorThreshold() && opModeIsActive()){
                robot.intakePosSet(false);
                robot.intakePowerSet(true,true);
                robot.intakePower = 0.75;
                robot.actions();
            }
            robot.horiSlidesSet(0);
            while(robot.horizontal.getCurrentPosition() > 10);
            robot.intakePosSet(true);
            sleep(250);
            robot.intakePowerSet(true,true);
            sleep(1500);
            robot.clawState("closed");
            robot.vertSlidesSet(robot.top);
            robot.actions();
            sleep(250);
            robot.armsPos("out");
            robot.actions();
            sleep(500);
            robot.clawState("open");
            robot.actions();
            sleep(250);
            robot.armsPos("in");
            robot.actions();
            sleep(250);
            robot.vertSlidesSet(0);



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
            */

        }
    }
}
