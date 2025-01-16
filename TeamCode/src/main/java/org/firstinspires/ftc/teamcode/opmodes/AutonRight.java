// This file is the code for when the robot is on the RIGHT side of the field

package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Onbot_HardwareITD;
import org.firstinspires.ftc.teamcode.statemachines.StateMachines;
import com.acmerobotics.roadrunner.*;

@Autonomous(name="AutonRight", group= "LinearOpMode")
public class AutonRight extends LinearOpMode {
    Onbot_HardwareITD robot = null;
    StateMachines robotState = null;
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

            robot = new Onbot_HardwareITD();
            robotState = new StateMachines(robot);

            robot.autoInitDrive(this);
            waitForStart();
            if(opModeIsActive()){

                robot.vertSlidesSet(800);
                robot.actionsJustForTheLift();
                sleep(750);

                robot.actionsJustForArms(0.6725);
                sleep(1500);

                robot.vertSlidesSet(0);
                robot.actionsJustForTheLift();
                sleep(1500);

                robot.sepIntakeFlip();
                sleep(1000);

                robot.claw.setPosition(robot.claw_closed);
                sleep(100);

                robot.actionsJustForArms(0.6);
                sleep(1250);

                robot.autoMovement(0,0,180,0.7);
                sleep(500);

                robot.autoMovement(0,50,0,0.7);
                sleep(750);

                robot.autoMovement(-38,0,0,0.7);
                sleep(1000);

                robot.actionsJustForArms(0.8);
                sleep(1000);

                robot.actionsJustForArms(0.6725);
                sleep(200);

                robot.claw.setPosition(robot.claw_open);
                sleep(1000);

                robot.autoMovement(30,0,0,0.5);
                sleep(250);

                robot.autoMovement(0,0,180,0.5);
                sleep(500);

                robot.autoMovement(0,60,0,0.4);
                sleep(500);

                robot.autoMovement(-7,0,0,0.5);
                sleep(250);
                robot.claw.setPosition(robot.claw_closed);

                sleep(10000);
            }
    }
}
