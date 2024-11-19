// This file is the code for when the robot is on the LEFT side of the field

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Onbot_HardwareITD;
import org.firstinspires.ftc.teamcode.statemachines.StateMachines;

@Autonomous(name="AutonLeft", group= "Autonomous")
public class AutonLeft extends OpMode {
    Onbot_HardwareITD robot = null;
    StateMachines robotState = null;

    @Override
    public void init() {
        robot = new Onbot_HardwareITD();
        robotState = new StateMachines(robot);

    }

    @Override
    public void loop() {

    }
}
