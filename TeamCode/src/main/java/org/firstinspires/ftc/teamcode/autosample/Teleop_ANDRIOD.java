package org.firstinspires.ftc.teamcode.autosample;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.HardwareTestbot;

@TeleOp(name="DRIVE-MODE-ANDRIOD", group="testbot")
//@Disabled
public class Teleop_ANDRIOD extends LinearOpMode {

    /* Declare OpMode members. */
    ElapsedTime runtime = new ElapsedTime();
    HardwareTestbot robot = new HardwareTestbot();   // Use a Pushbot's hardware
    FtcDashboard dashboard =  FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    boolean intake_mode = false;
    boolean intake_lock = false;
    boolean outtake_mode = false;
    boolean outtake_lock = false;
    double timeMark = 0.0;



    public enum LiftState{
        LIFT_START,
        INTAKE_START,
        INTAKE_END,
        PREPARE_LIFT,
        GRAB,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT,
    }
    LiftState liftState = LiftState.LIFT_START;

    @Override
    public void runOpMode() {
        robot.initDrive(this);
        robot.encoderState("reset");
        robot.encoderState("run");

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();
        waitForStart();
        //run loop while button pressed
        while (opModeIsActive() && !isStopRequested()){

            /* GAMEPAD 1 */
            robot.roboCentric(-gamepad1.left_stick_y,-gamepad1.left_stick_x,gamepad1.right_stick_x);
            teleUpdate();
            
            if(gamepad1.left_stick_button){
                robot.resetHeading();
            }

            //drone
            if(gamepad1.b){
                robot.drone.setPosition(robot.drone_release);
            }
            
            //robot.lift(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.y);
            /*
            if(gamepad1.dpad_right){
                robot.turnToAngle(-90);
            } else if(gamepad1.dpad_left){
                robot.turnToAngle(90);
            } else if(gamepad1.dpad_up){
                robot.turnToAngle(180);
            } else if(gamepad1.dpad_down){
                robot.turnToAngle(180);
            }
            */


            // claw toggle
            if(gamepad1.right_bumper && !intake_lock && !intake_mode){
                telemetry.addData("arm position: ", robot.intakeArm.getPosition());
                telemetry.update();
                if (0.05 > robot.intakeArm.getPosition() - robot.intakeArm_closed) {
                    robot.intake1.setPosition(robot.intake1_open + 0.23);
                    robot.intake2.setPosition(robot.intake2_open - 0.20);
                    telemetry.addData("pleasese ", "YYA");
                }
                else {
                    robot.intake1.setPosition(robot.intake1_open);
                    robot.intake2.setPosition(robot.intake2_open);
                }

                intake_lock = true;
                intake_mode = true;

            } else if(gamepad1.right_bumper && !intake_lock && intake_mode ){
                robot.intake1.setPosition(robot.intake1_closed);
                robot.intake2.setPosition(robot.intake2_closed);
                intake_mode = false;
                intake_lock = true;

            }else if(!gamepad1.right_bumper && intake_lock){
                intake_lock = false;
            }

            // intake arm
            if(gamepad1.y){
                robot.intakeArm.setPosition(robot.intakeArm_closed);
            }

            if(gamepad1.a){
                robot.intakeArm.setPosition(robot.intakeArm_open);
            }

            switch(liftState){
                case LIFT_START:
                    if(gamepad1.x){
                        //outtake open
                        robot.outtake1.setPosition(robot.outtake1_small);
                        robot.outtake2.setPosition(robot.outtake2_small);

                        // slides go up
                        robot.lift1.setTargetPosition(400);
                        robot.lift2.setTargetPosition(400);
                        robot.lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.lift1.setPower(robot.MAX_POWER);
                        robot.lift2.setPower(robot.MAX_POWER);
                        runtime.reset();
                        liftState = LiftState.INTAKE_START;
                    }
                    break;
                case INTAKE_START:
                    if(Math.abs(robot.lift1.getCurrentPosition() - 400) < 10){
                        robot.arm1.setPosition(robot.arm1_closed);
                        robot.arm2.setPosition(robot.arm2_closed);
                        // intake arm goes in
                        robot.intakeArm.setPosition(robot.intakeArm_closed);
                        if(runtime.milliseconds() > 2000){
                            // claws open a little
                            robot.intake1.setPosition(robot.intake1_open + 0.23);
                            robot.intake2.setPosition(robot.intake2_open - 0.20);
                            runtime.reset();
                            liftState = LiftState.INTAKE_END;
                        }

                    }
                    break;
                case INTAKE_END:
                    if(runtime.milliseconds()>1000){
                        // intake arm goes out
                        robot.intakeArm.setPosition(robot.intakeArm_open);

                        runtime.reset();
                        liftState = LiftState.PREPARE_LIFT;
                    }
                    break;
                case PREPARE_LIFT:
                    if(runtime.milliseconds()>1000){
                        // claws open full
                        robot.intake1.setPosition(robot.intake1_open);
                        robot.intake2.setPosition(robot.intake2_open);
                        // slides go back down
                        robot.lift1.setTargetPosition(robot.lift1_down);
                        robot.lift2.setTargetPosition(robot.lift2_down);
                        robot.lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.lift1.setPower(robot.MAX_POWER);
                        robot.lift2.setPower(robot.MAX_POWER);
                        if(runtime.milliseconds()>1750){
                            robot.arm1.setPosition(0.12);
                            robot.arm2.setPosition(0.88);
                            runtime.reset();
                            liftState = LiftState.GRAB;
                        }
                    }
                    break;
                case GRAB:
                    if(runtime.milliseconds() > 1000){
                        // outtake grabbers grab
                        robot.outtake1.setPosition(robot.outtake1_closed);
                        robot.outtake2.setPosition(robot.outtake2_closed);
                        if(gamepad2.dpad_up) {
                            robot.lift1.setTargetPosition(robot.lift1_up);
                            robot.lift2.setTargetPosition(robot.lift2_up);
                            robot.lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.lift1.setPower(robot.MAX_POWER);
                            robot.lift2.setPower(robot.MAX_POWER);
                            robot.arm1.setPosition(robot.arm1_open);
                            robot.arm2.setPosition(robot.arm2_open);
                            runtime.reset();
                            liftState = LiftState.LIFT_EXTEND;
                        }
                    }
                    break;
                case LIFT_EXTEND:
                    if(gamepad2.right_bumper){
                        robot.outtake1.setPosition(robot.outtake1_open);
                        robot.outtake2.setPosition(robot.outtake2_open);
                        runtime.reset();
                        liftState = LiftState.LIFT_DUMP;
                    }
                    break;
                case LIFT_DUMP:
                    if(runtime.milliseconds() > 750){
                        robot.arm1.setPosition(robot.arm1_closed);
                        robot.arm2.setPosition(robot.arm2_closed);
                        robot.lift1.setTargetPosition(0);
                        robot.lift2.setTargetPosition(0);
                        robot.lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.lift1.setPower(robot.MAX_POWER);
                        robot.lift2.setPower(robot.MAX_POWER);
                        runtime.reset();
                        liftState = LiftState.LIFT_RETRACT;
                    }
                    break;
                case LIFT_RETRACT:
                    if(robot.lift1.getCurrentPosition() < 200){
                        liftState = LiftState.LIFT_START;
                    }
                default:
                    liftState = LiftState.LIFT_START;
            }
            if(gamepad2.x && liftState != LiftState.LIFT_START){
                liftState = LiftState.LIFT_START;
            }
            // linear slides
            if(gamepad2.dpad_up){


                robot.lift1.setPower(-robot.MAX_POWER);
                robot.lift2.setPower(-robot.MAX_POWER);
            }

           else  if(gamepad2.dpad_down){


                robot.lift1.setPower(robot.MAX_POWER);
                robot.lift2.setPower(robot.MAX_POWER);
            }
            else {
                robot.lift1.setPower(0);
                robot.lift2.setPower(0);
            }

             //



            /* GAMEPAD 2 */
            // arm
            if(gamepad2.y){
                robot.arm1.setPosition(robot.arm1_open);
                robot.arm2.setPosition(robot.arm2_open);
            }

            if(gamepad2.a){
                robot.arm1.setPosition(robot.arm1_closed);
                robot.arm2.setPosition(robot.arm2_closed);
            }

            // outtake fingers
            if(gamepad2.right_bumper && !outtake_lock && !outtake_mode){
                if (0.1 > robot.arm2.getPosition() - robot.arm1_closed) {
                    robot.outtake1.setPosition(0.7);
                    robot.outtake2.setPosition(0.3);
                }
                else {
                    robot.outtake1.setPosition(robot.outtake1_open);
                    robot.outtake2.setPosition(robot.outtake2_open);
                }

                outtake_lock = true;
                outtake_mode = true;

            } else if(gamepad2.right_bumper && !outtake_lock && outtake_mode){
                robot.outtake1.setPosition(robot.outtake1_closed);
                robot.outtake2.setPosition(robot.outtake2_closed);
                outtake_mode = false;
                outtake_lock = true;

            }else if(!gamepad2.right_bumper && outtake_lock){
                outtake_lock = false;
            }

             //




            //robot.lift(-gamepad2.left_stick_y>0.5, gamepad2.left_stick_y>0.5, gamepad2.left_stick_button);

            telemetry.addData("lift 1 position: ", robot.lift1.getCurrentPosition());
            telemetry.addData("lift 2 position: ", robot.lift2.getCurrentPosition());
            telemetry.update();
        }
    }
    private void teleUpdate(){
        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        dashboardTelemetry.addData("Heading:",botHeading);
        dashboardTelemetry.update();
    }
}