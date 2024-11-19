package org.firstinspires.ftc.teamcode.autosample;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.hardware.HardwareTestbot;
import org.firstinspires.ftc.teamcode.vision.CustomProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name="VisionPortal_Test", group="Linear OpMode")
public class CameraOpmode extends LinearOpMode {
  private CustomProcessor color;
  private HardwareTestbot robot;
  private VisionPortal visionPortal;

  private ElapsedTime runtime = new ElapsedTime();

  public static double forwardDist = 0;
  public static double sideDist = 0;
  public static double time = 2000;
  public static double turn = 0;

  public double satRectLeft = 0;
  public double satRectMiddle = 0;
  public double satRectRight = 0;



  @Override
  public void runOpMode() throws InterruptedException {
    robot = new HardwareTestbot();
    robot.initDrive(this);

    color = new CustomProcessor();
    visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), color);
    setManualExposure(6, 100);  // Use low exposure time to reduce motion blur
    ;

    waitForStart();
    runtime.reset();
    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      sleep(1000);
      robot.imu.resetYaw();

      satRectMiddle = color.getLeft();
      satRectRight = color.getRight();


      // MOVEEEEEEEEE  STRAFEEE

      satRectLeft = color.getLeft();


      int selection = getSelection();




    }
  }

  public void score_line(){

  }
  public void score1() {

    robot.intake1.setPosition(robot.intake1_open);
    robot.intake2.setPosition(robot.intake2_open);
    sleep(2000);

    robot.intakeArm.setPosition(robot.intakeArm_closed);
    robot.intake1.setPosition(robot.intake1_closed);
    robot.intake2.setPosition(robot.intake2_closed);

  }

  public void score(){
    robot.lift1.setTargetPosition(robot.lift1_up);
    robot.lift2.setTargetPosition(robot.lift2_up);
    robot.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.lift1.setPower(robot.MAX_POWER);
    robot.lift2.setPower(robot.MAX_POWER);
    sleep(2500);
    robot.arm1.setPosition(robot.arm1_open);
    robot.arm2.setPosition(robot.arm2_open);
    sleep(1000);
    robot.outtake1.setPosition(robot.outtake1_open);
    robot.outtake2.setPosition(robot.outtake2_open);
    sleep(1000);
    robot.outtake1.setPosition(robot.outtake1_closed);
    robot.outtake2.setPosition(robot.outtake2_closed);
    robot.arm1.setPosition(robot.arm1_closed);
    robot.arm2.setPosition(robot.arm2_closed);
    sleep(2000);
    robot.lift1.setTargetPosition(robot.lift1_down);
    robot.lift2.setTargetPosition(robot.lift2_down);
    robot.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.lift1.setPower(robot.MAX_POWER);
    robot.lift2.setPower(robot.MAX_POWER);
    sleep(2500);
    robot.distanceDrive(0,10,0,0.5);
  }
  public void drive_by_time(double forw, double side, double spin, long time){
    double FLPow = forw + side + spin;
    double FRPow = +forw - side - spin;

    double RLPow = -forw - side + spin;
    double RRPow = -forw + side - spin;
    // normalize all motor speeds so no values exceeds 100%.
    FLPow = Range.clip(FLPow, -0.5, 0.5);
    FRPow = Range.clip(FRPow, -0.5, 0.5);
    RLPow = Range.clip(RLPow, -0.5, 0.5);
    RRPow = Range.clip(RRPow, -0.5, 0.5);
    // Set drive motor power levels.
    robot.frontLeft.setPower(FLPow);
    robot.frontRight.setPower(FRPow);
    robot.backLeft.setPower(RLPow);
    robot.backRight.setPower(RRPow);
    runtime.reset();
    while(runtime.time()<time){
    }
    robot.frontLeft.setPower(0);
    robot.frontRight.setPower(0);
    robot.backLeft.setPower(0);
    robot.backRight.setPower(0);
  }
  private void setManualExposure(int exposureMS, int gain) {
    // Wait for the camera to be open, then use the controls

    if (visionPortal == null) {
      return;
    }

    // Make sure camera is streaming before we try to set the exposure controls
    if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
      telemetry.addData("Camera", "Waiting");
      telemetry.update();
      while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
        sleep(20);
      }
      telemetry.addData("Camera", "Ready");
      telemetry.update();
    }

    // Set camera controls unless we are stopping.
    if (!isStopRequested())
    {
      ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
      if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
        exposureControl.setMode(ExposureControl.Mode.Manual);
        sleep(50);
      }
      exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
      sleep(20);
      GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
      gainControl.setGain(gain);
      sleep(20);
    }
  }

  public int getSelection(){
    if (satRectLeft > satRectMiddle && satRectLeft > satRectRight) {
      return 1; // LEFT
    }
    else if (satRectMiddle > satRectLeft && satRectMiddle > satRectRight) {
      return 2; // MIDDLE
    }
    else return 3; // RIGHT

  }

}