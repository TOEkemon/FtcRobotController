package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Autonomous (Blocks to Java)")
public class Autonomous extends LinearOpMode {

  private CRServo LeftShooterServo;
  private CRServo RightShooterServo;
  private DcMotor right;
  private DcMotor left;
  private DcMotor Shooter;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    LeftShooterServo = hardwareMap.get(CRServo.class, "Left Shooter Servo");
    RightShooterServo = hardwareMap.get(CRServo.class, "Right Shooter Servo");
    right = hardwareMap.get(DcMotor.class, "right");
    left = hardwareMap.get(DcMotor.class, "left");
    Shooter = hardwareMap.get(DcMotor.class, "Shooter");

    // Put initialization blocks here.
    LeftShooterServo.setDirection(CRServo.Direction.FORWARD);
    RightShooterServo.setDirection(CRServo.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // At 900 ms of backwards time 300 ms of correct time (first block) makes it go straight
      right.setPower(0.65);
      left.setPower(-0.65);
      sleep(300);
      right.setPower(-1);
      left.setPower(-1);
      sleep(610);
      right.setPower(0);
      left.setPower(0);
      Shooter.setPower(0.6);
      sleep(5000);
      LeftShooterServo.setPower(1);
      RightShooterServo.setPower(1);
      sleep(200);
      LeftShooterServo.setPower(-1);
      RightShooterServo.setPower(-1);
      sleep(500);
      LeftShooterServo.setPower(0);
      RightShooterServo.setPower(0);
      Shooter.setPower(0.65);
      sleep(2000);
      LeftShooterServo.setPower(1);
      RightShooterServo.setPower(1);
      right.setPower(1);
      left.setPower(1);
      sleep(175);
      right.setPower(0);
      left.setPower(0);
      RightShooterServo.setPower(-1);
      LeftShooterServo.setPower(-1);
      sleep(500);
      RightShooterServo.setPower(0);
      LeftShooterServo.setPower(0);
      Shooter.setPower(0.67);
      sleep(5000);
      RightShooterServo.setPower(1);
      LeftShooterServo.setPower(1);
      sleep(250);
      RightShooterServo.setPower(0);
      LeftShooterServo.setPower(0);
      Shooter.setPower(0);
    }
  }
}