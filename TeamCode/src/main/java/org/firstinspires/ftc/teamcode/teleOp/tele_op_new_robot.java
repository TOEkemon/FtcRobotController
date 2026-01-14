package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "tele_op_new_robot (Blocks to Java)")
public class tele_op_new_robot extends LinearOpMode {

  private DcMotor back_left_motor;
  private DcMotor front_left_motor;
  private DcMotor front_right_motor;
  private DcMotor back_right_motor;
  private DcMotor intakemotor;
  private DcMotor shooter_motor;
  private Servo ball_push;
  private Servo wheel_rotation;

  // Variables to track button states
  private boolean aPressed = false;
  private boolean xPressed = false;
  private boolean bPressed = false;
  private boolean yPressed = false;
  private boolean rightBumperPressed = false;
  private boolean lastRightTriggerPressed = false;
  private boolean lastLeftTriggerPressed = false;

  // Variables for servo positions
  private double currentWheelPosition = 0.0; // Start at minimum position
  private double currentBallPushPosition = 1.0; // Start open position

  @Override
  public void runOpMode() {
    float vertical_power;
    float horizontal_power;
    float pivot;

    // Initialize hardware
    back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
    front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
    front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
    back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");
    intakemotor = hardwareMap.get(DcMotor.class, "intake motor");
    shooter_motor = hardwareMap.get(DcMotor.class, "shooter_motor");
    ball_push = hardwareMap.get(Servo.class, "ball_push");
    wheel_rotation = hardwareMap.get(Servo.class, "wheel_rotation");

    // Configure motors
    back_left_motor.setDirection(DcMotor.Direction.REVERSE);
    front_left_motor.setDirection(DcMotor.Direction.REVERSE);

    // Set motors to brake mode for better control
    back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Initialize servos to safe positions
    ball_push.setPosition(currentBallPushPosition);
    // Set wheel rotation to center initially
    wheel_rotation.setPosition(currentWheelPosition);

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Wait for the game to start
    waitForStart();

    // Main loop
    while (opModeIsActive()) {
      // Read joystick values
      vertical_power = -gamepad1.left_stick_y; // Negative to correct direction
      horizontal_power = gamepad1.left_stick_x;
      pivot = gamepad1.right_stick_x;

      // Apply deadzone to prevent drift
      if (Math.abs(vertical_power) < 0.1) vertical_power = 0;
      if (Math.abs(horizontal_power) < 0.1) horizontal_power = 0;
      if (Math.abs(pivot) < 0.1) pivot = 0;

      // Calculate motor powers for mecanum drive
      double frontLeftPower = pivot + vertical_power + horizontal_power;
      double rearLeftPower = pivot + vertical_power - horizontal_power;
      double frontRightPower = -pivot + vertical_power - horizontal_power;
      double rearRightPower = -pivot + vertical_power + horizontal_power;

      // Find the largest power value to normalize if any motor exceeds 1.0
      double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(rearLeftPower));
      maxPower = Math.max(maxPower, Math.abs(frontRightPower));
      maxPower = Math.max(maxPower, Math.abs(rearRightPower));

      if (maxPower > 1.0) {
          frontLeftPower /= maxPower;
          rearLeftPower /= maxPower;
          frontRightPower /= maxPower;
          rearRightPower /= maxPower;
      }

      // Set motor powers
      front_left_motor.setPower(frontLeftPower);
      back_left_motor.setPower(rearLeftPower);
      front_right_motor.setPower(frontRightPower);
      back_right_motor.setPower(rearRightPower);

      // Handle intake motor controls
      if (gamepad1.a && !aPressed) {
          intakemotor.setPower(1.0); // Intake in
          aPressed = true;
      } else if (!gamepad1.a) {
          aPressed = false;
      }

      if (gamepad1.x && !xPressed) {
          intakemotor.setPower(-1.0); // Intake out
          xPressed = true;
      } else if (!gamepad1.x) {
          xPressed = false;
      }

      // Stop intake when neither A nor X is pressed
      if (!gamepad1.a && !gamepad1.x) {
          intakemotor.setPower(0.0);
      }

      // Handle wheel rotation servo - 120 degree rotation (1/3 of full range)
      if (gamepad1.b && !bPressed) {
          // Move servo 1/3 of the way toward one end (120 degrees worth of movement)
          currentWheelPosition += 0.333; // Approximately 1/3 of the servo range
          if (currentWheelPosition > 1.0) currentWheelPosition = 1.0;
          wheel_rotation.setPosition(currentWheelPosition);
          bPressed = true;
      } else if (!gamepad1.b) {
          bPressed = false;
      }

      if (gamepad1.y && !yPressed) {
          // Move servo 1/3 of the way toward the other end (120 degrees worth of movement)
          currentWheelPosition -= 0.333; // Approximately 1/3 of the servo range
          if (currentWheelPosition < 0.0) currentWheelPosition = 0.0;
          wheel_rotation.setPosition(currentWheelPosition);
          yPressed = true;
      } else if (!gamepad1.y) {
          yPressed = false;
      }

      // Handle ball push servo
      if (gamepad1.right_bumper && !rightBumperPressed) {
          // Toggle ball push servo position
          if (currentBallPushPosition == 0.0) {
              currentBallPushPosition = 1.0; // Open position
          } else {
              currentBallPushPosition = 0.0; // Closed position
          }
          ball_push.setPosition(currentBallPushPosition);
          rightBumperPressed = true;
      } else if (!gamepad1.right_bumper) {
          rightBumperPressed = false;
      }

      // Handle shooter motor with triggers
      boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
      boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;

      if (rightTriggerPressed && !lastRightTriggerPressed) {
          // Right trigger pressed (and wasn't pressed before) - run shooter forward at 100%
          shooter_motor.setPower(1.0);
      } else if (leftTriggerPressed && !lastLeftTriggerPressed) {
          // Left trigger pressed (and wasn't pressed before) - run shooter backward at 100%
          shooter_motor.setPower(-1.0);
      } else if ((!rightTriggerPressed && !leftTriggerPressed) &&
                 (lastRightTriggerPressed || lastLeftTriggerPressed)) {
          // Neither trigger pressed, but one was pressed before - stop shooter
          shooter_motor.setPower(0.0);
      }

      // Update trigger state for next iteration
      lastRightTriggerPressed = rightTriggerPressed;
      lastLeftTriggerPressed = leftTriggerPressed;

      // Send telemetry data to driver station
      telemetry.addData("Vertical Power", "%.2f", vertical_power);
      telemetry.addData("Horizontal Power", "%.2f", horizontal_power);
      telemetry.addData("Pivot Power", "%.2f", pivot);
      telemetry.addData("Wheel Position", "%.2f", currentWheelPosition);
      telemetry.addData("Ball Push Position", "%.2f", currentBallPushPosition);
      telemetry.addData("Intake Motor Power", "%.2f", intakemotor.getPower());
      telemetry.addData("Shooter Motor Power", "%.2f", shooter_motor.getPower());
      telemetry.update();
    }
  }

  @Override
  public void stop() {
    // When stopping the op mode, ensure the ball push servo returns to position 1
    if(ball_push != null) {
      ball_push.setPosition(1.0);
    }
    super.stop();
  }
}