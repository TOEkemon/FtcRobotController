package org.firstinspires.ftc.teamcode.teleOp;

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
  private DcMotor kicker_motor; // New kicker motor
  private Servo ball_push;
  private Servo wheel_rotation;

  // Variables to track button states
  private boolean aPressed = false;
  private boolean xPressed = false;
  private boolean bPressed = false;
  private boolean yPressed = false;
  private boolean rightBumperPressed = false;
  private boolean leftBumperPressed = false;     // Track left bumper for clockwise kicker motor
  private boolean dpadLeftPressed = false;       // Track D-pad left for clockwise kicker motor
  private boolean dpadRightPressed = false;      // Track D-pad right for counter-clockwise kicker motor
  private boolean lastRightTriggerPressed = false;
  private boolean lastLeftTriggerPressed = false;

  // Variables for kicker motor timing and state
  private boolean kickerActive = false;           // Flag to indicate if kicker is currently active
  private long kickerStartTime = 0;              // Time when kicker started
  private static final double KICKER_POWER_CLOCKWISE = 1.0; // Clockwise at full speed (1.0)
  private static final double KICKER_POWER_COUNTER_CLOCKWISE = -1.0; // Counter-clockwise at full speed (-1.0)
  private static final double KICKER_DURATION = 2.0; // Duration in seconds (2000 ms)

  // Variables for servo positions
  private double currentWheelPosition = 0.0; // Start at minimum position
  private double currentBallPushPosition = 1.0; // Start open position

  @Override
  public void runOpMode() {
    // Initialize hardware
    try {
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");
        intakemotor = hardwareMap.get(DcMotor.class, "intake_motor");
        shooter_motor = hardwareMap.get(DcMotor.class, "shooter_motor");
        kicker_motor = hardwareMap.get(DcMotor.class, "kicker_motor"); // Initialize kicker motor
        ball_push = hardwareMap.get(Servo.class, "ball_push");
        wheel_rotation = hardwareMap.get(Servo.class, "wheel_rotation");
    } catch (Exception e) {
        telemetry.addData("Error", "Hardware mapping failed: " + e.getMessage());
        telemetry.update();
        return; // Exit if hardware mapping fails
    }

    // Configure motors
    if (back_left_motor != null) back_left_motor.setDirection(DcMotor.Direction.REVERSE);
    if (front_left_motor != null) front_left_motor.setDirection(DcMotor.Direction.REVERSE);

    // Set motors to brake mode for better control
    if (back_left_motor != null) back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    if (front_left_motor != null) front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    if (back_right_motor != null) back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    if (front_right_motor != null) front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    if (shooter_motor != null) shooter_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    if (kicker_motor != null) kicker_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Configure kicker motor brake mode

    // Initialize servos to safe positions
    if (ball_push != null) ball_push.setPosition(currentBallPushPosition);
    // Set wheel rotation to center initially
    if (wheel_rotation != null) wheel_rotation.setPosition(currentWheelPosition);

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Wait for the game to start
    waitForStart();

    // Main loop
    while (opModeIsActive()) {
      // Read joystick values
      float verticalPower = -gamepad1.left_stick_y; // Negative to correct direction
      float horizontalPower = gamepad1.left_stick_x;
      float pivot = gamepad1.right_stick_x;

      // Apply dead zone to prevent drift
      if (Math.abs(verticalPower) < 0.1) verticalPower = 0;
      if (Math.abs(horizontalPower) < 0.1) horizontalPower = 0;
      if (Math.abs(pivot) < 0.1) pivot = 0;

      // Calculate motor powers for Mecanum drive
      double frontLeftPower = pivot + verticalPower + horizontalPower;
      double rearLeftPower = pivot + verticalPower - horizontalPower;
      double frontRightPower = -pivot + verticalPower - horizontalPower;
      double rearRightPower = -pivot + verticalPower + horizontalPower;

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
      if (front_left_motor != null) front_left_motor.setPower(frontLeftPower);
      if (back_left_motor != null) back_left_motor.setPower(rearLeftPower);
      if (front_right_motor != null) front_right_motor.setPower(frontRightPower);
      if (back_right_motor != null) back_right_motor.setPower(rearRightPower);

      // Handle intake motor controls
      if (intakemotor != null) {
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
      }

      // Handle wheel rotation servo - 120 degree rotation (1/3 of full range)
      if (wheel_rotation != null) {
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
      }

      // Handle ball push servo
      if (ball_push != null) {
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
      }

      // Handle shooter motor with triggers
      boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
      boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;

      if (shooter_motor != null) {
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
      }

      // Update trigger state for next iteration
      lastRightTriggerPressed = rightTriggerPressed;
      lastLeftTriggerPressed = leftTriggerPressed;

      // Handle kicker motor - activated with D-pad for different directions
      // D-pad left moves clockwise, D-pad right moves counter-clockwise
      if (kicker_motor != null) {
          // Handle clockwise rotation (D-pad left)
          if (gamepad1.dpad_left && !dpadLeftPressed) {
              kicker_motor.setPower(KICKER_POWER_CLOCKWISE); // Clockwise at full speed
              kickerActive = true;
              kickerStartTime = System.currentTimeMillis(); // Record start time
              dpadLeftPressed = true; // Mark that D-pad left is pressed
          }
          // Handle counter-clockwise rotation (D-pad right)
          else if (gamepad1.dpad_right && !dpadRightPressed) {
              kicker_motor.setPower(KICKER_POWER_COUNTER_CLOCKWISE); // Counter-clockwise at full speed
              kickerActive = true;
              kickerStartTime = System.currentTimeMillis(); // Record start time
              dpadRightPressed = true; // Mark that D-pad right is pressed
          }
          // Reset button states when released
          else if (!gamepad1.dpad_left) {
              dpadLeftPressed = false; // Reset when D-pad left is released
          }
          else if (!gamepad1.dpad_right) {
              dpadRightPressed = false; // Reset when D-pad right is released
          }

          // Check if kicker is active and duration has elapsed
          if (kickerActive) {
              long currentTime = System.currentTimeMillis();
              if ((currentTime - kickerStartTime) >= (KICKER_DURATION * 1000)) {
                  // Duration has elapsed, stop the kicker motor
                  kicker_motor.setPower(0.0);
                  kickerActive = false;
              }
          }
      }

      // Send telemetry data to driver station
      telemetry.addData("Vertical Power", "%.2f", verticalPower);
      telemetry.addData("Horizontal Power", "%.2f", horizontalPower);
      telemetry.addData("Pivot Power", "%.2f", pivot);
      telemetry.addData("Wheel Position", "%.2f", currentWheelPosition);
      telemetry.addData("Ball Push Position", "%.2f", currentBallPushPosition);
      if (intakemotor != null) {
          telemetry.addData("Intake Motor Power", "%.2f", intakemotor.getPower());
      } else {
          telemetry.addData("Intake Motor", "Not Found");
      }
      if (shooter_motor != null) {
          telemetry.addData("Shooter Motor Power", "%.2f", shooter_motor.getPower());
      } else {
          telemetry.addData("Shooter Motor", "Not Found");
      }
      if (kicker_motor != null) {
          telemetry.addData("Kicker Motor Power", "%.2f", kicker_motor.getPower());
          telemetry.addData("Kicker Active", kickerActive);
          if (kickerActive) {
              long currentTime = System.currentTimeMillis();
              double elapsedTime = (currentTime - kickerStartTime) / 1000.0; // Convert to seconds
              telemetry.addData("Kicker Elapsed Time", "%.2f", elapsedTime);

              // Indicate which direction the kicker is rotating
              if (kicker_motor.getPower() > 0) {
                  telemetry.addData("Kicker Direction", "Clockwise");
              } else if (kicker_motor.getPower() < 0) {
                  telemetry.addData("Kicker Direction", "Counter-Clockwise");
              }
          }
      } else {
          telemetry.addData("Kicker Motor", "Not Found");
      }
      telemetry.update();
    }

    // When exiting the loop (op mode ending), ensure all motors are stopped
    if (back_left_motor != null) back_left_motor.setPower(0);
    if (front_left_motor != null) front_left_motor.setPower(0);
    if (front_right_motor != null) front_right_motor.setPower(0);
    if (back_right_motor != null) back_right_motor.setPower(0);
    if (intakemotor != null) intakemotor.setPower(0);
    if (shooter_motor != null) shooter_motor.setPower(0);
    if (kicker_motor != null) kicker_motor.setPower(0); // Stop kicker motor on shutdown
    // Ensure ball push servo returns to safe position
    if (ball_push != null) ball_push.setPosition(1.0);
  }
}