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
  private DcMotor wheel_rotation;
  private Servo grip_servo_left;  // Left gripper servo
  private Servo grip_servo_right; // Right gripper servo

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

  // Variables for kicker motor position control
  private static final double KICKER_INCREMENT = 0.15; // Amount to move per button press
  private static final double KICKER_POWER = 0.8; // Power level for kicker movement
  private static final long KICKER_MOVE_DURATION_MS = 150; // Duration to run motor for each increment (0.15 sec)
  private double kickerPosition = 0.0; // Current position of kicker
  private static final double MAX_KICKER_POSITION = 1.0; // Maximum position limit
  private static final double MIN_KICKER_POSITION = -1.0; // Minimum position limit

  // Variables for kicker motor timing
  private boolean kickerMoving = false; // Is kicker currently moving
  private long kickerMoveStartTime = 0; // Time when kicker started moving
  private int kickerPhase = 0; // Phase: 0 = not moving, 1 = positive movement, 2 = negative movement
  private double kickerTargetPower = 0.0; // Target power for current movement

  // Variables for wheel rotation motor position control
  private static final double WHEEL_ROTATION_INCREMENT = 0.14; // Amount to move per button press (1/3 of range)
  private static final double WHEEL_ROTATION_POWER = 0.8; // Power level for wheel rotation movement
  private static final long WHEEL_ROTATION_MOVE_DURATION_MS = 150; // Duration to run motor for each increment (0.15 sec)
  private double wheelRotationPosition = 0.0; // Current position of wheel rotation
  private static final double MAX_WHEEL_ROTATION_POSITION = 1.0; // Maximum position limit
  private static final double MIN_WHEEL_ROTATION_POSITION = 0.0; // Minimum position limit

  // Variables for wheel rotation motor timing
  private boolean wheelRotationMoving = false; // Is wheel rotation currently moving
  private long wheelRotationMoveStartTime = 0; // Time when wheel rotation started moving
  private int wheelRotationPhase = 0; // Phase: 0 = not moving, 1 = positive movement, 2 = negative movement
  private double wheelRotationTargetPower = 0.0; // Target power for current movement

  // Constants for gripper servo positions
  private static final double GRIP_SERVO_LEFT_OPEN = 0.0;    // Left gripper servo open position
  private static final double GRIP_SERVO_LEFT_CLOSE = 1.0;   // Left gripper servo closed position
  private static final double GRIP_SERVO_RIGHT_OPEN = 0.0;   // Right gripper servo open position (changed from 1.0 to 0.0)
  private static final double GRIP_SERVO_RIGHT_CLOSE = 1.0;  // Right gripper servo closed position (changed from 0.0 to 1.0)

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
        wheel_rotation = hardwareMap.get(DcMotor.class, "wheel_rotation");
        grip_servo_left = hardwareMap.get(Servo.class, "grip_servo_left");  // Initialize left gripper servo
        grip_servo_right = hardwareMap.get(Servo.class, "grip_servo_right"); // Initialize right gripper servo
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
    if (wheel_rotation != null) wheel_rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Configure wheel rotation motor brake mode

    // Initialize servos to safe positions
    if (ball_push != null) ball_push.setPosition(currentBallPushPosition);
    // Initialize gripper servos to neutral position
    if (grip_servo_left != null) grip_servo_left.setPosition(0.5);
    if (grip_servo_right != null) grip_servo_right.setPosition(0.5);

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
      double frontLeftPower = pivot + verticalPower - horizontalPower;
      double rearLeftPower = pivot + verticalPower + horizontalPower;
      double frontRightPower = -pivot + verticalPower + horizontalPower;
      double rearRightPower = -pivot + verticalPower - horizontalPower;

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

      // Handle wheel rotation motor - 120 degree rotation (1/3 of full range)
      if (wheel_rotation != null) {
          // Check if we need to start a new movement for wheel rotation
          if (!wheelRotationMoving) {
              // Handle clockwise increment (B button)
              if (gamepad1.b && !bPressed) {
                  // Move wheel rotation position positively by increment
                  wheelRotationPosition += WHEEL_ROTATION_INCREMENT;
                  // Limit to maximum position
                  if (wheelRotationPosition > MAX_WHEEL_ROTATION_POSITION) {
                      wheelRotationPosition = MAX_WHEEL_ROTATION_POSITION;
                  }
                  // Start motor movement
                  wheelRotationTargetPower = WHEEL_ROTATION_POWER;
                  wheel_rotation.setPower(wheelRotationTargetPower);
                  wheelRotationMoving = true;
                  wheelRotationMoveStartTime = System.currentTimeMillis();
                  bPressed = true; // Mark that B button is pressed
              }
              // Handle counter-clockwise increment (Y button)
              else if (gamepad1.y && !yPressed) {
                  // Move wheel rotation position negatively by increment
                  wheelRotationPosition -= WHEEL_ROTATION_INCREMENT;
                  // Limit to minimum position
                  if (wheelRotationPosition < MIN_WHEEL_ROTATION_POSITION) {
                      wheelRotationPosition = MIN_WHEEL_ROTATION_POSITION;
                  }
                  // Start motor movement
                  wheelRotationTargetPower = -WHEEL_ROTATION_POWER; // Negative power for reverse direction
                  wheel_rotation.setPower(wheelRotationTargetPower);
                  wheelRotationMoving = true;
                  wheelRotationMoveStartTime = System.currentTimeMillis();
                  yPressed = true; // Mark that Y button is pressed
              }
          }
          // Check if current movement has completed
          else if (wheelRotationMoving) {
              long currentTime = System.currentTimeMillis();
              if ((currentTime - wheelRotationMoveStartTime) >= WHEEL_ROTATION_MOVE_DURATION_MS) {
                  // Regular incremental stepper movement completed
                  wheel_rotation.setPower(0.0);
                  wheelRotationMoving = false;

                  // Reset the appropriate button state
                  if (bPressed) {
                      bPressed = false;
                  } else if (yPressed) {
                      yPressed = false;
                  }
              }
          }
      }

      // Update button states when released
      if (!gamepad1.b) bPressed = false;
      if (!gamepad1.y) yPressed = false;

      //inertia for ball to be at trigger
      if (front_left_motor != null && front_right_motor != null && back_left_motor != null && back_right_motor != null) {
          if (gamepad1.right_bumper && !rightBumperPressed) {
             front_left_motor.setPower(-1); // go back to push ball
             front_right_motor.setPower(-1); 
             back_left_motor.setPower(-1);
             back_right_motor.setPower(-1);
             sleep(50);
             front_left_motor.setPower(0); // go forward to move ball t rtigger
             front_right_motor.setPower(0); 
             back_left_motor.setPower(0);
             back_right_motor.setPower(0);
             sleep(50);

          }
      }
      // Handle shooter motor and gripper servos with triggers
      boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
      boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;

      if (shooter_motor != null) {
          if (rightTriggerPressed && !lastRightTriggerPressed) {
              // Right trigger pressed (and wasn't pressed before) - run shooter forward at 100%
              shooter_motor.setPower(1);
              // Also move gripper servos: left to 1.0, right to 0.0
              if (grip_servo_left != null) grip_servo_left.setPosition(1.0);
              if (grip_servo_right != null) grip_servo_right.setPosition(0.0);
          } else if (leftTriggerPressed && !lastLeftTriggerPressed) {
              // Left trigger pressed (and wasn't pressed before) - run shooter backward at 100%
              shooter_motor.setPower(-1);
              // Also move gripper servos: left to 0.0, right to 1.0
              if (grip_servo_left != null) grip_servo_left.setPosition(0.0);
              if (grip_servo_right != null) grip_servo_right.setPosition(1.0);
          } else if ((!rightTriggerPressed && !leftTriggerPressed) &&
                     (lastRightTriggerPressed || lastLeftTriggerPressed)) {
              // Neither trigger pressed, but one was pressed before - stop shooter and set servos to neutral
              shooter_motor.setPower(0.0);
              // Also set gripper servos to neutral position (0.5) when stopping shooter
              if (grip_servo_left != null) grip_servo_left.setPosition(0.5);
              if (grip_servo_right != null) grip_servo_right.setPosition(0.5);
          }
      }

      // Update trigger state for next iteration
      lastRightTriggerPressed = rightTriggerPressed;
      lastLeftTriggerPressed = leftTriggerPressed;

      // Handle kicker motor - incremental stepper with D-pad
      // D-pad left moves kicker 0.15 units in positive direction
      // D-pad right moves kicker 0.15 units in negative direction
      // D-pad down triggers two-phase movement (out and back)
      
      // Check if we need to start a new movement - prioritize two-phase movement (D-pad down)
      if (kicker_motor != null && !kickerMoving && gamepad1.dpad_down) {
          // Start the two-phase movement sequence (out and back)
          kickerPhase = 1; // Start with positive movement
          kicker_motor.setPower(KICKER_POWER); // Positive power for first phase
          kickerMoving = true;
          kickerMoveStartTime = System.currentTimeMillis();
      }
      // Check if we need to start a new movement - incremental stepper (D-pad left/right)
      else if (kicker_motor != null && !kickerMoving) {
          // Handle clockwise increment (D-pad left)
          if (gamepad1.dpad_left && !dpadLeftPressed) {
              // Move kicker position positively by increment
              kickerPosition += KICKER_INCREMENT;
              // Limit to maximum position
              if (kickerPosition > MAX_KICKER_POSITION) {
                  kickerPosition = MAX_KICKER_POSITION;
              }
              // Start motor movement
              kickerTargetPower = KICKER_POWER;
              kicker_motor.setPower(kickerTargetPower);
              kickerMoving = true;
              kickerMoveStartTime = System.currentTimeMillis();
              dpadLeftPressed = true; // Mark that D-pad left is pressed
          }
          // Handle counter-clockwise increment (D-pad right)
          else if (gamepad1.dpad_right && !dpadRightPressed) {
              // Move kicker position negatively by increment
              kickerPosition -= KICKER_INCREMENT;
              // Limit to minimum position
              if (kickerPosition < MIN_KICKER_POSITION) {
                  kickerPosition = MIN_KICKER_POSITION;
              }
              // Start motor movement
              kickerTargetPower = -KICKER_POWER; // Negative power for reverse direction
              kicker_motor.setPower(kickerTargetPower);
              kickerMoving = true;
              kickerMoveStartTime = System.currentTimeMillis();
              dpadRightPressed = true; // Mark that D-pad right is pressed
          }
      }
      // Check if current movement has completed
      else if (kickerMoving) {
          long currentTime = System.currentTimeMillis();
          if ((currentTime - kickerMoveStartTime) >= KICKER_MOVE_DURATION_MS) {
              // Check if this is the two-phase movement (D-pad down)
              if (kickerPhase == 1) {
                  // First phase (positive movement) completed, start second phase (negative movement)
                  kickerPhase = 2;
                  kicker_motor.setPower(-KICKER_POWER); // Negative power for return movement
                  kickerMoveStartTime = System.currentTimeMillis();
              } else if (kickerPhase == 2) {
                  // Second phase (negative movement) completed, stop motor and reset
                  kicker_motor.setPower(0.0);
                  kickerMoving = false;
                  kickerPhase = 0; // Reset to initial state
              } else {
                  // Regular incremental stepper movement completed
                  kicker_motor.setPower(0.0);
                  kickerMoving = false;
                  
                  // Reset the appropriate button state
                  if (dpadLeftPressed) {
                      dpadLeftPressed = false;
                  } else if (dpadRightPressed) {
                      dpadRightPressed = false;
                  }
              }
          }
      }
      
      // Update button states when released
      if (!gamepad1.dpad_left) dpadLeftPressed = false;
      if (!gamepad1.dpad_right) dpadRightPressed = false;

      // Send telemetry data to driver station
      telemetry.addData("Vertical Power", "%.2f", verticalPower);
      telemetry.addData("Horizontal Power", "%.2f", horizontalPower);
      telemetry.addData("Pivot Power", "%.2f", pivot);
      telemetry.addData("Wheel Rotation Position", "%.2f", wheelRotationPosition);
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
          telemetry.addData("Kicker Position", "%.2f", kickerPosition);
          telemetry.addData("Kicker Moving", kickerMoving);

          // Indicate which direction the kicker is rotating
          if (kicker_motor.getPower() > 0) {
              telemetry.addData("Kicker Direction", "Clockwise");
          } else if (kicker_motor.getPower() < 0) {
              telemetry.addData("Kicker Direction", "Counter-Clockwise");
          } else {
              telemetry.addData("Kicker Status", "Stopped");
          }
      } else {
          telemetry.addData("Kicker Motor", "Not Found");
      }
      if (wheel_rotation != null) {
          telemetry.addData("Wheel Rotation Motor Power", "%.2f", wheel_rotation.getPower());
          telemetry.addData("Wheel Rotation Position", "%.2f", wheelRotationPosition);
          telemetry.addData("Wheel Rotation Moving", wheelRotationMoving);

          // Indicate which direction the wheel rotation is moving
          if (wheel_rotation.getPower() > 0) {
              telemetry.addData("Wheel Rotation Direction", "Clockwise");
          } else if (wheel_rotation.getPower() < 0) {
              telemetry.addData("Wheel Rotation Direction", "Counter-Clockwise");
          } else {
              telemetry.addData("Wheel Rotation Status", "Stopped");
          }
      } else {
          telemetry.addData("Wheel Rotation Motor", "Not Found");
      }

      // Add gripper servo telemetry
      if (grip_servo_left != null) {
          telemetry.addData("Grip Servo Left Pos", "%.2f", grip_servo_left.getPosition());
      } else {
          telemetry.addData("Grip Servo Left", "Not Found");
      }
      if (grip_servo_right != null) {
          telemetry.addData("Grip Servo Right Pos", "%.2f", grip_servo_right.getPosition());
      } else {
          telemetry.addData("Grip Servo Right", "Not Found");
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
    if (wheel_rotation != null) wheel_rotation.setPower(0); // Stop wheel rotation motor on shutdown
    // Ensure ball push servo returns to safe position
    if (ball_push != null) ball_push.setPosition(1.0);
    // Ensure gripper servos return to neutral position (0.5)
    if (grip_servo_left != null) grip_servo_left.setPosition(0.5);
    if (grip_servo_right != null) grip_servo_right.setPosition(0.5);
  }
}