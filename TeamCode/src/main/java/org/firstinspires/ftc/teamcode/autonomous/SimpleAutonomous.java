package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Simple autonomous routine for DECODE game
 * Uses inches and degrees for movement with IMU telemetry
 * Supports alliance and side selection
 */
@Autonomous(name = "Simple Autonomous - Red Far", group = "Autonomous")
public class SimpleAutonomous extends LinearOpMode {

    // Drive motors
    private DcMotor front_left_motor;
    private DcMotor front_right_motor;
    private DcMotor back_left_motor;
    private DcMotor back_right_motor;
    
    // Shooter and intake motors
    private DcMotor intakemotor;
    private DcMotor shooter_motor;
    private DcMotor wheel_rotation;  // Barrel rotation motor
    private DcMotor kicker_motor;     // Arm motor for ball pickup (if needed)
    
    // Servos
    private Servo grip_servo_left;  // Left grip servo
    private Servo grip_servo_right; // Right grip servo
    private Servo ball_push;       // Ball pusher servo
    
    // IMU
    private IMU imu;
    
    // Alliance and side selection
    private boolean isRedAlliance = true;  // Default to red
    private boolean isFarSide = true;      // Default to far side
    
    // Constants for movement (in inches and degrees)
    private static final double TICKS_PER_INCH = 52.48; // Use your calibrated value from the test
    private static final double TICKS_PER_DEGREE = 11.5; // Approximate value - needs tuning
    
    // Barrel positions - these values need to be tuned for your specific servo
    // These are the positions for each ball shot (0.13, 0.13, 0.14 as you specified)
    private static final double[] BARREL_POSITIONS = {0.13, 0.26, 0.40}; // Incremental positions
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();
        
        // Alliance and side selection during init
        telemetry.addData(">", "Select Alliance and Side using Gamepad");
        telemetry.addData("A/B", "Red/Blue Alliance");
        telemetry.addData("X/Y", "Near/Far Side");
        telemetry.addData("Alliance", isRedAlliance ? "Red" : "Blue");
        telemetry.addData("Side", isFarSide ? "Far" : "Near");
        telemetry.update();
        
        while (opModeInInit()) {
            // Alliance selection
            if (gamepad1.a) {
                isRedAlliance = true;
                telemetry.addData("Alliance", "Red Selected");
            } else if (gamepad1.b) {
                isRedAlliance = false;
                telemetry.addData("Alliance", "Blue Selected");
            }
            
            // Side selection
            if (gamepad1.x) {
                isFarSide = false;  // Near side
                telemetry.addData("Side", "Near Selected");
            } else if (gamepad1.y) {
                isFarSide = true;   // Far side
                telemetry.addData("Side", "Far Selected");
            }
            
            // Update IMU telemetry
            updateIMUTelemetry();
            
            telemetry.update();
            sleep(50);
        }
        
        // Wait for start
        waitForStart();
        
        // Run the appropriate autonomous based on alliance and side
        if (isRedAlliance && isFarSide) {
            runRedFarAutonomous();
        } else if (!isRedAlliance && isFarSide) {
            runBlueFarAutonomous();
        } else if (isRedAlliance && !isFarSide) {
            // For now, just display message for unimplemented autonomous
            telemetry.addData("Status", "Red Near Autonomous not implemented yet");
            telemetry.update();
            while (opModeIsActive()) {
                updateIMUTelemetry();
                telemetry.update();
                sleep(100);
            }
        } else {
            // For now, just display message for unimplemented autonomous
            telemetry.addData("Status", "Blue Near Autonomous not implemented yet");
            telemetry.update();
            while (opModeIsActive()) {
                updateIMUTelemetry();
                telemetry.update();
                sleep(100);
            }
        }
    }
    
    /**
     * Initialize all hardware components
     */
    private void initializeHardware() {
        // Drive motors
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");
        
        // Set motor directions (adjust as needed for your robot)
        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        front_right_motor.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(DcMotor.Direction.FORWARD);
        
        // Shooter and intake motors
        intakemotor = hardwareMap.get(DcMotor.class, "intake_motor");
        shooter_motor = hardwareMap.get(DcMotor.class, "shooter_motor");
        kicker_motor = hardwareMap.get(DcMotor.class, "kicker_motor");
        
        // Servos
        wheel_rotation = hardwareMap.get(DcMotor.class, "wheel_rotation");
        ball_push = hardwareMap.get(Servo.class, "ball_push");
        
        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        
        // Set motors to run using encoders
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Red Alliance Far Side Autonomous
     */
    private void runRedFarAutonomous() {
        // Initial actions
        // Drive backward 40 inches
        driveDistance(-40.0, 0.5);  // Negative distance = backward
        
        // Start shooter motor at maximum speed
        shooter_motor.setPower(1.0);
        
        // Shoot 3 balls with barrel rotation
        for (int i = 0; i < 3; i++) {
            // Rotate barrel to next position (incremental)
            double barrelPosition = BARREL_POSITIONS[i];
            if (wheel_rotation != null) {
                wheel_rotation.setPower(150); // Rotate barrel to position
            }
            sleep(500); // Wait for servo to reach position
            
            // Activate ball pusher to shoot
            if (kicker_motor != null) {
                kicker_motor.setPower(-1.0);  // Ensure kicker is retracted before shooting
                sleep(150); // Hold retraction
                kicker_motor.setPower(1.0);  // Activate kicker motor
                sleep(150); // Hold kicker motor
                kicker_motor.setPower(0.0);  // Stop kicker motor
                if (grip_servo_left != null && grip_servo_right != null) {
                    // Open gripper to release any held balls (if needed)
                    grip_servo_left.setPosition(0.0);  // Open position
                    grip_servo_right.setPosition(1.0); // Open position
                    sleep(200); // Wait for gripper to open
                }
            }
            sleep(200); // Wait for retraction
        }
        
        // First ball pickup row
        // Turn 45 degrees clockwise (positive for clockwise)
        turnDegrees(45.0, 0.3);
        
        // Drive forward 25 inches while running intake
        if (intakemotor != null) {
            intakemotor.setPower(1.0); // Start intake
        }
        driveDistance(25.0, 0.6); // Forward while collecting
        
        // Drive backward 25 inches
        if (intakemotor != null) {
            intakemotor.setPower(0.0); // Stop intake
        }
        driveDistance(-25.0, 0.6); // Back to launch line
        
        // Turn 45 degrees clockwise to face launch line
        turnDegrees(45.0, 0.3);
        
        // Shoot collected balls (same as initial sequence)
        for (int i = 0; i < 3; i++) {
            // Rotate barrel to next position (incremental)
            double barrelPosition = BARREL_POSITIONS[i];
            if (wheel_rotation != null) {
                wheel_rotation.setPower(150); // Rotate barrel to position
            }
            sleep(500); // Wait for servo to reach position
            
            // Activate ball pusher to shoot
            if (kicker_motor != null) {
                kicker_motor.setPower(-1.0);  // Ensure kicker is retracted before shooting
                sleep(150); // Hold retraction
                kicker_motor.setPower(1.0);  // Activate kicker motor
                sleep(150); // Hold kicker motor
                kicker_motor.setPower(0.0);  // Stop kicker motor
                if (grip_servo_left != null && grip_servo_right != null) {
                    // Open gripper to release any held balls (if needed)
                    grip_servo_left.setPosition(0.0);  // Open position
                    grip_servo_right.setPosition(1.0); // Open position
                    sleep(200); // Wait for gripper to open
                }
            }
            sleep(200); // Wait for retraction
        }
        
        // Second ball pickup row
        // Turn 45 degrees clockwise
        turnDegrees(45.0, 0.3);
        
        // Drive right 18 inches
        strafeDistance(18.0, 0.5);
        
        // Drive forward 25 inches while running intake
        if (intakemotor != null) {
            intakemotor.setPower(1.0); // Start intake
        }
        driveDistance(25.0, 0.6); // Forward while collecting
        
        // Drive backward 25 inches
        if (intakemotor != null) {
            intakemotor.setPower(0.0); // Stop intake
        }
        driveDistance(-25.0, 0.6);
        
        // Drive left 18 inches
        strafeDistance(-18.0, 0.5);
        
        // Turn 45 degrees clockwise
        turnDegrees(45.0, 0.3);
        
        // Shoot collected balls
        for (int i = 0; i < 3; i++) {
            // Rotate barrel to next position (incremental)
            double barrelPosition = BARREL_POSITIONS[i];
            if (wheel_rotation != null) {
                wheel_rotation.setPower(150); // Rotate barrel to position
            }
            sleep(500); // Wait for servo to reach position
            
           // Activate ball pusher to shoot
            if (kicker_motor != null) {
                kicker_motor.setPower(-1.0);  // Ensure kicker is retracted before shooting
                sleep(150); // Hold retraction
                kicker_motor.setPower(1.0);  // Activate kicker motor
                sleep(150); // Hold kicker motor
                kicker_motor.setPower(0.0);  // Stop kicker motor
                if (grip_servo_left != null && grip_servo_right != null) {
                    // Open gripper to release any held balls (if needed)
                    grip_servo_left.setPosition(0.0);  // Open position
                    grip_servo_right.setPosition(1.0); // Open position
                    sleep(200); // Wait for gripper to open
                }
            }
            sleep(200); // Wait for retraction
        }
        
        // Third ball pickup row
        // Turn 45 degrees clockwise
        turnDegrees(45.0, 0.3);
        
        // Drive right 36 inches
        strafeDistance(36.0, 0.5);
        
        // Drive forward 25 inches while running intake
        if (intakemotor != null) {
            intakemotor.setPower(1.0); // Start intake
        }
        driveDistance(25.0, 0.6); // Forward while collecting
        
        // Drive backward 25 inches
        if (intakemotor != null) {
            intakemotor.setPower(0.0); // Stop intake
        }
        driveDistance(-25.0, 0.6);
        
        // Drive left 36 inches
        strafeDistance(-36.0, 0.5);
        
        // Turn 45 degrees clockwise
        turnDegrees(45.0, 0.3);
        
        // Shoot collected balls
        for (int i = 0; i < 3; i++) {
            // Rotate barrel to next position (incremental)
            double barrelPosition = BARREL_POSITIONS[i];
            if (wheel_rotation != null) {
                wheel_rotation.setPower(150); // Rotate barrel to position
            }
            sleep(500); // Wait for servo to reach position
            
            // Activate ball pusher to shoot
            if (kicker_motor != null) {
                kicker_motor.setPower(-1.0);  // Ensure kicker is retracted before shooting
                sleep(150); // Hold retraction
                kicker_motor.setPower(1.0);  // Activate kicker motor
                sleep(150); // Hold kicker motor
                kicker_motor.setPower(0.0);  // Stop kicker motor
                if (grip_servo_left != null && grip_servo_right != null) {
                    // Open gripper to release any held balls (if needed)
                    grip_servo_left.setPosition(0.0);  // Open position
                    grip_servo_right.setPosition(1.0); // Open position
                    sleep(200); // Wait for gripper to open
                }
            }
            sleep(200); // Wait for retraction
        }
        
        // Parking
        // Turn 45 degrees clockwise
        turnDegrees(45.0, 0.3);
        
        // Drive right 34 inches
        strafeDistance(34.0, 0.5);
        
        // Turn off all systems
        if (shooter_motor != null) {
            shooter_motor.setPower(0.0);
        }
        if (intakemotor != null) {
            intakemotor.setPower(0.0);
        }
    }
    
    /**
     * Blue Alliance Far Side Autonomous
     */
    private void runBlueFarAutonomous() {
        // Initial actions
        // Drive backward 40 inches (from facing backward)
        driveDistance(-40.0, 0.5);  // Negative distance = backward
        
        // Start shooter motor at maximum speed
        if (shooter_motor != null) {
            shooter_motor.setPower(1.0);
        }
        
        // Shoot 3 balls with barrel rotation
        for (int i = 0; i < 3; i++) {
            // Rotate barrel to next position (incremental)
            double barrelPosition = BARREL_POSITIONS[i];
            if (wheel_rotation != null) {
                wheel_rotation.setPower(150); // Rotate barrel to position
            }
            sleep(500); // Wait for servo to reach position
            
            // Activate ball pusher to shoot
            if (kicker_motor != null) {
                kicker_motor.setPower(-1.0);  // Ensure kicker is retracted before shooting
                sleep(150); // Hold retraction
                kicker_motor.setPower(1.0);  // Activate kicker motor
                sleep(150); // Hold kicker motor
                kicker_motor.setPower(0.0);  // Stop kicker motor
                if (grip_servo_left != null && grip_servo_right != null) {
                    // Open gripper to release any held balls (if needed)
                    grip_servo_left.setPosition(0.0);  // Open position
                    grip_servo_right.setPosition(1.0); // Open position
                    sleep(200); // Wait for gripper to open
                }
            }
            sleep(200); // Wait for retraction
           
        }
        
        // First ball pickup row
        // Turn 45 degrees counterclockwise (negative for counterclockwise)
        turnDegrees(-45.0, 0.3);
        
        // Drive forward 25 inches while running intake
        if (intakemotor != null) {
            intakemotor.setPower(1.0); // Start intake
        }
        driveDistance(25.0, 0.6); // Forward while collecting
        
        // Drive backward 25 inches
        if (intakemotor != null) {
            intakemotor.setPower(0.0); // Stop intake
        }
        driveDistance(-25.0, 0.6); // Back to launch line
        
        // Turn 45 degrees clockwise to face launch line
        turnDegrees(45.0, 0.3);
        
        // Shoot collected balls (same as initial sequence)
        for (int i = 0; i < 3; i++) {
            // Rotate barrel to next position (incremental)
            double barrelPosition = BARREL_POSITIONS[i];
            if (wheel_rotation != null) {
                wheel_rotation.setPower(150); // Rotate barrel to position
            }
            sleep(500); // Wait for servo to reach position
            
            // Activate ball pusher to shoot
            if (kicker_motor != null) {
                kicker_motor.setPower(-1.0);  // Ensure kicker is retracted before shooting
                sleep(150); // Hold retraction
                kicker_motor.setPower(1.0);  // Activate kicker motor
                sleep(150); // Hold kicker motor
                kicker_motor.setPower(0.0);  // Stop kicker motor
                if (grip_servo_left != null && grip_servo_right != null) {
                    // Open gripper to release any held balls (if needed)
                    grip_servo_left.setPosition(0.0);  // Open position
                    grip_servo_right.setPosition(1.0); // Open position
                    sleep(200); // Wait for gripper to open
                }
            }
            sleep(200); // Wait for retraction
        }
        
        // Second ball pickup row
        // Turn 45 degrees counterclockwise
        turnDegrees(-45.0, 0.3);
        
        // Drive left 18 inches (negative strafe = left)
        strafeDistance(-18.0, 0.5);
        
        // Drive forward 25 inches while running intake
        if (intakemotor != null) {
            intakemotor.setPower(1.0); // Start intake
        }
        driveDistance(25.0, 0.6); // Forward while collecting
        
        // Drive backward 25 inches
        if (intakemotor != null) {
            intakemotor.setPower(0.0); // Stop intake
        }
        driveDistance(-25.0, 0.6);
        
        // Drive right 18 inches (positive strafe = right)
        strafeDistance(18.0, 0.5);
        
        // Turn 45 degrees clockwise
        turnDegrees(45.0, 0.3);
        
        // Shoot collected balls
        for (int i = 0; i < 3; i++) {
            // Rotate barrel to next position (incremental)
            double barrelPosition = BARREL_POSITIONS[i];
            if (wheel_rotation != null) {
                wheel_rotation.setPower(150); // Rotate barrel to position
            }
            sleep(500); // Wait for servo to reach position
            
            // Activate ball pusher to shoot
            if (kicker_motor != null) {
                kicker_motor.setPower(-1.0);  // Ensure kicker is retracted before shooting
                sleep(150); // Hold retraction
                kicker_motor.setPower(1.0);  // Activate kicker motor
                sleep(150); // Hold kicker motor
                kicker_motor.setPower(0.0);  // Stop kicker motor
                if (grip_servo_left != null && grip_servo_right != null) {
                    // Open gripper to release any held balls (if needed)
                    grip_servo_left.setPosition(0.0);  // Open position
                    grip_servo_right.setPosition(1.0); // Open position
                    sleep(200); // Wait for gripper to open
                }
            }
            sleep(200); // Wait for retraction
        }
        
        // Third ball pickup row
        // Turn 45 degrees counterclockwise
        turnDegrees(-45.0, 0.3);
        
        // Drive left 36 inches
        strafeDistance(-36.0, 0.5);
        
        // Drive forward 25 inches while running intake
        if (intakemotor != null) {
            intakemotor.setPower(1.0); // Start intake
        }
        driveDistance(25.0, 0.6); // Forward while collecting
        
        // Drive backward 25 inches
        if (intakemotor != null) {
            intakemotor.setPower(0.0); // Stop intake
        }
        driveDistance(-25.0, 0.6);
        
        // Drive right 36 inches
        strafeDistance(36.0, 0.5);
        
        // Turn 45 degrees clockwise
        turnDegrees(45.0, 0.3);
        
        // Shoot collected balls
        for (int i = 0; i < 3; i++) {
            // Rotate barrel to next position (incremental)
            double barrelPosition = BARREL_POSITIONS[i];
            if (wheel_rotation != null) {
                wheel_rotation.setPower(150); // Rotate barrel to position
            }
            sleep(500); // Wait for servo to reach position
            
           // Activate ball pusher to shoot
            if (kicker_motor != null) {
                kicker_motor.setPower(-1.0);  // Ensure kicker is retracted before shooting
                sleep(150); // Hold retraction
                kicker_motor.setPower(1.0);  // Activate kicker motor
                sleep(150); // Hold kicker motor
                kicker_motor.setPower(0.0);  // Stop kicker motor
                if (grip_servo_left != null && grip_servo_right != null) {
                    // Open gripper to release any held balls (if needed)
                    grip_servo_left.setPosition(0.0);  // Open position
                    grip_servo_right.setPosition(1.0); // Open position
                    sleep(200); // Wait for gripper to open
                }
            }
            sleep(200); // Wait for retraction
        }
        
        // Parking
        // Turn 45 degrees counterclockwise
        turnDegrees(-45.0, 0.3);
        
        // Drive left 34 inches
        strafeDistance(-34.0, 0.5);
        
        // Turn off all systems
        if (shooter_motor != null) {
            shooter_motor.setPower(0.0);
        }
        if (intakemotor != null) {
            intakemotor.setPower(0.0);
        }
    }
    
    /**
     * Drive a specific distance in inches using encoders
     * @param distanceInches Distance to travel in inches (positive = forward, negative = backward)
     * @param power Motor power (0.0 to 1.0)
     */
    private void driveDistance(double distanceInches, double power) {
        // Calculate target encoder counts
        int targetTicks = (int)(Math.abs(distanceInches) * TICKS_PER_INCH);

        // Reset encoders
        resetEncoders();

        // Set target positions for all motors
        if (front_left_motor != null) front_left_motor.setTargetPosition(targetTicks);
        if (front_right_motor != null) front_right_motor.setTargetPosition(targetTicks);
        if (back_left_motor != null) back_left_motor.setTargetPosition(targetTicks);
        if (back_right_motor != null) back_right_motor.setTargetPosition(targetTicks);

        // Set motors to run to position
        setRunToPositionMode();

        // Set power with correct direction
        double direction = distanceInches >= 0 ? power : -power;
        if (front_left_motor != null) front_left_motor.setPower(direction);
        if (front_right_motor != null) front_right_motor.setPower(direction);
        if (back_left_motor != null) back_left_motor.setPower(direction);
        if (back_right_motor != null) back_right_motor.setPower(direction);

        // Wait until target is reached
        while (opModeIsActive() &&
               (front_left_motor != null && front_left_motor.isBusy()) &&
               (front_right_motor != null && front_right_motor.isBusy()) &&
               (back_left_motor != null && back_left_motor.isBusy()) &&
               (back_right_motor != null && back_right_motor.isBusy())) {
            updateIMUTelemetry();
            telemetry.addData("Driving", "%.1f inches", distanceInches);
            telemetry.addData("Target Ticks", targetTicks);
            if (front_left_motor != null) telemetry.addData("Current FL", front_left_motor.getCurrentPosition());
            if (front_right_motor != null) telemetry.addData("Current FR", front_right_motor.getCurrentPosition());
            if (back_left_motor != null) telemetry.addData("Current BL", back_left_motor.getCurrentPosition());
            if (back_right_motor != null) telemetry.addData("Current BR", back_right_motor.getCurrentPosition());
            telemetry.update();
            sleep(10);
        }
        
        // Stop motors
        stopDriveMotors();
    }
    
    /**
     * Strafe a specific distance in inches using encoders
     * @param distanceInches Distance to strafe in inches (positive = right, negative = left)
     * @param power Motor power (0.0 to 1.0)
     */
    private void strafeDistance(double distanceInches, double power) {
        // Calculate target encoder counts
        int targetTicks = (int)(Math.abs(distanceInches) * TICKS_PER_INCH);

        // Reset encoders
        resetEncoders();

        // For mecanum strafing: left motors move opposite of right motors
        // Right strafe: FL+, FR-, BL-, BR+
        // Left strafe: FL-, FR+, BL+, BR-
        if (front_left_motor != null) front_left_motor.setTargetPosition(distanceInches >= 0 ? targetTicks : -targetTicks);
        if (front_right_motor != null) front_right_motor.setTargetPosition(distanceInches >= 0 ? -targetTicks : targetTicks);
        if (back_left_motor != null) back_left_motor.setTargetPosition(distanceInches >= 0 ? -targetTicks : targetTicks);
        if (back_right_motor != null) back_right_motor.setTargetPosition(distanceInches >= 0 ? targetTicks : -targetTicks);

        // Set motors to run to position
        setRunToPositionMode();

        // Set power with correct direction
        double absPower = Math.abs(power);
        if (front_left_motor != null) front_left_motor.setPower(distanceInches >= 0 ? absPower : -absPower);
        if (front_right_motor != null) front_right_motor.setPower(distanceInches >= 0 ? -absPower : absPower);
        if (back_left_motor != null) back_left_motor.setPower(distanceInches >= 0 ? -absPower : absPower);
        if (back_right_motor != null) back_right_motor.setPower(distanceInches >= 0 ? absPower : -absPower);

        // Wait until target is reached
        while (opModeIsActive() &&
               (front_left_motor != null && front_left_motor.isBusy()) &&
               (front_right_motor != null && front_right_motor.isBusy()) &&
               (back_left_motor != null && back_left_motor.isBusy()) &&
               (back_right_motor != null && back_right_motor.isBusy())) {
            updateIMUTelemetry();
            telemetry.addData("Strafing", "%.1f inches", distanceInches);
            telemetry.addData("Target Ticks", targetTicks);
            if (front_left_motor != null) telemetry.addData("Current FL", front_left_motor.getCurrentPosition());
            if (front_right_motor != null) telemetry.addData("Current FR", front_right_motor.getCurrentPosition());
            if (back_left_motor != null) telemetry.addData("Current BL", back_left_motor.getCurrentPosition());
            if (back_right_motor != null) telemetry.addData("Current BR", back_right_motor.getCurrentPosition());
            telemetry.update();
            sleep(10);
        }
        
        // Stop motors
        stopDriveMotors();
    }
    
    /**
     * Turn a specific number of degrees using IMU
     * @param degrees Degrees to turn (positive = clockwise, negative = counterclockwise)
     * @param power Turn power (0.0 to 1.0)
     */
    private void turnDegrees(double degrees, double power) {
        if (imu == null) {
            telemetry.addData("ERROR", "IMU not initialized - skipping turn");
            telemetry.update();
            return;
        }
        
        // Get initial heading
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double initialHeading = orientation.getYaw(AngleUnit.DEGREES);
        
        // Calculate target heading
        double targetHeading = initialHeading + degrees;
        
        // Normalize target heading to -180 to 180 range
        while (targetHeading > 180) targetHeading -= 360;
        while (targetHeading <= -180) targetHeading += 360;
        
        // Calculate turn power with correct direction
        double turnPower = Math.abs(power);
        if (degrees < 0) {
            // Turn counterclockwise: left motors go backward, right motors go forward
            if (front_left_motor != null) front_left_motor.setPower(-turnPower);
            if (front_right_motor != null) front_right_motor.setPower(turnPower);
            if (back_left_motor != null) back_left_motor.setPower(-turnPower);
            if (back_right_motor != null) back_right_motor.setPower(turnPower);
        } else {
            // Turn clockwise: left motors go forward, right motors go backward
            if (front_left_motor != null) front_left_motor.setPower(turnPower);
            if (front_right_motor != null) front_right_motor.setPower(-turnPower);
            if (back_left_motor != null) back_left_motor.setPower(turnPower);
            if (back_right_motor != null) back_right_motor.setPower(-turnPower);
        }
        
        // Continue turning until we reach the target heading
        while (opModeIsActive()) {
            orientation = imu.getRobotYawPitchRollAngles();
            double currentHeading = orientation.getYaw(AngleUnit.DEGREES);
            
            // Check if we've reached the target (within tolerance)
            double headingError = targetHeading - currentHeading;
            
            // Handle wraparound
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;
            
            if (Math.abs(headingError) < 2.0) { // 2 degree tolerance
                break;
            }
            
            updateIMUTelemetry();
            telemetry.addData("Turning", "%.1f degrees", degrees);
            telemetry.addData("Target Heading", "%.1f", targetHeading);
            telemetry.addData("Current Heading", "%.1f", currentHeading);
            telemetry.addData("Heading Error", "%.1f", headingError);
            telemetry.update();
            sleep(10);
        }
        
        // Stop motors
        stopDriveMotors();
    }
    
    /**
     * Reset all drive motor encoders
     */
    private void resetEncoders() {
        if (front_left_motor != null) front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (front_right_motor != null) front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (back_left_motor != null) back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (back_right_motor != null) back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Small delay to allow reset to complete
        sleep(50);
        
        // Set back to run using encoder mode
        if (front_left_motor != null) front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (front_right_motor != null) front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (back_left_motor != null) back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (back_right_motor != null) back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Set all drive motors to run to position mode
     */
    private void setRunToPositionMode() {
        if (front_left_motor != null) front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (front_right_motor != null) front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (back_left_motor != null) back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (back_right_motor != null) back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    /**
     * Stop all drive motors
     */
    private void stopDriveMotors() {
        if (front_left_motor != null) front_left_motor.setPower(0);
        if (front_right_motor != null) front_right_motor.setPower(0);
        if (back_left_motor != null) back_left_motor.setPower(0);
        if (back_right_motor != null) back_right_motor.setPower(0);
    }
    
    /**
     * Update IMU telemetry with current values
     * This is the IMU data that will be displayed on the Driver Station
     */
    private void updateIMUTelemetry() {
        if (imu != null) {
            try {
                YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                
                // Display IMU data in simple format
                telemetry.addData("IMU Heading", "%.2f", angles.getYaw(AngleUnit.DEGREES));
                telemetry.addData("IMU Pitch", "%.2f", angles.getPitch(AngleUnit.DEGREES));
                telemetry.addData("IMU Roll", "%.2f", angles.getRoll(AngleUnit.DEGREES));
            } catch (Exception e) {
                telemetry.addData("IMU Error", e.getMessage());
            }
        } else {
            telemetry.addData("IMU", "Not initialized");
        }
    }
}