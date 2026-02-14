package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Coordinate Tester", group = "Testing")
public class coordinate_tester extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    // Variables to track button states
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean yPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;

    // Field constants for DECODE game
    private static class FieldConstants {
        // Launch Line positions (where robots start)
        public static final double RED_LAUNCH_LINE_X = -60.0; // X position for red alliance
        public static final double RED_LAUNCH_LINE_Y_NEAR = 60.0;  // Y position for red alliance near side
        public static final double RED_LAUNCH_LINE_Y_FAR = 45.0;   // Y position for red alliance far side
        public static final double BLUE_LAUNCH_LINE_X = 60.0;  // X position for blue alliance (mirrored)
        public static final double BLUE_LAUNCH_LINE_Y_NEAR = -60.0; // Y position for blue alliance near side
        public static final double BLUE_LAUNCH_LINE_Y_FAR = -45.0;  // Y position for blue alliance far side

        // Spike Mark positions (for ARTIFACT collection)
        // Near (Audience side)
        public static final double NEAR_SPIKE_RED_X = -10.0;
        public static final double NEAR_SPIKE_RED_Y = 50.0;
        public static final double NEAR_SPIKE_BLUE_X = 10.0;
        public static final double NEAR_SPIKE_BLUE_Y = -50.0;

        // Far (GOAL side)
        public static final double FAR_SPIKE_RED_X = -10.0;
        public static final double FAR_SPIKE_RED_Y = -50.0;
        public static final double FAR_SPIKE_BLUE_X = 10.0;
        public static final double FAR_SPIKE_BLUE_Y = 50.0;

        // Middle (Center)
        public static final double MIDDLE_SPIKE_X = 0.0;
        public static final double MIDDLE_SPIKE_Y = 0.0;

        // BASE ZONE positions
        public static final double RED_BASE_X_MIN = -72.0;
        public static final double RED_BASE_Y_MIN = 54.0;
        public static final double RED_BASE_X_MAX = -54.0;
        public static final double RED_BASE_Y_MAX = 72.0;

        public static final double BLUE_BASE_X_MIN = 54.0;
        public static final double BLUE_BASE_Y_MIN = -72.0;
        public static final double BLUE_BASE_X_MAX = 72.0;
        public static final double BLUE_BASE_Y_MAX = -54.0;
    }

    @Override
    public void runOpMode() {
        // Initialize hardware
        try {
            frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
            frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
            backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
            backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
        } catch (Exception e) {
            telemetry.addData("Error", "Hardware mapping failed: " + e.getMessage());
            telemetry.update();
            return; // Exit if hardware mapping fails
        }

        // Configure motors
        if (backLeftMotor != null) backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        if (frontLeftMotor != null) frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to brake mode for better control
        if (backLeftMotor != null) backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (frontLeftMotor != null) frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backRightMotor != null) backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (frontRightMotor != null) frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Coordinate Tester Initialized");
        telemetry.addData("Instructions", "Use D-Pad to move to preset positions");
        telemetry.addData("A", "Red-Near Launch Line");
        telemetry.addData("B", "Blue-Far Launch Line");
        telemetry.addData("X", "Red-Far Launch Line");
        telemetry.addData("Y", "Blue-Near Launch Line");
        telemetry.addData("D-Pad Up", "Near Spike Red");
        telemetry.addData("D-Pad Down", "Far Spike Red");
        telemetry.addData("D-Pad Left", "Middle Spike");
        telemetry.addData("D-Pad Right", "Red Base Zone Center");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Read joystick values for manual driving
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

            // Set motor powers for manual driving
            if (frontLeftMotor != null) frontLeftMotor.setPower(frontLeftPower);
            if (backLeftMotor != null) backLeftMotor.setPower(rearLeftPower);
            if (frontRightMotor != null) frontRightMotor.setPower(frontRightPower);
            if (backRightMotor != null) backRightMotor.setPower(rearRightPower);

            // Handle preset position movements using buttons
            // A button: Red-Near Launch Line
            if (gamepad1.a && !aPressed) {
                moveRobotToPosition(FieldConstants.RED_LAUNCH_LINE_X, FieldConstants.RED_LAUNCH_LINE_Y_NEAR);
                aPressed = true;
            } else if (!gamepad1.a) {
                aPressed = false;
            }

            // B button: Blue-Far Launch Line
            if (gamepad1.b && !bPressed) {
                moveRobotToPosition(FieldConstants.BLUE_LAUNCH_LINE_X, FieldConstants.BLUE_LAUNCH_LINE_Y_FAR);
                bPressed = true;
            } else if (!gamepad1.b) {
                bPressed = false;
            }

            // X button: Red-Far Launch Line
            if (gamepad1.x && !xPressed) {
                moveRobotToPosition(FieldConstants.RED_LAUNCH_LINE_X, FieldConstants.RED_LAUNCH_LINE_Y_FAR);
                xPressed = true;
            } else if (!gamepad1.x) {
                xPressed = false;
            }

            // Y button: Blue-Near Launch Line
            if (gamepad1.y && !yPressed) {
                moveRobotToPosition(FieldConstants.BLUE_LAUNCH_LINE_X, FieldConstants.BLUE_LAUNCH_LINE_Y_NEAR);
                yPressed = true;
            } else if (!gamepad1.y) {
                yPressed = false;
            }

            // D-Pad Up: Near Spike Red
            if (gamepad1.dpad_up && !dpadUpPressed) {
                moveRobotToPosition(FieldConstants.NEAR_SPIKE_RED_X, FieldConstants.NEAR_SPIKE_RED_Y);
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }

            // D-Pad Down: Far Spike Red
            if (gamepad1.dpad_down && !dpadDownPressed) {
                moveRobotToPosition(FieldConstants.FAR_SPIKE_RED_X, FieldConstants.FAR_SPIKE_RED_Y);
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            // D-Pad Left: Middle Spike
            if (gamepad1.dpad_left && !dpadLeftPressed) {
                moveRobotToPosition(FieldConstants.MIDDLE_SPIKE_X, FieldConstants.MIDDLE_SPIKE_Y);
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

            // D-Pad Right: Red Base Zone Center
            if (gamepad1.dpad_right && !dpadRightPressed) {
                double redBaseCenterX = (FieldConstants.RED_BASE_X_MIN + FieldConstants.RED_BASE_X_MAX) / 2.0;
                double redBaseCenterY = (FieldConstants.RED_BASE_Y_MIN + FieldConstants.RED_BASE_Y_MAX) / 2.0;
                moveRobotToPosition(redBaseCenterX, redBaseCenterY);
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }

            // Send telemetry data to driver station
            telemetry.addData("Manual Driving", "Use joysticks to drive manually");
            telemetry.addData("Preset Positions", "Use buttons to move to preset positions");
            telemetry.addData("Current Position", "Estimating based on encoders");
            telemetry.addData("A", "Red-Near Launch (%.1f, %.1f)", FieldConstants.RED_LAUNCH_LINE_X, FieldConstants.RED_LAUNCH_LINE_Y_NEAR);
            telemetry.addData("B", "Blue-Far Launch (%.1f, %.1f)", FieldConstants.BLUE_LAUNCH_LINE_X, FieldConstants.BLUE_LAUNCH_LINE_Y_FAR);
            telemetry.addData("X", "Red-Far Launch (%.1f, %.1f)", FieldConstants.RED_LAUNCH_LINE_X, FieldConstants.RED_LAUNCH_LINE_Y_FAR);
            telemetry.addData("Y", "Blue-Near Launch (%.1f, %.1f)", FieldConstants.BLUE_LAUNCH_LINE_X, FieldConstants.BLUE_LAUNCH_LINE_Y_NEAR);
            telemetry.update();
        }

        // When exiting the loop (op mode ending), ensure all motors are stopped
        if (backLeftMotor != null) backLeftMotor.setPower(0);
        if (frontLeftMotor != null) frontLeftMotor.setPower(0);
        if (frontRightMotor != null) frontRightMotor.setPower(0);
        if (backRightMotor != null) backRightMotor.setPower(0);
    }

    /**
     * Moves the robot to a specific position using mecanum drive
     * @param x Target X coordinate (forward/backward)
     * @param y Target Y coordinate (left/right)
     */
    private void moveRobotToPosition(double x, double y) {
        // This is a simplified movement function using encoder-based movement
        // In real implementation, you'd use encoders and PID control for precise movement

        // For now, just log the intended movement
        telemetry.addData("Moving to", "(%.1f, %.1f)", x, y);
        telemetry.addData("Note", "Movement implementation would go here");
        telemetry.update();
        sleep(1000); // Brief pause to allow operator to see the target
    }
}