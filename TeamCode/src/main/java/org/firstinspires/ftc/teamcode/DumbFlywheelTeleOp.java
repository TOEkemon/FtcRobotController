package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class DumbFlywheelTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("drive_front_left");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("drive_back_left");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("drive_front_right");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("drive_back_right");
        Servo turretServo = hardwareMap.servo.get("turret");

        DcMotorEx flywheelMotor = (DcMotorEx) hardwareMap.dcMotor.get("flywheel");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorEx intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        DcMotorEx rampMotor = (DcMotorEx) hardwareMap.dcMotor.get("ramp");
        rampMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        // we need to do the Left motors :)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            //double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            var power = gamepad2.left_trigger - gamepad2.right_trigger;
            if (gamepad2.a) intakeMotor.setPower(power);
            if (gamepad2.b) rampMotor.setPower(power);
            if (gamepad2.x) flywheelMotor.setPower(power * 0.5);
            turretServo.setPosition((-gamepad2.left_stick_x));


            /*
            double targetRpm = (gamepad1.left_trigger - gamepad1.right_trigger) * 4000;
            intakeMotor.setVelocity(targetRpm * 6, AngleUnit.DEGREES);
            telemetry.addLine("ehgwfuh");
            telemetry.addData("Target RPM:", targetRpm);
            telemetry.addData("Actual RPM:", intakeMotor.getVelocity());
             */
            if (gamepad2.rightBumperWasPressed()) { //stop everything
                flywheelMotor.setPower(0);
                intakeMotor.setPower(0);
                rampMotor.setPower(0);
            }
            if (gamepad2.leftBumperWasPressed()) {
                flywheelMotor.setPower(power);
                rampMotor.setPower(power);
                intakeMotor.setPower(power);

            }
        }
    }
}