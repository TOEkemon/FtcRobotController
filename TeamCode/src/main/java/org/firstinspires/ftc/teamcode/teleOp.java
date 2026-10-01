package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="teleOp")
public class teleOp extends LinearOpMode {

    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor LFL;
    private DcMotor LFR;

    private Servo Push;

    private Servo Plane;

    @Override
    public void runOpMode() {

        BL = hardwareMap.get(DcMotor.class, "motor_back_left");
        BR = hardwareMap.get(DcMotor.class, "motor_back_right");
        FL = hardwareMap.get(DcMotor.class, "motor_front_left");
        FR = hardwareMap.get(DcMotor.class, "motor_front_right");
        LFL = hardwareMap.get(DcMotor.class, "lift_front_left");
        LFR = hardwareMap.get(DcMotor.class, "lift_front_right");
        Push = hardwareMap.get(Servo.class, "pusher");
        Plane = hardwareMap.get(Servo.class, "plane");


        LFR.setDirection(REVERSE);

        BL.setDirection(REVERSE);
        FL.setDirection(REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;


                double runtime = getRuntime();

                double currentPushPosition = 0;
                double currentPlanePosition = 0;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPow = (y + x + rx) / denominator;
                double backLeftPow = (y - x + rx) / denominator;
                double frontRightPow = (y - x - rx) / denominator;
                double backRightPow = (y + x - rx) / denominator;

                FL.setPower(frontLeftPow);
                FR.setPower(frontRightPow);
                BL.setPower(backLeftPow);
                BR.setPower(backRightPow);



                //Arm lift motors
                if (gamepad1.aWasPressed()) {
                    LFL.setPower(.6);
                    LFR.setPower(.6);
                } else if (gamepad1.aWasReleased()) {
                    LFL.setPower(0);
                    LFR.setPower(0);
                }
                if (gamepad1.bWasPressed()){
                    LFL.setPower(-.6);
                    LFR.setPower(-.6);
                } else if (gamepad1.bWasReleased()) {
                    LFL.setPower(0);
                    LFR.setPower(0);
                }
                //keeps track of pixel push servo and moves it
                if (gamepad1.xWasPressed()) {
                    Push.setPosition(currentPushPosition+.5);
                    currentPushPosition += .5;
                    telemetry.addData("currentPushPosition (intended updated) ", currentPushPosition);
                }

                if (gamepad1.yWasPressed()){
                    Push.setPosition(currentPushPosition-.5);
                    currentPushPosition -=.5;
                    telemetry.addData("currentPushPosition (intended updated) ", currentPushPosition);
                }
                //plane throwing servo
                if (gamepad1.dpadUpWasPressed()) {
                    Plane.setPosition(currentPlanePosition+.5);
                    currentPlanePosition += .5;
                    telemetry.addData("currentPlanePosition (intended updated ", currentPlanePosition);
                }
                if (gamepad1.dpadDownWasPressed()) {
                    Plane.setPosition(currentPlanePosition+.5);
                    currentPlanePosition += .5;
                    telemetry.addData("currentPlanePosition intended updated ", currentPlanePosition);
                }

                telemetry.update();


            }
        }
    }
}
