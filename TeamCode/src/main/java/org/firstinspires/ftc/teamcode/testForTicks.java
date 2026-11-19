package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name="testForTicks")
public class testForTicks extends LinearOpMode {

    private DcMotorEx BL;
    private DcMotorEx BR;
    private DcMotorEx FL;
    private DcMotorEx FR;

    int flpos;
    int frpos;
    int blpos;
    int brpos;




    @Override
    public void runOpMode() {

        BL = hardwareMap.get(DcMotorEx.class, "drive_back_left");
        BR = hardwareMap.get(DcMotorEx.class, "drive_back_right");
        FL = hardwareMap.get(DcMotorEx.class, "drive_front_left");
        FR = hardwareMap.get(DcMotorEx.class, "drive_front_right");





        BL.setDirection(REVERSE);
        FL.setDirection(REVERSE);
        //BR.setDirection(REVERSE);
        //FR.setDirection(REVERSE);


        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;






                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPow = (y + x + rx) / denominator;
                double backLeftPow = (y - x + rx) / denominator;
                double frontRightPow = (y - x - rx) / denominator;
                double backRightPow = (y + x - rx) / denominator;

                if (gamepad1.xWasPressed()) {
                    FL.setPower(.2);
                    FR.setPower(.2);
                    BL.setPower(.2);
                    BR.setPower(.2);
                }
                if (gamepad1.xWasReleased()) {
                    FL.setPower(0);
                    FR.setPower(0);
                    BL.setPower(0);
                    BR.setPower(0);
                }

                FL.setPower(frontLeftPow / 4);
                FR.setPower(frontRightPow / 4);
                BL.setPower(backLeftPow / 4);
                BR.setPower(backRightPow / 4);

                flpos = FL.getCurrentPosition();
                frpos = FR.getCurrentPosition();
                blpos = BL.getCurrentPosition();
                brpos = BR.getCurrentPosition();

                telemetry.addData("", flpos);
                telemetry.addData("", frpos);
                telemetry.addData("", blpos);
                telemetry.addData("", brpos);
                telemetry.update();

            }
        }
    }
}


