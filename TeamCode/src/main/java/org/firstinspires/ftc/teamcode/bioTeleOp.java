package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;


@TeleOp(name="bioTeleOp")
public class bioTeleOp extends LinearOpMode {

    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;
    private DcMotor FR;

    private CRServo turret;



    private DcMotor intakemotor;

    private DcMotor launcher;

    //for stopping motor seamlessly









    @Override
    public void runOpMode() {

        BL = hardwareMap.get(DcMotor.class, "drive_back_left");
        BR = hardwareMap.get(DcMotor.class, "drive_back_right");
        FL = hardwareMap.get(DcMotor.class, "drive_front_left");
        FR = hardwareMap.get(DcMotor.class, "drive_front_right");
        intakemotor = hardwareMap.get(DcMotor.class, "intake");
        //CRservo fro continuous rotation
        turret = hardwareMap.get(CRServo.class, "turret");
        launcher = hardwareMap.get(DcMotor.class, "flywheel" );






        BL.setDirection(REVERSE);
        FL.setDirection(REVERSE);
        //BR.setDirection(REVERSE);
        //FR.setDirection(REVERSE);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //for turret
                double currentTurret = 0;



                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;






                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPow = (y + x + rx) / denominator;
                double backLeftPow = (y - x + rx) / denominator;
                double frontRightPow = (y - x - rx) / denominator;
                double backRightPow = (y + x - rx) / denominator;

                FL.setPower(frontLeftPow);
                FR.setPower(frontRightPow);
                BL.setPower(backLeftPow);
                BR.setPower(backRightPow);


                if (gamepad1.rightTriggerWasPressed()) {
                    intakemotor.setPower(-1);
                }
                if (gamepad1.leftTriggerWasPressed()) {
                    intakemotor.setPower(1);
                }
                if (gamepad1.rightTriggerWasReleased() || gamepad1.leftTriggerWasReleased()) {
                    intakemotor.setPower(0);
                }




                if (gamepad1.dpadLeftWasPressed()) {
                turret.setPower(1);
                }
                if (gamepad1.dpadRightWasPressed()) {
                turret.setPower(-1);
                }
                if (gamepad1.dpadLeftWasReleased() || gamepad1.dpadRightWasReleased()) {
                    turret.setPower(0);
                }

                if (gamepad1.rightBumperWasPressed()) {
                    launcher.setPower(-.49);

                }
                if (gamepad1.rightBumperWasReleased()) {
                    launcher.setPower(0);

                }
                if (gamepad1.leftBumperWasPressed()) {
                    launcher.setPower(launcher.getPower());
                }
                if (gamepad1.leftBumperWasReleased()) {
                    launcher.setPower(0);
                }
                
                // timing of the controls; seamless connection
                telemetry.addData("launcher", launcher.getPower());
                telemetry.update();




            }
        }
    }
}


                
