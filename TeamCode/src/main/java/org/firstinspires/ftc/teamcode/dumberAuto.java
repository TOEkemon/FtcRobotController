package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name="dumberAuto")
public class dumberAuto extends LinearOpMode {


    private DcMotorEx BL;
    private DcMotorEx BR;
    private DcMotorEx FL;
    private DcMotorEx FR;

    @Override
    public void runOpMode() {

        BL = hardwareMap.get(DcMotorEx.class, "drive_back_left");
        BR = hardwareMap.get(DcMotorEx.class, "drive_back_right");
        FL = hardwareMap.get(DcMotorEx.class, "drive_front_left");
        FR = hardwareMap.get(DcMotorEx.class, "drive_front_right");



        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {




                BL.setPower(1);
                FL.setPower(1);
                FR.setPower(1);
                BR.setPower(1);

                sleep(1000);

                BL.setPower(0);
                FL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);


            }
        }
    }


}
