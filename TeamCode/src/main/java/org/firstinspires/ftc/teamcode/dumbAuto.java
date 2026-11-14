package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name="dumbAuto")
public class dumbAuto extends LinearOpMode {


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

        BL.setDirection(REVERSE);
        FL.setDirection(REVERSE);

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


                BL.setTargetPosition(13536); //1.5 feet
                FL.setTargetPosition(13536);
                BR.setTargetPosition(13536);
                FR.setTargetPosition(13536);

                BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                BL.setPower(1);
                FL.setPower(1);
                FR.setPower(1);
                BR.setPower(1);


            }
        }
    }


}
