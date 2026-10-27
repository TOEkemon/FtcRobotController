/*
    package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;



@TeleOp
(name = "MecanumDrive")

public class MecanumDrive {

    private DcMotor FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor;

    private IMU imu;

    private double FrontLeftPower;
    private double FrontRightPower;
    private double BackRightPower;
    private double BackLeftPower;

    private double RevOrientation;

    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double KA = 0;
    public static double KStatic = 0;

    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO =
    public static double TRACK_WIDTH =

    public static final double MinPower = 0;
    public static final double MaxPower = 0;

    public static final double TICK_PER_INCH =

    public static final double MAX_RPM = 1;

    public static final double TICKS_PER_REV = 1;

    public static double MAX_VEL = (Max rpm * gear ratio * wheel ratio * 2pi);
    public static double MIN_VEL =

    public static double TURN_SPEED = 0.80;


    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0, 0, 0,
           getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));





public init(HardwareMap hwMap) {
    // initialize the variables
   FrontLeftMotor = hwMap.get(DcMotor.class, "front_left_motors");
   FrontRightMotor = hwMap.get(DcMotor.class, "front_right_motors");
   BackLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");
   BackRightMotor = hwMap.get(DcMotor.class, "back_right_motor");


   BackRightMotor.setDirection(Direction.REVERSE);
   BackLeftMotor.setDirection(Direction.REVERSE);
   FrontLeftMotor.setDirection(Direction.FORWARD);
   FrontRightMotor.setDirection(Direction.FORWARD);


   FrontLeftMotor.setMode(RunMode.RUN_USING_ENCODER);
   FrontRightMotor.setMode(RunMode.RUN_USING_ENCODER);
   BackRightMotor.setMode(RunMode.RUN_USING_ENCODER);
   BackRightMotor.setMode(RunMode.RUN_USING_ENCODER);



   IMU imu = hwMap.get(IMU.class, "imu");

   RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
    RevHubOrientationOnRobot.LogoFacingDirection.UP,
    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);



 imu.initialize(new IMU.Parameters(RevOrientation);


}


public void Drive(double Left_Y, double Left_X, double Right_X, double Current_Speed) {

    double forward = Left_Y * Current_Speed;
    double turn = Right_X * (1.0 - forward) * TURN_SPEED;
    double strafe = Left_X * Current_Speed;

    FrontLeftPower = forward + strafe + turn;
    FrontRightPower = forward - strafe - turn;
    BackLeftPower = forward - strafe + turn;
    BackRightPower = forward + strafe - turn;



}


public void scaleMotorPower() {


    if (Math.abs( FrontLeftPower) > MaxPower) {
        MaxPower = FrontLeftPower;

    }

    if (Math.abs( FrontRightPower) > MaxPower) {
        MaxPower = FrontRightPower;

    }

    if (Math.abs(BackRightPower) > MaxPower) {
        MaxPower = BackRightPower;
    }

    if (Math.abs(BackLeftPower) > MaxPower) {
        MaxPower = BackLeftPower;
    }

    if (MaxPower > 1) {
        scalePower(MaxPower);
}


}

public void scalePower(double ScalePower) {


    FrontRightPower = FrontRightPower / ScalePower;
    FrontLeftPower = FrontLeftPower / ScalePower;
    BackLeftPower = BackLeftPower / ScalePower;
    BackRightPower = BackLeftPower / ScalePower;

}
















public void resetEncoders() {

    FrontLeftMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
    FrontRightMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
    BackLeftMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
    BackRightMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);


}

}

w










}












 */