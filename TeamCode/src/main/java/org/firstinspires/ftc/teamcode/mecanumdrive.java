/*
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;

public class MecanumDrive {

   public static class Params {
   
   RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
    RevHubOrientationOnRobot.LogoFacingDirection.UP,
    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

    public double InPerTicks = 1;
    public double LateralPerTicks = InPerTicks;
    public double WidthInTracks = 0;

    public double kA = 0;
    public double kS = 0;
    public double kV = 0;

    
    
    }

    public Params PARAMs = new Params();

    public MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

            
   

    private DcMotorEx FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor;

    private IMU imu;

    private double FrontLeftPower;
    private double FrontRightPower;
    private double BackRightPower;
    private double BackLeftPower;

    private double orientation;

    public static final double MinPower = 0;
    public static final double MaxPower = 0;

    public static double TURN_SPEED = 0.80;






public void init(HardwareMap hwMap) {
    // initialize the variables
   FrontLeftMotor = hwMap.get(DcMotorEx.class, "FL");
   FrontRightMotor = hwMap.get(DcMotorEx.class, "FR");
   BackLeftMotor = hwMap.get(DcMotorEx.class, "BL");
   BackRightMotor = hwMap.get(DcMotorEx.class, "BR");


   BackRightMotor.setDirection(SimpleMotor.Direction.REVERSE);
   BackLeftMotor.setDirection(SimpleMotor.Direction.REVERSE);
   FrontLeftMotor.setDirection(SimpleMotor.Direction.FORWARD);
   FrontRightMotor.setDirection(SimpleMotor.Direction.FORWARD);


   FrontLeftMotor.setMode(RunMode.RUN_USING_ENCODER);
   FrontRightMotor.setMode(RunMode.RUN_USING_ENCODER);
   BackRightMotor.setMode(RunMode.RUN_USING_ENCODER);
   BackLeftMotor.setMode(RunMode.RUN_USING_ENCODER);



   this.imu = hwMap.get(IMU.class, "imu");
   

   imu.initialize( new IMU.Parameter(orientation));


}


public void Drive(double Left_Y, double Left_X, double Right_X, double Current_Speed) {


   if (!Double.isFinite(Left_Y)
            || !Double.isFinite(Left_X)
            || !Double.isFinite(Right_X)) {
        throw new IllegalArgumentException(
                "Drive inputs must be finite."
        );
    }

   if (!(Current_Speed >= 0.0 && Current_Speed <= 1.0)) {
        throw new IllegalArgumentException(
                "Current_Speed must be between 0 and 1."
        );
    }

    double forward = Left_Y;
    double turn = Left_X;
    double strafe = Right_X * Current_Speed;

    FrontLeftPower = forward + strafe + turn;
    FrontRightPower = forward - strafe - turn;
    BackLeftPower = forward - strafe + turn;
    BackRightPower = forward + strafe - turn;


    scaleMotorPower();

    FrontLeftMotor.setPower(FrontLeftPower * Current_Speed);
    FrontRightMotor.setPower(FrontRightPower * Current_Speed);
    BackRightMotor.setPower(BackRightPower * Current_Speed);
    BackLeftMotor.setPower(BackLeftPower * Current_Speed);



}


public void scaleMotorPower() {
    double maxPower = 1.0;

    maxPower = Math.max(maxPower, Math.abs(FrontLeftPower));
    maxPower = Math.max(maxPower, Math.abs(FrontRightPower));
    maxPower = Math.max(maxPower, Math.abs(BackLeftPower));
    maxPower = Math.max(maxPower, Math.abs(BackRightPower));

    if (!Double.isFinite(maxPower)) {
        throw new IllegalArgumentException(
                "Calculated wheel powers must be finite."
        );
    }

    scalePower(maxPower);
}

public void scalePower(double ScalePower) {
    if (!Double.isFinite(ScalePower) || ScalePower < 1.0) {
        throw new IllegalArgumentException(
                "The normalization divisor must be finite and at least 1."
        );
    }

    FrontLeftPower = FrontLeftPower / ScalePower;
    FrontRightPower = FrontRightPower / ScalePower;
    BackLeftPower = BackLeftPower / ScalePower;
    BackRightPower = BackRightPower / ScalePower;
}



public driveFieldRelative(double forward, double strafe, double turn) {

double theta = Math.atan2(forward, strafe);
double r = Math.hypot(strafe, forward);

double heading = imu.getRobotYawPitchRollAngles()
            .getYaw(AngleUnit.RADIANS);
            
theta = AngleUnit.normalizeRadians(theta - heading);

double robotForward = r * Math.sin(theta);
double robotStrafe = r * Math.cos(theta);

    Drive(robotForward, robotStrafe, turn, 1.0);

}



public void resetEncoders() {
   drive(0,0,0);

    FrontLeftMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
    FrontRightMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
    BackLeftMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
    BackRightMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
    
    FrontLeftMotor.setMode(RunMode.RUN_USING_ENCODER);
    FrontRightMotor.setMode(RunMode.RUN_USING_ENCODER);
    BackLeftMotor.setMode(RunMode.RUN_USING_ENCODER);
    BackRightMotor.setMode(RunMode.RUN_USING_ENCODER);


}



}












 */
