/*
    package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.navigation.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.library.controller.PIDFController;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.naviagation.Pose2D;
import org.firstinpsires.ftc.teamcode.utilities.ShooterConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimpleDirection;





public class ShooterSubsystem extends  {

    private Servo hoodServo;;
    private DcMotorEx shooterMotor;
    private PIDFController velocityPIDfController;
    public static double  GATE_SERVO_CLOSED_POSITION = 0.0;
    public static double GATE_SERVO_OPEN_POSITION = 0.0;
    public static double HOOD_SERVO_UP_POSITION = 0.0;
    public static double HOOD_SERVO_DOWN_POSITION = 0.0;

    private Pose2D currentPose;
    private AngleUnit angleUnit;
    private double targetX, targetY, targetAngle;
    private double target_Position




    public Shooter(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection



        velocityPIDfController = new PIDFController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);
        velocityPIDFController.setTolerance(100);
        
    }

    public onInit() {
    shooterMotor.setPower(0.0);
    hoodServo.setPosition();




    }

    public void setHoodServoPosition(double position, AngleUnit angleUnit) {

        hoodServos.setHoodServoPosition(position);
        HoodToAngleUnit(angleUnit);


    }


    public void setVelocityPoint(double target_Position) {
    }

    public double get HoodServoPosition() {
       return hoodServo.getPosition();

}

 */
